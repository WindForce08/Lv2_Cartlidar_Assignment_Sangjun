// 상태머신 공통 (v1.2 §2 규칙): enum class + 허용 전이표 + 상태를 바꾸는 함수 하나.
// 허용되지 않은 전이는 거부하고 이유를 남긴다. ROS 의존 없음 → 단위 시험 가능.
#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <set>
#include <string>
#include <utility>

namespace control
{

template<typename E>
class Fsm
{
public:
  using Listener = std::function<void (E from, E to, const std::string & reason)>;
  using Rejector = std::function<void (E from, E to, const std::string & reason)>;

  explicit Fsm(E initial)
  : state_(initial) {}

  void allow(E from, std::initializer_list<E> tos)
  {
    for (E to : tos) {allowed_.insert({key(from), key(to)});}
  }

  bool can(E to) const {return allowed_.count({key(state_), key(to)}) > 0;}

  // 유일한 상태 변경 함수
  bool transition(E to, const std::string & reason = "")
  {
    const E from = state_;
    if (!can(to)) {
      if (on_reject_) {on_reject_(from, to, reason);}
      return false;
    }
    state_ = to;
    reason_ = reason;
    if (on_change_) {on_change_(from, to, reason);}
    return true;
  }

  E state() const {return state_;}
  const std::string & reason() const {return reason_;}
  void on_change(Listener f) {on_change_ = std::move(f);}
  void on_reject(Rejector f) {on_reject_ = std::move(f);}

private:
  static uint32_t key(E e) {return static_cast<uint32_t>(e);}
  E state_;
  std::string reason_;
  std::set<std::pair<uint32_t, uint32_t>> allowed_;
  Listener on_change_;
  Rejector on_reject_;
};

// ---- 미션 상태 (mission_node, MissionStatus.state 와 같은 값) ----
//   PAUSED: E-stop · 진단 오류로 goal 을 모두 취소하고 멈춤 → 시작 버튼으로 현재 단계부터 재개 (자동 재개 없음)
enum class MissionState : uint8_t { IDLE = 0, RUNNING, PAUSED, DONE, FAILED };

inline const char * to_string(MissionState s)
{
  static const char * n[] = {"IDLE", "RUNNING", "PAUSED", "DONE", "FAILED"};
  return n[static_cast<int>(s)];
}

inline Fsm<MissionState> make_mission_fsm()
{
  using S = MissionState;
  Fsm<S> f(S::IDLE);
  f.allow(S::IDLE, {S::RUNNING});
  f.allow(S::RUNNING, {S::PAUSED, S::DONE, S::FAILED, S::IDLE});
  f.allow(S::PAUSED, {S::RUNNING, S::IDLE});
  f.allow(S::DONE, {S::RUNNING, S::IDLE});
  f.allow(S::FAILED, {S::RUNNING, S::IDLE});
  return f;
}

// ---- 팔 스킬 실행 (manipulator_node, v1.2 §3) ----
enum class SkillState : uint8_t { IDLE = 0, PRECHECK, RUNNING, VERIFY, HOLD, DONE, FAILED, REJECTED };

inline const char * to_string(SkillState s)
{
  static const char * n[] = {"IDLE", "PRECHECK", "RUNNING", "VERIFY", "HOLD", "DONE", "FAILED", "REJECTED"};
  return n[static_cast<int>(s)];
}

inline Fsm<SkillState> make_skill_fsm()
{
  using S = SkillState;
  Fsm<S> f(S::IDLE);
  f.allow(S::IDLE, {S::PRECHECK});
  f.allow(S::PRECHECK, {S::RUNNING, S::REJECTED});
  f.allow(S::RUNNING, {S::VERIFY, S::HOLD});      // 성공 · timeout → VERIFY, 안전 위반 · cancel · 실행 오류 → HOLD
  f.allow(S::VERIFY, {S::DONE, S::FAILED});
  f.allow(S::HOLD, {S::FAILED});
  f.allow(S::DONE, {S::IDLE});
  f.allow(S::FAILED, {S::IDLE});
  f.allow(S::REJECTED, {S::IDLE});
  return f;
}

// ---- 팔 안전 래치 (manipulator_node, v1.2 §4) ----
enum class SafetyState : uint8_t { NORMAL = 0, HOLD, ESTOP };

inline const char * to_string(SafetyState s)
{
  static const char * n[] = {"NORMAL", "HOLD", "ESTOP"};
  return n[static_cast<int>(s)];
}

// ESTOP 은 래치: 운용자 reset 으로만 NORMAL. HOLD 는 새 action + 자세 정상일 때 NORMAL.
class SafetyLatch
{
public:
  SafetyLatch()
  : fsm_(SafetyState::NORMAL)
  {
    using S = SafetyState;
    fsm_.allow(S::NORMAL, {S::HOLD, S::ESTOP});
    fsm_.allow(S::HOLD, {S::NORMAL, S::ESTOP});
    fsm_.allow(S::ESTOP, {S::NORMAL});
  }

  SafetyState state() const {return fsm_.state();}
  const std::string & reason() const {return fsm_.reason();}
  Fsm<SafetyState> & fsm() {return fsm_;}

  bool hold(const std::string & why)
  {
    return fsm_.state() == SafetyState::NORMAL && fsm_.transition(SafetyState::HOLD, why);
  }
  bool estop(const std::string & why)
  {
    return fsm_.state() != SafetyState::ESTOP && fsm_.transition(SafetyState::ESTOP, why);
  }
  // 새 action 이 왔을 때: HOLD 이고 현재 자세가 허용 범위면 NORMAL
  bool release_hold(bool pose_ok)
  {
    return fsm_.state() == SafetyState::HOLD && pose_ok && fsm_.transition(SafetyState::NORMAL, "새 action · 자세 정상");
  }
  // 운용자 /arm/estop_reset
  bool reset_estop(bool estop_input_active, bool joints_fresh, std::string & why)
  {
    if (fsm_.state() != SafetyState::ESTOP) {why = "ESTOP 상태가 아님"; return false;}
    if (estop_input_active) {why = "/e_stop 이 아직 true"; return false;}
    if (!joints_fresh) {why = "관절 상태가 들어오지 않음"; return false;}
    why = "운용자 해제";
    return fsm_.transition(SafetyState::NORMAL, why);
  }

private:
  Fsm<SafetyState> fsm_;
};

}  // namespace control
