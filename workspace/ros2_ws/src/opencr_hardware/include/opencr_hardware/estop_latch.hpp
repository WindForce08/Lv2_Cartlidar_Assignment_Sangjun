// 잠금식 E-stop 판단 (OpenCRHardware 안, ROS 의존 없음 → 단위 시험)
//   잠김: 조이스틱 heartbeat 무수신(joy_timeout) · E-stop 버튼 · 물리 스위치 · OpenCR 링크 끊김
//   풀림: 리셋 버튼 + 조이스틱 정상 + 물리 스위치 해제 + 링크 정상 + E-stop 버튼 안 누름 (모두 동시에)
//   시작은 잠김 상태 (전원 켠 뒤 리셋 버튼을 눌러야 움직임)
//   trip: 펌웨어 팔 HOLD 진입처럼 한 번 잠그기만 하는 신호 — 원인으로 남지 않아 리셋을 막지 않는다
//         (HOLD 는 REARM 으로만 풀리므로 원인으로 두면 리셋 → REARM 이 순환한다)
#pragma once

#include <string>

namespace opencr_hardware
{

struct EstopInputs
{
  bool joy_required = true;     // false 면 조이스틱 heartbeat 를 보지 않음 (시험 · 조이스틱 없는 벤치)
  bool joy_fresh = false;       // 마지막 /joy 가 joy_timeout 안
  bool estop_button = false;
  bool reset_button = false;
  bool physical_pressed = false;  // STATE ERR_HW_ESTOP
  bool link_ok = false;
  bool trip = false;              // 이번 주기에 한 번 잠금 (STATE ERR_HOLD 상승)
};

class EstopLatch
{
public:
  explicit EstopLatch(bool start_latched = true)
  : latched_(start_latched), reason_(start_latched ? "시작 — 리셋 버튼 필요" : "") {}

  // 매 주기 호출. 반환: 잠김 여부
  bool update(const EstopInputs & in)
  {
    std::string cause;
    if (!in.link_ok) {cause = "OpenCR 링크 끊김";}
    else if (in.physical_pressed) {cause = "물리 E-stop 스위치";}
    else if (in.estop_button) {cause = "조이스틱 E-stop 버튼";}
    else if (in.joy_required && !in.joy_fresh) {cause = "조이스틱 무수신";}

    if (!cause.empty()) {
      if (!latched_) {reason_ = cause;}
      latched_ = true;
      blocking_ = cause;
      return latched_;
    }
    blocking_.clear();
    if (in.trip) {
      if (!latched_) {reason_ = "OpenCR 팔 HOLD";}
      latched_ = true;
      return latched_;   // 같은 주기에는 풀지 않는다
    }
    if (latched_ && (in.reset_button || !in.joy_required)) {
      // joy 를 안 보는 설정에서는 원인이 사라지면 리셋 버튼 없이 풀림
      latched_ = false;
      reason_.clear();
    }
    return latched_;
  }

  bool latched() const {return latched_;}
  const std::string & reason() const {return reason_;}      // 잠긴 원인 (처음 원인)
  const std::string & blocking() const {return blocking_;}  // 지금도 남아 있는 원인 (빈 값이면 리셋 가능)

private:
  bool latched_;
  std::string reason_;
  std::string blocking_;
};

// v5 REARM 요청 판단 (매 주기). 요청 = 리셋 버튼으로 잠금이 풀린 순간 (버튼을 누른 채 원인이 사라진 경우 포함),
// 잠금이 풀린 뒤 리셋 버튼을 새로 누름, 또는 auto_rearm 이면 잠금이 풀린 순간
inline bool rearm_request(bool was_latched, bool now_latched, bool reset_button, bool prev_reset_button, bool auto_rearm)
{
  if (now_latched) {return false;}
  const bool released = was_latched;
  return (released && (reset_button || auto_rearm)) || (reset_button && !prev_reset_button);
}

// v5 REARM 펄스: 잠금이 풀린 상태에서 운용자 재활성화 요청이 있고 팔 보드가 HOLD 를 보고하면
// REARM 을 width 주기 동안 1 로 보낸다. 펌웨어는 0→1 에지만 보므로 요청 한 번 = 재활성화 한 번.
// 잠김 · FAULT 이면 대기 중인 요청을 버린다 (HOLD 진입은 trip 으로 잠그므로 오래된 요청이 남지 않음).
class RearmPulse
{
public:
  explicit RearmPulse(int width = 3)
  : width_(width) {}

  // 매 주기 호출. request: 리셋 버튼 누름 (잠금이 풀린 뒤) 또는 auto_rearm. 반환: 이번 CMD 에 REARM 을 실을지
  bool update(bool latched, bool request, bool arm_hold, bool fault)
  {
    if (latched || fault) {
      pending_ = false;
      left_ = 0;
      return false;
    }
    if (request) {pending_ = true;}
    if (pending_ && arm_hold && left_ == 0) {
      pending_ = false;
      left_ = width_;
    }
    if (left_ > 0) {
      --left_;
      return true;
    }
    return false;
  }

private:
  int width_;
  bool pending_ = false;
  int left_ = 0;
};

}  // namespace opencr_hardware
