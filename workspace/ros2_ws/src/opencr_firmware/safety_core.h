// safety_core.h — 펌웨어 안전 판단 (Firmware 팀). Arduino 의존 없음 → 호스트 gtest 로 역할별 시험.
// opencr_firmware.ino 가 그대로 include 한다. 시간 · 토크 상태 · 위치 유효성은 ino 가 넣어 준다.
//
// CMD 검증 on_cmd(): 역할 · 유한값(NaN · Inf) · 팔 관절 범위 · seq 가 모두 맞아야 받는다.
//   거부한 CMD 는 적용하지 않고 무수신 타이머도 갱신하지 않는다.
//   단 역할이 맞으면 값 · seq 가 틀린 프레임의 ESTOP 비트도 정지 요청으로 받는다 (정지는 안전한 쪽).
//   seq: 직전 승인 seq 보다 1~127 앞선 값만 (같으면 재전송, 뒤면 오래된 명령). 무수신이 되면 기준을 지운다.
//   바퀴 속도 범위 초과는 거부하지 않는다 — ino 가 ±WHEEL_VEL_LIMIT_RAW 로 자른다 (D6 미결).
// 팔 HOLD (v5): 토크를 켠 순간, 그리고 토크 ON 중 무수신 · ESTOP · 토크 OFF 요청이면 HOLD.
//   HOLD 동안 새 목표를 적용하지 않는다 (ino 가 무수신이면 마지막 승인 목표, ESTOP · 토크 OFF 요청이면 현재 위치를 유지).
//   REARM 상승 에지 + ESTOP 없음 + 무수신 아님 + 전 관절 위치 유효 + FAULT 아님 → ACTIVE.
//   통신 복구나 ESTOP 해제만으로는 풀리지 않는다. 부팅 · 무수신 뒤에는 REARM 0 을 본 다음의 1 만 에지로 친다.
// 팔 토크 OFF (D5-4): TORQUE_ON 없는 정상 CMD 가 arm_torque_off_cmds 번 연속 + HOLD 일 때만 허용.
//   CMD 하나 (flags=0 기본값 등) 로는 팔 토크가 꺼지지 않는다. 바퀴는 즉시 허용.
// 바퀴: HOLD 래치 없음 — 무수신 · ESTOP 동안 0, 풀리면 다시 명령을 따른다 (D3 미결, v4 와 같은 동작).
// FAULT: 부팅 점검 실패 (래치, 재부팅으로만 해제). 토크를 켜지 않고 REARM 도 받지 않는다.
// GripGuard (2026-10-27 arm_bench): 그리퍼 칸이 막히면 목표를 "막힌 자리 + 쥐는 힘" 으로 고정 (아래 클래스 설명).
#ifndef OPENCR_SAFETY_CORE_H_
#define OPENCR_SAFETY_CORE_H_

#include <stdint.h>
#include <string.h>

#include "opencr_protocol.h"

namespace opencr_safety
{

enum Reject : uint8_t { ACCEPTED = 0, BAD_ROLE, NOT_FINITE, OUT_OF_RANGE, BAD_SEQ };

// NaN · ±Inf 판별 (지수 비트가 모두 1). -ffast-math 로 빌드해도 사라지지 않게 비트로 본다.
inline bool finite(float v)
{
  uint32_t b;
  memcpy(&b, &v, sizeof(b));
  return (b & 0x7F800000u) != 0x7F800000u;
}

class SafetyCore
{
public:
  // arm_min · arm_max: 팔 관절 한계 (rad, ARM_JOINTS 개). arm_torque_off_cmds: 팔 토크 OFF 에 필요한 연속 CMD 수
  SafetyCore(uint8_t role, const float * arm_min, const float * arm_max, uint32_t timeout_ms, uint16_t arm_torque_off_cmds)
  : role_(role), arm_min_(arm_min), arm_max_(arm_max), timeout_ms_(timeout_ms), arm_torque_off_cmds_(arm_torque_off_cmds) {}

  // CRC · 길이를 통과한 CMD 프레임 하나
  Reject on_cmd(const opencr_proto::Cmd & c, uint32_t now_ms)
  {
    const Reject r = check(c);
    if (r == BAD_ROLE) {role_mismatch_ = true;}
    if (r != ACCEPTED) {
      if (r != BAD_ROLE && (c.flags & opencr_proto::FLAG_ESTOP)) {estop_req_ = true;}
      return r;
    }
    role_mismatch_ = false;
    have_seq_ = true;
    last_seq_ = c.seq;
    estop_req_ = (c.flags & opencr_proto::FLAG_ESTOP) != 0;
    const bool rearm = (c.flags & opencr_proto::FLAG_REARM) != 0;
    if (rearm && !last_rearm_) {rearm_edge_ = true;}
    last_rearm_ = rearm;
    if (c.flags & opencr_proto::FLAG_TORQUE_ON) {
      torque_off_run_ = 0;
    } else if (torque_off_run_ < 0xFFFF) {
      ++torque_off_run_;
    }
    cmd_ = c;
    have_cmd_ = true;
    last_ok_ms_ = now_ms;
    return ACCEPTED;
  }

  // 제어 주기마다 명령 적용 전에. hw_estop = 물리 핀, positions_valid = 이번 SyncRead 에서 전 관절 응답
  void tick(uint32_t now_ms, bool hw_estop, bool torque_on, bool positions_valid)
  {
    timeout_ = !have_cmd_ || (uint32_t)(now_ms - last_ok_ms_) > timeout_ms_;
    estop_ = hw_estop || estop_req_;
    if (timeout_) {
      have_seq_ = false;    // 호스트가 다시 시작해 seq 가 처음부터 와도 받는다
      last_rearm_ = true;   // 끊기기 전부터 1 이던 REARM 을 에지로 보지 않는다
      torque_off_run_ = 0;  // 토크 OFF 는 끊김 없이 이어진 요청만 센다
    }
    const bool torque_off_req = have_cmd_ && !(cmd_.flags & opencr_proto::FLAG_TORQUE_ON);
    entered_stop_hold_ = false;
    if (role_ == opencr_proto::ROLE_ARM && torque_on) {
      if (!hold_ && (timeout_ || estop_ || torque_off_req)) {
        hold_ = true;
        entered_stop_hold_ = estop_ || torque_off_req;
      } else if (hold_ && rearm_edge_ && !timeout_ && !estop_ && positions_valid && !fault_) {
        hold_ = false;
      }
    }
    rearm_edge_ = false;   // 조건이 안 맞아 쓰지 못한 에지는 버린다 — 다시 0→1 이 필요
  }

  // 팔은 토크를 켠 순간 현재 위치를 붙잡고 REARM 을 기다린다. 바퀴는 바로 명령을 따른다
  void on_torque_on() {hold_ = role_ == opencr_proto::ROLE_ARM; rearm_edge_ = false;}
  void on_torque_off() {hold_ = false;}
  void set_fault() {fault_ = true;}

  // 새 목표를 적용해도 되는가 (토크 ON 여부는 ino 가 따로 본다)
  bool apply_targets() const
  {
    if (timeout_ || estop_) {return false;}
    return role_ != opencr_proto::ROLE_ARM || !hold_;
  }

  // 마지막 승인 CMD 의 토크 OFF 요청을 따라도 되는가 (D5-4)
  bool torque_off_allowed() const
  {
    if (!have_cmd_ || (cmd_.flags & opencr_proto::FLAG_TORQUE_ON)) {return false;}
    if (role_ != opencr_proto::ROLE_ARM) {return true;}
    return hold_ && torque_off_run_ >= arm_torque_off_cmds_;
  }

  uint8_t status_bits() const
  {
    uint8_t e = 0;
    if (timeout_) {e |= opencr_proto::ERR_CMD_TIMEOUT;}
    if (role_mismatch_) {e |= opencr_proto::ERR_ROLE_MISMATCH;}
    if (hold_) {e |= opencr_proto::ERR_HOLD;}
    if (fault_) {e |= opencr_proto::ERR_FAULT;}
    return e;
  }

  bool have_cmd() const {return have_cmd_;}
  const opencr_proto::Cmd & cmd() const {return cmd_;}
  bool timeout() const {return timeout_;}
  bool estop() const {return estop_;}
  bool hold() const {return hold_;}
  bool fault() const {return fault_;}
  bool entered_stop_hold() const {return entered_stop_hold_;}   // 이번 주기에 ESTOP · 토크 OFF 요청으로 HOLD 진입

private:
  Reject check(const opencr_proto::Cmd & c) const
  {
    if (c.role != role_) {return BAD_ROLE;}
    if (role_ == opencr_proto::ROLE_WHEEL) {
      for (int i = 0; i < opencr_proto::MAX_WHEELS; ++i) {
        if (!finite(c.wheel_vel[i])) {return NOT_FINITE;}
      }
    } else {
      for (int i = 0; i < opencr_proto::ARM_JOINTS; ++i) {
        if (!finite(c.arm_pos[i])) {return NOT_FINITE;}
        if (c.arm_pos[i] < arm_min_[i] || c.arm_pos[i] > arm_max_[i]) {return OUT_OF_RANGE;}
      }
      if ((c.flags & opencr_proto::FLAG_GRIPPER_VALID) && !finite(c.gripper_pos)) {return NOT_FINITE;}
    }
    if (have_seq_) {
      const uint8_t d = (uint8_t)(c.seq - last_seq_);
      if (d == 0 || d > 127) {return BAD_SEQ;}
    }
    return ACCEPTED;
  }

  uint8_t role_;
  const float * arm_min_;
  const float * arm_max_;
  uint32_t timeout_ms_;
  uint16_t arm_torque_off_cmds_;

  opencr_proto::Cmd cmd_{};
  bool have_cmd_ = false;
  uint32_t last_ok_ms_ = 0;
  bool have_seq_ = false;
  uint8_t last_seq_ = 0;
  bool last_rearm_ = true;
  bool rearm_edge_ = false;
  bool role_mismatch_ = false;
  bool estop_req_ = false;        // 마지막 승인 CMD 의 ESTOP, 또는 그 뒤 거부된 프레임의 ESTOP
  uint16_t torque_off_run_ = 0;   // TORQUE_ON 없는 승인 CMD 연속 수
  bool timeout_ = true;
  bool estop_ = false;
  bool hold_ = false;
  bool entered_stop_hold_ = false;
  bool fault_ = false;
};

// 그리퍼 막힘 보호 (2026-10-27 arm_bench). 단위 tick. ino 가 제어 주기마다 그리퍼 칸의 이번 목표 · 현재 위치를 넣고, 돌려준 목표를 쓴다.
//   막힘 = 목표가 현재보다 stall 넘게 앞서 있는데 stall_ms 동안 progress 도 못 움직임 (물체를 잡았거나 끼임).
//     빈 손으로 먼 목표에 가는 중에는 계속 움직이므로 막힘이 아니다 (오차가 커도).
//   → 목표 = 현재 + squeeze (목표 쪽, lo ~ hi 안) 로 고정한다. 들어오는 목표가 그 자리보다 덜 가는 쪽이면 (보통 열기) 풀고 따른다.
//   토크 ON · OFF 때 reset. 위치를 못 읽은 주기에는 판정하지 않고 고정 목표만 유지한다.
class GripGuard
{
public:
  GripGuard(int32_t stall, int32_t progress, uint32_t stall_ms, int32_t squeeze, int32_t lo, int32_t hi)
  : stall_(stall), progress_(progress), stall_ms_(stall_ms), squeeze_(squeeze), lo_(lo < hi ? lo : hi), hi_(lo < hi ? hi : lo) {}

  int32_t update(int32_t goal, int32_t pos, bool pos_valid, uint32_t now_ms)
  {
    if (latched_) {
      if ((goal - hold_) * dir_ >= 0) {return hold_;}   // 여전히 막힌 쪽으로 → 쥔 목표 유지
      latched_ = false;                                // 반대쪽 → 풀고 그 목표를 따름
    }
    if (!pos_valid) {watching_ = false; return goal;}
    const int32_t err = goal - pos;
    if (err <= stall_ && err >= -stall_) {watching_ = false; return goal;}
    if (!watching_ || pos - anchor_ > progress_ || anchor_ - pos > progress_) {
      watching_ = true;   // 처음 보거나 아직 움직이는 중 → 이 자리부터 다시 잰다
      anchor_ = pos;
      since_ms_ = now_ms;
      return goal;
    }
    if ((uint32_t)(now_ms - since_ms_) < stall_ms_) {return goal;}
    dir_ = err > 0 ? 1 : -1;
    const int32_t h = pos + dir_ * squeeze_;
    hold_ = h < lo_ ? lo_ : (h > hi_ ? hi_ : h);
    latched_ = true;
    watching_ = false;
    return hold_;
  }

  void reset() {latched_ = false; watching_ = false;}
  bool latched() const {return latched_;}
  int32_t hold() const {return hold_;}

private:
  int32_t stall_;
  int32_t progress_;
  uint32_t stall_ms_;
  int32_t squeeze_;
  int32_t lo_;
  int32_t hi_;
  bool latched_ = false;
  bool watching_ = false;
  int8_t dir_ = 1;
  int32_t hold_ = 0;
  int32_t anchor_ = 0;
  uint32_t since_ms_ = 0;
};

}  // namespace opencr_safety

#endif  // OPENCR_SAFETY_CORE_H_
