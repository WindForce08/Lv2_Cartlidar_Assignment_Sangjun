// 잠금식 E-stop 판단 (OpenCRHardware 안, ROS 의존 없음 → 단위 시험)
//   잠김: 조이스틱 heartbeat 무수신(joy_timeout) · E-stop 버튼 · 물리 스위치 · OpenCR 링크 끊김
//   풀림: 리셋 버튼 + 조이스틱 정상 + 물리 스위치 해제 + 링크 정상 + E-stop 버튼 안 누름 (모두 동시에)
//   시작은 잠김 상태 (전원 켠 뒤 리셋 버튼을 눌러야 움직임)
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

}  // namespace opencr_hardware
