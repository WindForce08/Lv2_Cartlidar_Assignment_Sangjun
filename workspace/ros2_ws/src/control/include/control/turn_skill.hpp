// 상자 퍽 45° 집기용 제자리 회전 skill (drive_node /drive/execute_skill "turn_p17" · "turn_m17" · "turn_0", #77) — ROS 의존 없음
//   계획기 (mission_planner poses.turn_deg) 는 로봇 중심 (base_link + center_x) 둘레로 deg 돌아
//   J1 을 옆으로 (j1_x − center_x)·sin(deg) 만큼 옮긴다고 본다 (J1→퍽 = 박스 축 = 퍽 면).
//   스키드 차체는 회전 중심을 고를 수 없고 실제 제자리 회전 중심 (ICR, base_link x = icr_x) 둘레로 돈다.
//   → 같은 J1 옆 이동이 되도록 각을 바꾼다 (icr_x = center_x 면 이름 각 그대로).
//   yaw 되먹임 = /odometry/filtered (EKF). 기준 yaw₀ = 직전 FINE 정렬 성공 때 yaw.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "control/align_controller.hpp"

namespace control
{

// "turn_p17" → +17, "turn_m17" → −17, "turn_0" → 0, "turn_p17.06" 도 받음 (p = 왼쪽 +). 그 밖은 nullopt
inline std::optional<double> parse_turn_deg(const std::string & name)
{
  const std::string pre = "turn_";
  if (name.rfind(pre, 0) != 0) {return std::nullopt;}
  std::string s = name.substr(pre.size());
  if (s == "0") {return 0.0;}
  if (s.size() < 2 || (s[0] != 'p' && s[0] != 'm')) {return std::nullopt;}
  const double sign = s[0] == 'p' ? 1.0 : -1.0;
  s = s.substr(1);
  if (s.find_first_not_of("0123456789.") != std::string::npos || s.front() == '.') {return std::nullopt;}
  try {
    size_t n = 0;
    const double d = std::stod(s, &n);
    if (n != s.size() || !(d > 0.0) || d >= 90.0) {return std::nullopt;}
    return sign * d;
  } catch (...) {
    return std::nullopt;
  }
}

struct TurnGeometry
{
  double j1_x = 0.096;        // base_link → J1 축 x (mission_planner poses.J1_X)
  double center_x = -0.04035; // 계획기가 가정한 회전 중심 = 외곽 가운데 (poses.CENTER_X)
  double icr_x = -0.04035;    // 실제 제자리 회전 중심 x
};

// 계획기 각 deg [°] → 실제로 돌 각 [rad]: J1 옆 이동이 같게
inline double turn_angle_rad(double deg, const TurnGeometry & g)
{
  const double rad = deg * M_PI / 180.0;
  const double r = g.j1_x - g.icr_x;
  if (r <= 1e-6) {return rad;}   // J1 이 회전 중심 뒤 — 보정 불가, 이름 각 그대로
  const double lat = (g.j1_x - g.center_x) * std::sin(rad);
  return std::asin(std::clamp(lat / r, -1.0, 1.0));
}

inline double wrap_pi(double a) {return std::atan2(std::sin(a), std::cos(a));}

struct TurnGains
{
  double kp = 2.0;                 // (rad/s) / rad
  double max_ang = 0.6;            // rad/s
  double min_ang = 0.15;           // rad/s, 허용 밖이면 적어도 이 속도 (정지 마찰)
  double tol = 1.0 * M_PI / 180.0; // rad
  int settle_cycles = 6;           // 20 Hz × 6 = 0.3 s
};

// 남은 각 err [rad] → 회전 속도. 허용 안이면 w 0, settle_cycles 연속이면 done
class TurnController
{
public:
  explicit TurnController(TurnGains g = {})
  : g_(g) {}

  void reset() {settled_ = 0;}

  AlignCmd step(double err)
  {
    AlignCmd c;
    err = wrap_pi(err);
    if (std::fabs(err) <= g_.tol) {
      c.in_tolerance = true;
      c.done = ++settled_ >= g_.settle_cycles;
      return c;
    }
    settled_ = 0;
    double w = std::clamp(g_.kp * err, -g_.max_ang, g_.max_ang);
    if (std::fabs(w) < g_.min_ang) {w = std::copysign(g_.min_ang, err);}
    c.w = w;
    return c;
  }

private:
  TurnGains g_;
  int settled_ = 0;
};

}  // namespace control
