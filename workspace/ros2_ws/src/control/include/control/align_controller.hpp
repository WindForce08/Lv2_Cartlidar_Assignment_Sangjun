// /drive/align 제어 법칙 (drive_node) — ROS 의존 없음
//   정밀(AlignController): Arducam 가로 오차 → 회전, TFmini 박스 앞면 거리(BoxRangeFilter, 차체 1층 렌즈 기준) → 전후 P 제어
//   먼 거리(CoarseAlign): D435 object_pose(base_link) → 물체 쪽 회전 + 전진/후진, 범퍼 기준 거리
//   scan_blocked(): 직접 P 제어라 nav2 회피가 없음 → /scan 으로 앞 · 뒤 장애물 확인
//   허용오차 안에서 N 주기 연속이면 완료.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace control
{

struct AlignGains
{
  double kp_yaw = 1.2;        // rad/s per 정규화 오차(-1~1)
  double kp_lin = 1.0;        // (m/s) / m
  double max_ang = 0.6;       // rad/s
  double max_lin = 0.08;      // m/s
  double lateral_tol = 0.04;  // 정규화 오차
  int settle_cycles = 10;     // 20 Hz × 10 = 0.5 s
};

struct AlignCmd
{
  double v = 0.0;
  double w = 0.0;
  bool in_tolerance = false;
  bool done = false;
};

class AlignController
{
public:
  explicit AlignController(AlignGains g = {})
  : g_(g) {}

  void reset() {settled_ = 0;}

  // lateral: 물체가 화면 오른쪽이면 + → 오른쪽(시계 방향, w<0)으로 돈다
  // distance: 현재 거리, target: 목표 거리, dist_tol: 허용오차 (m)
  AlignCmd step(double lateral, double distance, double target, double dist_tol)
  {
    AlignCmd c;
    const double e_d = distance - target;
    const bool lat_ok = std::fabs(lateral) <= g_.lateral_tol;
    const bool dist_ok = std::fabs(e_d) <= dist_tol;
    c.in_tolerance = lat_ok && dist_ok;
    settled_ = c.in_tolerance ? settled_ + 1 : 0;
    if (settled_ >= g_.settle_cycles) {c.done = true; return c;}   // v = w = 0
    c.w = lat_ok ? 0.0 : std::clamp(-g_.kp_yaw * lateral, -g_.max_ang, g_.max_ang);
    // 옆 오차가 크면 먼저 돌고, 어느 정도 맞으면 전후 이동
    const double lin_scale = std::fabs(lateral) > 3 * g_.lateral_tol ? 0.3 : 1.0;
    c.v = dist_ok ? 0.0 : std::clamp(g_.kp_lin * e_d, -g_.max_lin, g_.max_lin) * lin_scale;
    return c;
  }

  int settled() const {return settled_;}

private:
  AlignGains g_;
  int settled_ = 0;
};

struct CoarseGains
{
  double kp_yaw = 1.5;          // rad/s per rad
  double kp_lin = 0.8;
  double max_ang = 0.8;
  double max_lin = 0.15;        // align.coarse_max_speed
  double yaw_tol = 0.05;        // rad
  double turn_first = 0.35;     // 이보다 각도 오차가 크면 제자리 회전 먼저
  int settle_cycles = 6;
};

// object (x, y) = base_link 기준 물체 위치. front_offset = base_link → 범퍼 앞면 x
class CoarseAlign
{
public:
  explicit CoarseAlign(CoarseGains g = {})
  : g_(g) {}
  void reset() {settled_ = 0;}

  AlignCmd step(double ox, double oy, double front_offset, double target, double tol)
  {
    AlignCmd c;
    const double ang = std::atan2(oy, ox);
    const double dist = ox - front_offset;           // 범퍼 기준 거리
    const double e = dist - target;
    const bool ang_ok = std::fabs(ang) <= g_.yaw_tol;
    const bool dist_ok = std::fabs(e) <= tol;
    c.in_tolerance = ang_ok && dist_ok;
    settled_ = c.in_tolerance ? settled_ + 1 : 0;
    if (settled_ >= g_.settle_cycles) {c.done = true; return c;}
    c.w = ang_ok ? 0.0 : std::clamp(g_.kp_yaw * ang, -g_.max_ang, g_.max_ang);
    if (std::fabs(ang) < g_.turn_first && !dist_ok) {
      c.v = std::clamp(g_.kp_lin * e, -g_.max_lin, g_.max_lin) * std::cos(ang);   // 이미 더 가까우면 후진
    }
    return c;
  }

private:
  CoarseGains g_;
  int settled_ = 0;
};

// 박스 앞면 거리 (TFmini /tfmini/range) 다듬기
//   최근 window 개 측정 중 max_age_s 안의 것만 본다. 유효값이 min_samples 개 이상이고 그 퍼짐(최대 − 최소)이
//   spread_m 이하일 때만 중앙값을 낸다. 빔이 박스 모서리에 걸려 박스 · 뒤쪽이 번갈아 읽히거나, 무효(+inf · 최소 거리 안)가
//   섞이면 nullopt → 정렬은 멈추고 lost_timeout 뒤 실패. 1 cm 계단 · 튀는 값 하나는 중앙값이 거른다
struct BoxRangeConfig
{
  int window = 5;
  int min_samples = 3;
  double spread_m = 0.03;
  double max_age_s = 0.3;   // 300 ms 규칙
};

class BoxRangeFilter
{
public:
  explicit BoxRangeFilter(BoxRangeConfig c = {})
  : c_(c) {}

  void reset() {q_.clear();}

  // t = 측정 stamp [s], r = 유효 거리 (nullopt = 무효)
  void push(double t, std::optional<double> r)
  {
    q_.push_back({t, r && std::isfinite(*r) ? *r : NAN});
    while (static_cast<int>(q_.size()) > c_.window) {q_.pop_front();}
  }

  std::optional<double> value(double now) const
  {
    std::vector<double> v;
    for (const auto & e : q_) {
      if (now - e.first <= c_.max_age_s && std::isfinite(e.second)) {v.push_back(e.second);}
    }
    if (static_cast<int>(v.size()) < c_.min_samples) {return std::nullopt;}
    std::sort(v.begin(), v.end());
    if (v.back() - v.front() > c_.spread_m) {return std::nullopt;}
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
  }

private:
  BoxRangeConfig c_;
  std::deque<std::pair<double, double>> q_;
};

// 박스 접근 중 obstacle_stop 정지 거리: 목표 − 여유, TFmini 최소 측정 거리 이상
inline double box_approach_stop(double target, double margin, double min_m)
{
  return std::max(min_m, target - margin);
}

// /scan (base_link 기준 각도) 에서 진행 방향 ±half_angle 안에 limit 보다 가까운 점이 있으면 true
//   전진: limit = 물체 거리 − 0.1 (물체보다 0.1 m 이상 가까운 점) · 후진: 후방 ±20° 에 0.3 m
inline bool scan_blocked(const std::vector<float> & ranges, double angle_min, double angle_inc,
  double center, double half_angle, double limit)
{
  for (size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r) || r <= 0.01f) {continue;}
    double a = angle_min + angle_inc * static_cast<double>(i) - center;
    a = std::atan2(std::sin(a), std::cos(a));
    if (std::fabs(a) <= half_angle && r < limit) {return true;}
  }
  return false;
}

// 뒤 D435 (/perception/rear_obstacle) 로 후진을 막을지 (#84). 스캔면 (82 mm) 아래 퍽 · 박스는 /scan 에 안 보임
//   blocked = 최신 값, 없음 · 오래됨 = nullopt → 막음 (rear_obstacle 도 입력이 끊기면 true 를 냄)
inline bool rear_camera_blocks(double v, bool enabled, std::optional<bool> blocked)
{
  return enabled && v < -1e-3 && (!blocked || *blocked);
}

}  // namespace control
