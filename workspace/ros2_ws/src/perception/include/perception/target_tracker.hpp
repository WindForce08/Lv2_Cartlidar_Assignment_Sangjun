// 물체 하나만 따라가기 (perception_node, ROS 의존 없음 → 단위 시험)
//   1) exclude_box(base_link: 차체 footprint + 그리퍼 작업 영역) 안 검출은 버림 — 잡은 물체를 다시 찾지 않게
//   2) confidence ≥ min_confidence 중 가장 가까운 것(base_link 수평 거리)을 고름
//   3) 고른 뒤에는 직전 위치(odom) gate(0.3 m) 안의 검출만 같은 물체로 봄
//   4) lost_timeout(1 s) 이상 안 보이면 다시 고름, reset()(set_mode OFF)에서 초기화
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace perception
{

struct Vec3
{
  double x = 0, y = 0, z = 0;
};

struct Candidate
{
  Vec3 base;          // base_link 기준
  Vec3 odom;          // odom 기준 (TF 없으면 base 와 같게)
  double confidence = 0;
  int camera = 0;     // 0 전방 · 1 후방
};

struct TrackerConfig
{
  Vec3 exclude_min{-0.30, -0.25, -0.10};   // base_link 기준 상자 ★ 차체 + 그리퍼 작업 영역
  Vec3 exclude_max{0.35, 0.25, 0.60};
  double min_confidence = 0.5;
  double gate = 0.3;
  double lost_timeout = 1.0;
};

class TargetTracker
{
public:
  explicit TargetTracker(TrackerConfig c = {})
  : c_(c) {}

  void reset() {locked_ = false;}
  bool locked() const {return locked_;}

  // t: 촬영 시각 (s). 고른 후보 반환
  std::optional<Candidate> update(const std::vector<Candidate> & cands, double t)
  {
    if (locked_ && t - last_seen_ > c_.lost_timeout) {locked_ = false;}
    const Candidate * best = nullptr;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & k : cands) {
      if (k.confidence < c_.min_confidence || excluded(k.base)) {continue;}
      double score;
      if (locked_) {
        score = dist(k.odom, last_odom_);
        if (score > c_.gate) {continue;}
      } else {
        score = std::hypot(k.base.x, k.base.y);
      }
      if (score < best_score) {best_score = score; best = &k;}
    }
    if (!best) {return std::nullopt;}
    locked_ = true;
    last_odom_ = best->odom;
    last_seen_ = t;
    return *best;
  }

  bool excluded(const Vec3 & p) const
  {
    return p.x >= c_.exclude_min.x && p.x <= c_.exclude_max.x && p.y >= c_.exclude_min.y && p.y <= c_.exclude_max.y &&
           p.z >= c_.exclude_min.z && p.z <= c_.exclude_max.z;
  }

private:
  static double dist(const Vec3 & a, const Vec3 & b) {return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));}
  TrackerConfig c_;
  bool locked_ = false;
  Vec3 last_odom_;
  double last_seen_ = -1e9;
};

}  // namespace perception
