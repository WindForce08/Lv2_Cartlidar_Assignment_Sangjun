// SagComp — 팔 처짐 보정 (manipulator_node 팔레트 칸 궤적, sag.*)
//
// 왜: 팔 구동은 펌웨어 위치 P 800 · I 0 (강성 약 18.88 N·m/rad) 이라 퍽을 쥔 팔이 6 ~ 13 mm 처진다.
//     팔레트 칸 틈은 1 ~ 2 mm 라 처진 채로는 칸 테두리에 걸린다 (Isaac 보정 없이 4/24).
// 어떻게 (Isaac urdf_and_usd/policy/manipulator/pallet_place_check.py --ki 와 같은 식, 24/24):
//     시간표의 계획 관절값 plan(t) 에 bias 를 더해 명령하고, 매 주기 bias += ki · dt · (plan − 측정) (± bias_max).
//     시작 자세에서 settle_time 동안 먼저 bias 를 맞춘 뒤 출발한다 (쥔 뒤 보정이 집게를 돌리지 않게).
// JointTimeline: 경유점 · 머묾 · 그리퍼 값을 시간축 하나로. 관절 · 그리퍼 모두 점 사이 직선 보간.
//     경유점 시간은 manipulator_node sendWaypoints 와 같음 (첫 점 max(first_min, Δq/speed), 그다음 max(0.02, Δq/speed)).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace control
{

using Arm5 = std::array<double, 5>;   // joint1~5

struct TimelinePoint
{
  double t{0.0};      // 도착 시각 (s, 시간표 시작 기준)
  Arm5 q{};           // 관절 1~5
  double grip{0.0};   // 6 번째 값 (그리퍼)
};

class JointTimeline
{
public:
  JointTimeline(const Arm5 & start, double grip) {pts_.push_back({0.0, start, grip});}

  // 경유점들을 speed (rad/s) 로. 그리퍼는 첫 점까지 grip 으로 바뀌고 그다음 그대로
  void moveThrough(const std::vector<Arm5> & qs, double speed, double grip, double first_min)
  {
    bool first = true;
    for (const auto & q : qs) {
      const auto & prev = pts_.back();
      double d = 0.0;
      for (size_t j = 0; j < 5; ++j) {d = std::max(d, std::abs(q[j] - prev.q[j]));}
      const double dt = std::max(first ? first_min : 0.02, d / speed);
      pts_.push_back({prev.t + dt, q, grip});
      first = false;
    }
  }

  // 같은 자리에서 duration 동안 (그리퍼는 그동안 grip 으로 바뀜)
  void hold(double duration, double grip)
  {
    const auto & prev = pts_.back();
    pts_.push_back({prev.t + std::max(0.0, duration), prev.q, grip});
  }

  double duration() const {return pts_.back().t;}
  const std::vector<TimelinePoint> & points() const {return pts_;}

  TimelinePoint at(double t) const
  {
    if (t <= 0.0) {return {t, pts_.front().q, pts_.front().grip};}
    for (size_t i = 1; i < pts_.size(); ++i) {
      const auto & a = pts_[i - 1];
      const auto & b = pts_[i];
      if (t <= b.t) {
        const double f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1.0;
        TimelinePoint p{t, {}, a.grip + (b.grip - a.grip) * f};
        for (size_t j = 0; j < 5; ++j) {p.q[j] = a.q[j] + (b.q[j] - a.q[j]) * f;}
        return p;
      }
    }
    return {t, pts_.back().q, pts_.back().grip};
  }

private:
  std::vector<TimelinePoint> pts_;
};

class SagIntegrator
{
public:
  SagIntegrator(double ki, double bias_max): ki_(ki), bias_max_(bias_max) {}

  void update(const Arm5 & plan, const Arm5 & measured, double dt)
  {
    for (size_t j = 0; j < 5; ++j) {
      bias_[j] = std::clamp(bias_[j] + ki_ * dt * (plan[j] - measured[j]), -bias_max_, bias_max_);
    }
  }

  Arm5 command(const Arm5 & plan) const
  {
    Arm5 c{};
    for (size_t j = 0; j < 5; ++j) {c[j] = plan[j] + bias_[j];}
    return c;
  }

  const Arm5 & bias() const {return bias_;}

private:
  double ki_, bias_max_;
  Arm5 bias_{};
};

}  // namespace control
