// /arm_range 계산 (ROS 의존 없음 → 단위 시험)
//   팔 끝(range_frame, 기본 tool_link, x = 접근 방향)에서 ARM_ALIGN 으로 정렬 중인 퍽 표면까지의 거리.
//   TFmini 는 박스 앞면 거리(/tfmini/range) 전용이라 쓰지 않고, D435 · Arducam 측위값으로 만든다.
//   출처 우선순위
//     1) Arducam 태그 퍽 중심 (READ_TAG 가 켜져 있을 때, 촬영 시각 팔 자세로 변환): 팔 축에서 옆으로 tag_gate 안인 것 중 가장 가까운 것
//     2) D435 추적 목표를 odom 에 붙잡아 둔 점 (팔 정렬 목표를 고른 그 점): 마지막으로 본 지 hold_s 안, 팔 축에서 seed_gate 안
//        퍽은 정지해 있으므로 D435 시야 아래로 빠진 정밀 거리에서도 odom 으로 현재 팔 위치에서 다시 계산할 수 있다
//   거리 = 퍽 중심의 x − surface_offset(퍽 반지름) → 기존 TFmini 기준과 같은 "표면까지" 거리
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace perception
{

using P3 = std::array<double, 3>;   // range_frame 기준 [x 앞, y, z]

struct ArmRangeConfig
{
  double tag_gate = 0.03;        // [m] 태그 퍽 중심이 팔 축(x)에서 옆으로 이 안이어야 정렬 중인 퍽으로 본다 (이웃 퍽 ±40 mm)
  double seed_gate = 0.05;       // [m] D435 보유 점의 팔 축 옆 거리 상한 (D435 깊이 · odom 오차 포함)
  double hold_s = 3.0;           // [s] D435 가 마지막으로 본 뒤 이 시간까지 보유 점을 쓴다
  double surface_offset = 0.015; // [m] 퍽 중심 → 팔 쪽 표면 (퍽 반지름 · 짧은 변 / 2)
};

struct ArmRangeEstimate
{
  enum class Source { TAG, D435_HOLD };
  double range = 0.0;            // [m] 표면까지 (0 미만이면 0)
  double lateral = 0.0;          // [m] 팔 축 옆 거리
  Source source = Source::TAG;
};

inline double lateral_of(const P3 & p) {return std::hypot(p[1], p[2]);}

// tags: 태그로 찾은 퍽 중심들, seed: D435 보유 점 (없으면 nullopt), seed_age: 그 점을 마지막으로 본 뒤 지난 시간 [s]
inline std::optional<ArmRangeEstimate> arm_range(
  const std::vector<P3> & tags, const std::optional<P3> & seed, double seed_age, const ArmRangeConfig & c)
{
  const P3 * best = nullptr;
  for (const auto & p : tags) {
    if (!(p[0] > 0.0) || lateral_of(p) > c.tag_gate) {continue;}
    if (!best || lateral_of(p) < lateral_of(*best)) {best = &p;}
  }
  ArmRangeEstimate e;
  if (best) {
    e.source = ArmRangeEstimate::Source::TAG;
  } else if (seed && std::isfinite(seed_age) && seed_age >= 0.0 && seed_age <= c.hold_s && (*seed)[0] > 0.0 &&
    lateral_of(*seed) <= c.seed_gate)
  {
    best = &*seed;
    e.source = ArmRangeEstimate::Source::D435_HOLD;
  } else {
    return std::nullopt;
  }
  e.range = std::max(0.0, (*best)[0] - c.surface_offset);
  e.lateral = lateral_of(*best);
  return e;
}

}  // namespace perception
