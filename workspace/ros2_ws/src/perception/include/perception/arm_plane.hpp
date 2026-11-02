// 팔 카메라 평면 교차 (ROS 의존 없음 → 단위 시험)
//   Arducam 은 깊이가 없어 박스 중심은 시선(방향) 하나만 준다. 퍽은 높이를 아는 받침(바닥 · 출발 상자 바닥 등) 위에 서 있거나
//   누워 있으므로, 퍽 중심 높이 = 받침 높이 + 반높이(서 있음 0.03 · 누움 0.015)인 수평면과 시선이 만나는 점을 퍽 중심으로 본다.
//   받침 · 반높이 조합은 D435 추적 목표 높이에 가장 가까운 것을 고른다(D435 높이는 표면 가운데라 중심보다 약 5 mm 낮게 나옴,
//   A1 모의 2026-11-02 — 그대로 쓰면 비스듬한 시선에서 수평 오차가 커져 목록 값으로 맞춘다).
//   D435 추적 위치의 예비로 쓴다(D435 가 못 볼 때 · 놓쳤을 때). 둘을 비교해 검증에도 쓴다
//   (A1 렌더 모의 재생 2026-11-02, 카메라 거리 0.39~0.55 m, 잘린 박스 뺌: 퍽 중심 수평 오차 중앙 3.6 · p95 8.0 mm,
//    D435 를 끈 뒤 팔 카메라만 3.7 · 13.1 mm. 같은 모의에서 D435 추적 위치는 2.5 · 3.0 mm — 모의 깊이는 잡음이 없다)
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

// 받침 높이 levels × 반높이 halves 중 measured_z 에 가장 가까운 합. 가장 가까운 것도 max_gap 넘게 떨어지면 없음
inline std::optional<double> pick_plane_height(double measured_z, const std::vector<double> & levels,
  const std::vector<double> & halves, double max_gap)
{
  if (!std::isfinite(measured_z)) {return std::nullopt;}
  double best = std::numeric_limits<double>::quiet_NaN();
  double gap = std::numeric_limits<double>::infinity();
  for (double l : levels) {
    for (double h : halves) {
      if (std::abs(l + h - measured_z) < gap) {gap = std::abs(l + h - measured_z); best = l + h;}
    }
  }
  if (!(gap <= max_gap)) {return std::nullopt;}
  return best;
}

// 화소 (u, v) 시선 ∩ 수평면 z = plane_z. R · t: 카메라 광학 좌표 → 평면 기준 좌표 (z 위). K: 왜곡 보정한 영상 내부값.
// 시선이 평면과 거의 나란하거나(|d_z| < 1e-6) 교점이 카메라 뒤면 없음
inline std::optional<cv::Vec3d> ray_plane(const cv::Matx33d & K, double u, double v,
  const cv::Matx33d & R, const cv::Vec3d & t, double plane_z)
{
  const cv::Vec3d d = R * cv::Vec3d((u - K(0, 2)) / K(0, 0), (v - K(1, 2)) / K(1, 1), 1.0);
  if (std::abs(d[2]) < 1e-6) {return std::nullopt;}
  const double s = (plane_z - t[2]) / d[2];
  if (!(s > 0.0)) {return std::nullopt;}
  return t + s * d;
}

}  // namespace perception
