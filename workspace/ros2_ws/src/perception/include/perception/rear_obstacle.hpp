// 뒤 D435 깊이로 차체 뒤 통로의 가장 가까운 장애물 거리를 잰다 (ROS 의존 없음 → 단위 시험)
//
//   깊이 화소 → 광학 좌표(deproject) → 바닥 기준 프레임(base_footprint, z = 바닥 위 높이) → 통로 안 점만 남긴다:
//     x < x_edge(차체 뒤 끝), |y| ≤ y_half, z_min ≤ z ≤ z_max, 거리 d = x_edge − x ≤ range_m
//   잡음 화소 하나에 정지하지 않게, 가까운 순서로 min_points 번째 점의 d 를 거리로 쓴다.
//   통로 안 점이 min_points 보다 적으면 "없음"(통로가 비었음).
//   라이다(바닥 위 한 높이 평면)가 못 보는 낮은 물체(누운 퍽) · 높은 물체(선반 끝)를 보려는 것이다.
//   카메라가 아래로 35° 기울어 바닥 높이에서는 뒤 끝 가까이(컬러 화각 기준 계산 약 0.31 m 안)가 화면 밖이다 → 그 안은 라이다 몫.
//   정지 · 해제 판단은 obstacle_stop.hpp(ObstacleStop)를 그대로 쓴다. 카메라는 "없음" = 비었음이므로 near_m 은 0 으로 둔다.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/geometry.hpp"

namespace perception
{

struct RearConfig
{
  double x_edge = -0.2285;   // 차체 뒤 끝 x [m] (base_footprint). lidar.yaml self_box x_min 과 같게
  double y_half = 0.2176;    // 통로 반폭 [m] = 차체 반폭 0.1676 + 0.05
  double z_min = 0.02;       // 바닥 위 높이 하한 [m] (바닥 · 깊이 잡음을 거름)
  double z_max = 0.45;       // 상한 [m] (차체 위 끝 근처)
  double range_m = 1.0;      // 뒤 끝에서 이 거리까지만 본다 [m] (해제 거리보다 커야 함)
  double depth_min_m = 0.10, depth_max_m = 3.0;   // 깊이 유효 범위 [m]
  int stride = 2;            // 화소 건너뛰기 (2 = 가로 · 세로 한 칸씩 건너 1/4)
  int min_points = 20;       // 장애물로 볼 최소 점 수 (stride 로 고른 점 기준)
};

struct RearResult
{
  std::optional<double> distance;   // 뒤 끝에서 장애물까지 [m]. 없으면 nullopt
  int points = 0;                   // 통로 안 점 수
};

// depth: CV_16U(× scale → m) 또는 CV_32F(m). K: 깊이 영상(컬러에 맞춘 깊이면 컬러) 내부값.
// T: 바닥 기준 프레임 ← 깊이 광학 프레임 (3×4, [R | t])
inline RearResult nearest_rear(const cv::Mat & depth, double scale, const PinholeK & K, const cv::Matx34d & T,
  const RearConfig & c)
{
  RearResult out;
  const bool f32 = depth.type() == CV_32F;
  if (depth.empty() || (!f32 && depth.type() != CV_16U) || K.fx <= 0 || K.fy <= 0) {return out;}
  const int step = std::max(1, c.stride);
  std::vector<double> d;
  for (int v = 0; v < depth.rows; v += step) {
    for (int u = 0; u < depth.cols; u += step) {
      const double z = f32 ? static_cast<double>(depth.at<float>(v, u)) : depth.at<uint16_t>(v, u) * scale;
      if (!std::isfinite(z) || z < c.depth_min_m || z > c.depth_max_m) {continue;}
      const cv::Point3d p = deproject(K, u, v, z);
      const cv::Vec4d q(p.x, p.y, p.z, 1.0);
      const double x = T.row(0).dot(q.t()), y = T.row(1).dot(q.t()), h = T.row(2).dot(q.t());
      const double dist = c.x_edge - x;
      if (dist <= 0.0 || dist > c.range_m || std::abs(y) > c.y_half || h < c.z_min || h > c.z_max) {continue;}
      d.push_back(dist);
    }
  }
  out.points = static_cast<int>(d.size());
  const int k = std::max(1, c.min_points);
  if (out.points < k) {return out;}
  std::nth_element(d.begin(), d.begin() + (k - 1), d.end());
  out.distance = d[k - 1];
  return out;
}

}  // namespace perception
