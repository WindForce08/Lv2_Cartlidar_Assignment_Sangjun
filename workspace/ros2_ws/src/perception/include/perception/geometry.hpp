// perception_node 기하 계산 (ROS · RealSense 의존 없음 → 단위 시험)
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct PinholeK
{
  double fx = 0, fy = 0, ppx = 0, ppy = 0;
};

// 화소 (u, v) + 깊이 z(m) → 광학 좌표 (x 오른쪽 · y 아래 · z 앞). D435 정렬 컬러는 왜곡 0.
inline cv::Point3d deproject(const PinholeK & k, double u, double v, double z)
{
  return {(u - k.ppx) / k.fx * z, (v - k.ppy) / k.fy * z, z};
}

// 광학 좌표 → 링크 좌표 (x 앞 · y 왼 · z 위) — camera_front_link 등 URDF 링크 규약
inline cv::Point3d optical_to_link(const cv::Point3d & p) {return {p.z, -p.x, -p.y};}

// 박스 중심 주변 (2·half+1)² 창의 깊이 중앙값 (m). 유효 값이 없으면 NaN
inline double median_depth(const cv::Mat & depth16, cv::Point c, int half, double scale, double min_m, double max_m)
{
  std::vector<double> v;
  for (int y = c.y - half; y <= c.y + half; ++y) {
    for (int x = c.x - half; x <= c.x + half; ++x) {
      if (x < 0 || y < 0 || x >= depth16.cols || y >= depth16.rows) {continue;}
      const double d = depth16.at<uint16_t>(y, x) * scale;
      if (d >= min_m && d <= max_m) {v.push_back(d);}
    }
  }
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

// 뎁스로 본 실제 크기 확인 (Xmen detector 의 크기 검증과 같은 생각): 퍽 30 × 30 × 60 mm 는
//   어느 방향에서 봐도 짧은 변 ≥ 약 2 cm, 긴 변 ≤ 약 7 cm. 벽 · 바닥 무늬 · 먼 큰 물체를 거른다.
inline bool size_plausible(const cv::Rect2f & box, double z, const PinholeK & k, double min_m, double max_m)
{
  if (!(z > 0) || k.fx <= 0 || k.fy <= 0) {return true;}   // 확인할 수 없으면 통과 (색 · 모양만)
  const double w = box.width * z / k.fx, h = box.height * z / k.fy;
  return std::min(w, h) >= min_m && std::max(w, h) <= max_m;
}

// ARM_ALIGN 오차: x · y = 화면 중심 대비 (-1~1, 오른쪽 · 아래 +), z = 박스 폭 / 화면 폭
inline cv::Vec3d image_offset(const cv::Rect2f & box, cv::Size size)
{
  const double cx = box.x + box.width / 2.0, cy = box.y + box.height / 2.0;
  return {(cx - size.width / 2.0) / (size.width / 2.0), (cy - size.height / 2.0) / (size.height / 2.0),
    box.width / static_cast<double>(size.width)};
}

}  // namespace perception
