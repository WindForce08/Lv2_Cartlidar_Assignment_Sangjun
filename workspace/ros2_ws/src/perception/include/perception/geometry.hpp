// perception_node 기하 계산 (ROS · RealSense 의존 없음 → 단위 시험)
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct PinholeK
{
  double fx = 0, fy = 0, ppx = 0, ppy = 0;
};

// 화소 (u, v) + 깊이 z(m) → 광학 좌표 (x 오른쪽 · y 아래 · z 앞). D435 정렬 컬러는 왜곡 0.
// 결과는 영상 header.frame_id(컬러 광학 프레임) 기준이므로 TF 로 바로 base_link 로 바꾼다.
inline cv::Point3d deproject(const PinholeK & k, double u, double v, double z)
{
  return {(u - k.ppx) / k.fx * z, (v - k.ppy) / k.fy * z, z};
}

// 깊이 영상 인코딩 → m 환산 배율. 16UC1(realsense2_camera)은 mm, 32FC1(Isaac Sim)은 m. 모르는 인코딩은 0
inline double depth_scale_for_encoding(const std::string & enc)
{
  if (enc == "16UC1" || enc == "mono16") {return 0.001;}
  if (enc == "32FC1") {return 1.0;}
  return 0.0;
}

// 박스 중심 주변 (2·half+1)² 창의 깊이 중앙값 (m). depth 는 CV_16U 또는 CV_32F. 유효 값이 없으면 NaN
//   0 · NaN · inf(깊이 없음)와 [min_m, max_m] 밖 값은 뺀다
inline double median_depth(const cv::Mat & depth, cv::Point c, int half, double scale, double min_m, double max_m)
{
  std::vector<double> v;
  const bool f32 = depth.type() == CV_32F;
  if (!f32 && depth.type() != CV_16U) {return std::numeric_limits<double>::quiet_NaN();}
  for (int y = c.y - half; y <= c.y + half; ++y) {
    for (int x = c.x - half; x <= c.x + half; ++x) {
      if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) {continue;}
      const double d = f32 ? static_cast<double>(depth.at<float>(y, x)) * scale : depth.at<uint16_t>(y, x) * scale;
      if (std::isfinite(d) && d >= min_m && d <= max_m) {v.push_back(d);}
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
