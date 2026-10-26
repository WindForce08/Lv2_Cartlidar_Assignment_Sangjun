// 태그 거리 맞추기 (ROS 의존 없음 → 단위 시험)
//   작은 태그는 네 꼭짓점만으로 구한 거리(PnP)가 흔들린다: D435 1 m 에서 48 mm 장소 태그(약 29 px)가 ±5 % (R5 렌더 2026-10-25,
//   위치 오차 최대 70 mm). 정렬 깊이가 있으면 태그 사각형 안 깊이 중앙값으로 광학축 거리를 맞추고, 같은 비율로 x · y 도 늘인다
//   (태그 가운데로 가는 시선 방향은 PnP 값을 그대로 둠)
#pragma once

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace perception
{

// corners: 화소 꼭짓점 4개, depth: CV_16U 또는 CV_32F(scale 곱해 m). 쓸 깊이 화소가 min_pixels 보다 적으면 false(그대로 둠)
inline bool scale_tag_to_depth(cv::Vec3d & tvec, const std::vector<cv::Point2f> & corners, const cv::Mat & depth,
  double scale, double min_m, double max_m, int min_pixels = 10)
{
  if (corners.size() != 4 || depth.empty() || tvec[2] <= 0.0) {return false;}
  const bool f32 = depth.type() == CV_32F;
  if (!f32 && depth.type() != CV_16U) {return false;}
  std::vector<cv::Point> poly;
  for (const auto & c : corners) {poly.emplace_back(static_cast<int>(std::lround(c.x)), static_cast<int>(std::lround(c.y)));}
  const cv::Rect box = cv::boundingRect(poly) & cv::Rect(0, 0, depth.cols, depth.rows);
  if (box.empty()) {return false;}
  cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8U);
  cv::fillConvexPoly(mask, poly, cv::Scalar(255));
  std::vector<double> v;
  for (int y = box.y; y < box.y + box.height; ++y) {
    for (int x = box.x; x < box.x + box.width; ++x) {
      if (!mask.at<uint8_t>(y, x)) {continue;}
      const double d = f32 ? static_cast<double>(depth.at<float>(y, x)) * scale : depth.at<uint16_t>(y, x) * scale;
      if (std::isfinite(d) && d >= min_m && d <= max_m) {v.push_back(d);}
    }
  }
  if (static_cast<int>(v.size()) < min_pixels) {return false;}
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  tvec *= v[v.size() / 2] / tvec[2];
  return true;
}

}  // namespace perception
