// 퍽 태그 자세 두 해 고르기 (ROS 의존 없음 → 단위 시험)
//   작은 태그는 IPPE 두 해의 재투영 오차 차가 매우 작다(팔 카메라 0.14 m 의 16 mm 태그 약 32 px 에서 0.002 px 까지).
//   그래서 오차가 작은 해가 법선이 크게 기운 틀린 해일 수 있다(R6 렌더 2026-10-25: 46° 기운 해를 골라 퍽 중심이 23 mm 밀림).
//   퍽은 서 있거나 누워 있어 태그가 붙은 끝면의 법선이 base_link 기준 수직 또는 수평이다. 두 해의 오차 차가 max_gap_px 이하면
//   법선이 수직 · 수평에 더 가까운 해를 쓴다. 오차 차가 크면 오차가 작은 해를 그대로 쓴다
#pragma once

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>

#include "perception/tag_detector.hpp"

namespace perception
{

// 법선(base_link)이 수직 또는 수평면에서 벗어난 각 [deg]: min(수직과 이루는 각, 수평면과 이루는 각) = 0 ~ 45
inline double axis_deviation_deg(const cv::Vec3d & n)
{
  const double nz = std::min(1.0, std::fabs(n[2]) / std::max(cv::norm(n), 1e-12));
  const double to_vertical = std::acos(nz) * 180.0 / M_PI;
  return std::min(to_vertical, 90.0 - to_vertical);
}

// R_base_cam: 카메라 광학 좌표 → base_link 회전. max_gap_px <= 0 이면 끔
inline TagDetection prefer_upright(const TagDetection & t, const cv::Matx33d & R_base_cam, double max_gap_px)
{
  if (max_gap_px <= 0.0 || !t.pose_valid || !t.alt_valid || t.err_alt - t.err > max_gap_px) {return t;}
  auto deviation = [&](const cv::Vec3d & rvec) {
      cv::Matx33d R;
      cv::Rodrigues(rvec, R);
      return axis_deviation_deg(R_base_cam * cv::Vec3d(R(0, 2), R(1, 2), R(2, 2)));
    };
  if (deviation(t.rvec_alt) >= deviation(t.rvec)) {return t;}
  TagDetection s = t;
  std::swap(s.rvec, s.rvec_alt);
  std::swap(s.tvec, s.tvec_alt);
  std::swap(s.err, s.err_alt);
  return s;
}

}  // namespace perception
