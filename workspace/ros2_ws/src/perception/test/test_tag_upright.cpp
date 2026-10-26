// 퍽 태그 자세 두 해 고르기 시험 (tag_upright.hpp)
#include <gtest/gtest.h>

#include <cmath>

#include "perception/tag_upright.hpp"

using namespace perception;

namespace
{
// x 축 둘레로 deg 만큼 돌린 자세: 태그 법선(z) = (0, −sin, cos)
cv::Vec3d tilt_x(double deg) {return cv::Vec3d(deg * M_PI / 180.0, 0, 0);}

TagDetection two_solutions(double tilt_best, double err_best, double tilt_alt, double err_alt)
{
  TagDetection t;
  t.pose_valid = true;
  t.rvec = tilt_x(tilt_best);
  t.tvec = cv::Vec3d(0.01, 0, 0.14);
  t.err = err_best;
  t.alt_valid = true;
  t.rvec_alt = tilt_x(tilt_alt);
  t.tvec_alt = cv::Vec3d(0.02, 0, 0.14);
  t.err_alt = err_alt;
  return t;
}
}  // namespace

TEST(TagUpright, AxisDeviation)
{
  EXPECT_NEAR(axis_deviation_deg({0, 0, 1}), 0.0, 1e-9);
  EXPECT_NEAR(axis_deviation_deg({0, 0, -2}), 0.0, 1e-9);
  EXPECT_NEAR(axis_deviation_deg({1, 0, 0}), 0.0, 1e-9);
  EXPECT_NEAR(axis_deviation_deg({0, std::sin(0.5236), std::cos(0.5236)}), 30.0, 1e-3);
  EXPECT_NEAR(axis_deviation_deg({1, 0, 1}), 45.0, 1e-9);
}

// 오차가 작은 해가 46° 기울고 다른 해가 거의 수직이며 오차 차가 작음 → 다른 해 (R6 렌더에서 본 경우)
TEST(TagUpright, PrefersVerticalWhenErrorsClose)
{
  const auto s = prefer_upright(two_solutions(46.0, 0.218, 0.9, 0.254), cv::Matx33d::eye(), 1.0);
  EXPECT_NEAR(s.rvec[0], tilt_x(0.9)[0], 1e-12);
  EXPECT_NEAR(s.tvec[0], 0.02, 1e-12);
  EXPECT_NEAR(s.err, 0.254, 1e-12);
  EXPECT_NEAR(s.rvec_alt[0], tilt_x(46.0)[0], 1e-12);
}

// 누운 퍽 끝면: 법선이 수평(base_link 기준). 카메라 → base_link 회전을 거쳐 판단
TEST(TagUpright, PrefersHorizontalInBaseFrame)
{
  const cv::Matx33d R_base_cam(1, 0, 0, 0, 0, 1, 0, -1, 0);   // 카메라 z → base −y (수평), 카메라 y → base z
  // 0° 해: base 법선 (0, −1, 0) 수평 / 40° 해: 수평에서 40° 기움
  const auto s = prefer_upright(two_solutions(40.0, 0.10, 0.0, 0.12), R_base_cam, 1.0);
  EXPECT_NEAR(s.rvec[0], 0.0, 1e-12);
}

// 오차 차가 기준보다 크거나, 끄거나, 다른 해가 없으면 그대로
TEST(TagUpright, KeepsBestOtherwise)
{
  EXPECT_NEAR(prefer_upright(two_solutions(46.0, 0.2, 0.9, 1.5), cv::Matx33d::eye(), 1.0).rvec[0], tilt_x(46.0)[0], 1e-12);
  EXPECT_NEAR(prefer_upright(two_solutions(46.0, 0.2, 0.9, 0.3), cv::Matx33d::eye(), 0.0).rvec[0], tilt_x(46.0)[0], 1e-12);
  auto t = two_solutions(46.0, 0.2, 0.9, 0.3);
  t.alt_valid = false;
  EXPECT_NEAR(prefer_upright(t, cv::Matx33d::eye(), 1.0).rvec[0], tilt_x(46.0)[0], 1e-12);
  // 이미 더 수직인 해를 골랐으면 그대로
  EXPECT_NEAR(prefer_upright(two_solutions(2.0, 0.2, 30.0, 0.21), cv::Matx33d::eye(), 1.0).rvec[0], tilt_x(2.0)[0], 1e-12);
}
