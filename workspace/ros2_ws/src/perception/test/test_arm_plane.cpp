// 팔 카메라 평면 교차 시험 (arm_plane.hpp)
#include <gtest/gtest.h>

#include <cmath>

#include "perception/arm_plane.hpp"

using namespace perception;

namespace
{
const cv::Matx33d K(500, 0, 320, 0, 500, 240, 0, 0, 1);
// 카메라 (0, 0, 0.4) 에서 +x 쪽을 아래로 pitch 만큼 내려다봄. 광학 z 앞 · x 오른쪽 · y 아래 → 평면 좌표 (x 앞 · y 왼쪽 · z 위)
cv::Matx33d look_down(double pitch)
{
  const double c = std::cos(pitch), s = std::sin(pitch);
  // 광학 x(오른쪽) = −y, 광학 y(아래) = 앞 성분 −sin · 아래 −cos, 광학 z(앞) = 앞 cos · 아래 −sin
  return cv::Matx33d(0, -s, c, -1, 0, 0, 0, -c, -s);
}
// 점 p 를 화소로
cv::Point2d project(const cv::Matx33d & R, const cv::Vec3d & t, const cv::Vec3d & p)
{
  const cv::Vec3d q = R.t() * (p - t);
  return {K(0, 0) * q[0] / q[2] + K(0, 2), K(1, 1) * q[1] / q[2] + K(1, 2)};
}
}  // namespace

// 받침 · 반높이 조합 중 D435 높이에 가장 가까운 것. 바닥 서 있음(0.03)은 D435 가 5 mm 낮게 재도 0.03
TEST(ArmPlane, PicksNearestSupportAndHalfHeight)
{
  const std::vector<double> levels{0.0, 0.07}, halves{0.015, 0.03};
  EXPECT_NEAR(*pick_plane_height(0.025, levels, halves, 0.02), 0.03, 1e-12);
  EXPECT_NEAR(*pick_plane_height(0.011, levels, halves, 0.02), 0.015, 1e-12);
  EXPECT_NEAR(*pick_plane_height(0.0953, levels, halves, 0.02), 0.10, 1e-12);   // 출발 상자 위 서 있는 퍽
  EXPECT_FALSE(pick_plane_height(0.30, levels, halves, 0.02));                   // 목록과 먼 높이(선반 등)는 내지 않음
  EXPECT_FALSE(pick_plane_height(std::nan(""), levels, halves, 0.02));
}

// 투영한 점을 다시 같은 높이 평면과 교차하면 원래 점
TEST(ArmPlane, RayPlaneRecoversProjectedPoint)
{
  const cv::Vec3d t(0, 0, 0.4);
  for (double pitch : {0.5, 0.9, 1.3}) {
    const cv::Matx33d R = look_down(pitch);
    for (const cv::Vec3d p : {cv::Vec3d(0.3, 0.05, 0.03), cv::Vec3d(0.25, -0.08, 0.015), cv::Vec3d(0.45, 0.0, 0.10)}) {
      const cv::Point2d px = project(R, t, p);
      const auto q = ray_plane(K, px.x, px.y, R, t, p[2]);
      ASSERT_TRUE(q) << pitch;
      EXPECT_NEAR(cv::norm(*q - p), 0.0, 1e-9) << pitch;
    }
  }
}

// 수평 시선(평면과 나란함) · 평면이 카메라 뒤(위쪽을 보는 시선)는 없음
TEST(ArmPlane, RejectsParallelAndBehind)
{
  const cv::Vec3d t(0, 0, 0.4);
  const cv::Matx33d level = look_down(0.0);
  EXPECT_FALSE(ray_plane(K, 320, 240, level, t, 0.03));
  EXPECT_FALSE(ray_plane(K, 320, 0, level, t, 0.03));   // 화면 위쪽 = 위를 보는 시선 → 아래 평면과 만나지 않음
}
