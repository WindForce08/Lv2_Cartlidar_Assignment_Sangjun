// TargetLookPose 시험 (#82): 타겟체크자세 후보의 카메라 광축이 퍽 윗면을 지나는지, 계획기 퍽 자리가 모두 관절 한계 안에서 풀리는지,
// 인지가 #82 에서 낸 타겟체크자세 (같은 URDF 치수) 가 이 카메라 모형으로도 퍽을 화면 가운데에 두는지
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>

#include "control/arm_ik.hpp"
#include "control/target_look_pose.hpp"

using control::ArmIK;
using control::ArmIKParams;
using control::ArmJoints;
using control::TargetLookParams;

namespace
{
const ArmIK kIk{ArmIKParams{}};
const TargetLookParams kTargetLook{};
// 퍽 윗면 = 바닥 0.13 (상자 70 + 퍽 60), arm_base_link 바닥 위 0.12025 (B안)
constexpr double kAimZ = 0.13 - 0.12025;
// manipulator.yaml lower/upper_limits (joint1~5) = URDF
constexpr std::array<double, 5> kLo{-3.14159, -2.14, -1.6, -1.92, -2.8};
constexpr std::array<double, 5> kHi{3.14159, 1.6, 1.6, 2.0, 2.8};

double rayDistance(const Eigen::Vector3d & cam, const Eigen::Vector3d & axis, const Eigen::Vector3d & p)
{
  const Eigen::Vector3d v = p - cam;
  return (v - axis * v.dot(axis)).norm();
}

std::optional<ArmJoints> solveTargetLook(const Eigen::Vector3d & aim)
{
  for (const auto & c : control::targetLookCandidates(aim, kTargetLook)) {
    const auto q = kIk.solve(c.tool, c.pitch, 0.0, true);
    if (!q || (kIk.forward(*q) - c.tool).norm() > 1e-6) {continue;}
    bool ok = true;
    for (size_t i = 0; i < 5; ++i) {ok = ok && (*q)[i] >= kLo[i] && (*q)[i] <= kHi[i];}
    if (ok) {return q;}
  }
  return std::nullopt;
}
}  // namespace

TEST(TargetLookPose, CameraAxisPassesThroughAim)
{
  for (const Eigen::Vector3d & aim : {Eigen::Vector3d(0.261, -0.08, kAimZ), Eigen::Vector3d(0.226, 0.0, kAimZ),
      Eigen::Vector3d(0.15, 0.05, 0.03)})
  {
    const auto cands = control::targetLookCandidates(aim, kTargetLook);
    ASSERT_FALSE(cands.empty());
    for (const auto & c : cands) {
      Eigen::Vector3d cam, axis;
      control::targetLookCamera(c.tool, c.pitch, std::atan2(aim.y(), aim.x()), kTargetLook, cam, axis);
      EXPECT_NEAR((aim - cam).norm(), c.distance, 1e-9);
      EXPECT_NEAR(rayDistance(cam, axis, aim), 0.0, 1e-9);
      EXPECT_GE(c.tool.z(), aim.z() + kTargetLook.min_clearance - 1e-12);
    }
  }
}

TEST(TargetLookPose, PlannerPuckPositionsReachable)
{
  // mission_planner 접근 자세표 (#64 안 A, FINE 자리) 회전 뒤 퍽 위치 (arm_base_link):
  //   상자 퍽 ±Y 접근 회전 뒤 (0.261, ±0.08) · inline 가까운 · 먼 퍽 (0.226 · 0.306, 0) · 인지 #82 의 경우들
  for (const auto & xy : std::array<std::array<double, 2>, 8>{{{0.261, 0.08}, {0.261, -0.08}, {0.226, 0.0}, {0.306, 0.0},
      {0.203, 0.04}, {0.205, 0.06}, {0.139, 0.0}, {0.219, 0.0}}})
  {
    const Eigen::Vector3d aim(xy[0], xy[1], kAimZ);
    const auto q = solveTargetLook(aim);
    ASSERT_TRUE(q.has_value()) << "no look pose for (" << xy[0] << ", " << xy[1] << ")";
    Eigen::Vector3d cam, axis;
    control::targetLookCamera(kIk.forward(*q), kIk.forwardPitch(*q), (*q)[0], kTargetLook, cam, axis);
    EXPECT_LT(rayDistance(cam, axis, aim), 1e-6);
  }
}

TEST(TargetLookPose, PerceptionIssue82PosesCenterThePuck)
{
  // 인지 #82 댓글 (정기구학 투영, 같은 URDF): 이 자세들에서 퍽 윗면 가운데가 화면 가운데.
  // 같은 자세를 이 모형에 넣으면 광축이 퍽 윗면 가까이 (1 cm 안) 를 지나야 카메라 장착 치수가 같다
  struct Case {double r; double q2, q3, q4;};
  for (const Case & c : {Case{0.203, -0.98, 0.42, 1.54}, Case{0.139, -1.12, 0.36, 1.88}, Case{0.219, -0.90, 0.34, 1.52}}) {
    const ArmJoints q{0.0, c.q2, c.q3, c.q4, 0.0};
    Eigen::Vector3d cam, axis;
    control::targetLookCamera(kIk.forward(q), kIk.forwardPitch(q), 0.0, kTargetLook, cam, axis);
    EXPECT_LT(rayDistance(cam, axis, Eigen::Vector3d(c.r, 0.0, kAimZ)), 0.01) << "r " << c.r;
  }
}
