// ArmIK 시험: 0 자세 · 팔레트 이동 자세 FK (kinematics.py 와 같은 값), FK → IK → FK 왕복, 45° 집기 도달 범위
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "control/arm_ik.hpp"

using control::ArmIK;
using control::ArmIKParams;
using control::ArmJoints;

namespace
{
const ArmIK kIk{ArmIKParams{}};   // 기본값 = 차체 URDF 치수
}  // namespace

TEST(ArmIK, ZeroPoseForward)
{
  // 0 자세: 위팔 (0.024, 0.128), 아래팔 · 집게는 +x → 끝 (0.024 + 0.124 + 0.167, 0, 0.077 + 0.128)
  const Eigen::Vector3d p = kIk.forward(ArmJoints{0, 0, 0, 0, 0});
  EXPECT_NEAR(p.x(), 0.315, 1e-9);
  EXPECT_NEAR(p.y(), 0.0, 1e-9);
  EXPECT_NEAR(p.z(), 0.205, 1e-9);
  EXPECT_NEAR(kIk.forwardPitch(ArmJoints{0, 0, 0, 0, 0}), 0.0, 1e-12);
}

TEST(ArmIK, PalletTransitPoseIs45Degrees)
{
  // pallet_place_traj.json transit q_front_tilt — kinematics.py fk_tip 로 낸 값과 같아야 함
  const ArmJoints q{0.0, -1.32032, 0.67766, 1.42805, 0.0};
  const Eigen::Vector3d p = kIk.forward(q);
  EXPECT_NEAR(p.x(), 0.0992934, 1e-6);
  EXPECT_NEAR(p.z(), 0.0882085, 1e-6);
  EXPECT_NEAR(kIk.forwardPitch(q), -M_PI_4, 1e-4);   // 집게 45° 아래
}

TEST(ArmIK, RandomFkIkRoundTrip)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> j1(-2.5, 2.5), j2(-1.5, 1.2), j3(-1.4, 1.4), j4(-1.7, 1.8), j5(-2.0, 2.0);
  int solved = 0;
  for (int k = 0; k < 500; ++k) {
    const ArmJoints q{j1(rng), j2(rng), j3(rng), j4(rng), j5(rng)};
    const Eigen::Vector3d p = kIk.forward(q);
    if (std::hypot(p.x(), p.y()) < 0.03) {continue;}   // joint1 축 바로 위는 요가 정해지지 않음
    const double pitch = kIk.forwardPitch(q);
    for (bool up : {true, false}) {
      const auto s = kIk.solve(p, pitch, q[4], up);
      if (!s) {continue;}
      EXPECT_NEAR((kIk.forward(*s) - p).norm(), 0.0, 1e-6);
      EXPECT_NEAR(kIk.forwardPitch(*s), pitch, 1e-6);
      ++solved;
    }
  }
  EXPECT_GT(solved, 400);
}

TEST(ArmIK, UnreachableReturnsNullopt)
{
  EXPECT_FALSE(kIk.solve({0.60, 0.0, 0.0}, 0.0, 0.0, true));    // 팔 길이 0.42 m 밖
  EXPECT_FALSE(kIk.solve({0.0, 0.0, 0.60}, M_PI_2, 0.0, true));
}

TEST(ArmIK, TiltedPickReachOnBoxTop)
{
  // 박스 윗면 (바닥 70 mm) 위 퍽 집기: arm_base_link 는 바닥 위 0.12 m → 집게 끝 z = 0.10 + 0.015 - 0.12.
  // 45° 로 J1 거리 0.14 ~ 0.33 m 가 풀려야 함 (mission_planner README: 45° 도달 0.12 ~ 0.35)
  const double z = 0.100 + 0.015 - 0.120;
  for (double d = 0.14; d <= 0.33 + 1e-9; d += 0.01) {
    EXPECT_TRUE(kIk.solve({d, 0.0, z}, -M_PI_4, 0.0, true)) << "d " << d;
  }
}
