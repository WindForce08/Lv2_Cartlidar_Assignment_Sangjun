// ArmIK 시험: FK → IK → FK 왕복, 도달 불가 · 관절 한계, 손목 롤
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "control/arm_ik.hpp"

using namespace control;

TEST(ArmIK, RoundTripTopDownGrid)
{
  ArmIK ik(ArmGeometry{}, JointLimits{});
  int solved = 0, tried = 0;
  for (double x = 0.12; x <= 0.26; x += 0.02) {
    for (double y = -0.10; y <= 0.10; y += 0.05) {
      for (double z = -0.10; z <= 0.05; z += 0.03) {
        IkRequest r;
        r.position = {x, y, z};
        r.tool_pitch = -M_PI / 2;
        r.pitch_fallbacks.clear();
        std::string why;
        auto q = ik.solve(r, why);
        ++tried;
        if (!q) {continue;}
        ++solved;
        const Eigen::Vector3d p = ik.fk_position(*q);
        EXPECT_NEAR((p - r.position).norm(), 0.0, 1e-6) << x << " " << y << " " << z;
        EXPECT_NEAR(ArmIK::tool_pitch(*q), -M_PI / 2, 1e-6);
        EXPECT_TRUE(ik.limits().within(*q));
      }
    }
  }
  // 집기 작업 영역(앞 12~26 cm, 바닥 근처)의 대부분은 위에서 집기로 닿아야 함
  EXPECT_GT(solved, tried * 7 / 10) << solved << "/" << tried;
}

TEST(ArmIK, RandomFkIkConsistency)
{
  ArmIK ik(ArmGeometry{}, JointLimits{});
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  int checked = 0;
  for (int n = 0; n < 2000; ++n) {
    Joints q{u(rng) * 2.5, -0.5 + u(rng) * 1.0, u(rng) * 1.2, u(rng) * 1.5, 0.0, 0.0};
    if (!ik.limits().within(q)) {continue;}
    IkRequest r;
    r.position = ik.fk_position(q);
    r.tool_pitch = ArmIK::tool_pitch(q);
    r.object_yaw = q[0];
    r.pitch_fallbacks.clear();
    std::string why;
    auto s = ik.solve(r, why);
    if (!s) {continue;}   // 다른 팔꿈치 해가 한계 밖일 수 있음
    ++checked;
    EXPECT_NEAR((ik.fk_position(*s) - r.position).norm(), 0.0, 1e-6);
    EXPECT_NEAR(ArmIK::tool_pitch(*s), r.tool_pitch, 1e-6);
  }
  EXPECT_GT(checked, 300);
}

TEST(ArmIK, UnreachableAndWristRoll)
{
  ArmIK ik(ArmGeometry{}, JointLimits{});
  IkRequest far;
  far.position = {0.6, 0.0, 0.0};
  std::string why;
  EXPECT_FALSE(ik.solve(far, why));
  EXPECT_FALSE(why.empty());

  IkRequest r;
  r.position = {0.18, 0.0, -0.05};
  r.object_yaw = 0.3;          // 물체가 30° 쯤 돌아 있음 → joint5 = 0.3
  auto q = ik.solve(r, why);
  ASSERT_TRUE(q) << why;
  EXPECT_NEAR((*q)[0], 0.0, 1e-9);
  EXPECT_NEAR((*q)[4], 0.3, 1e-9);
  r.object_yaw = 2.0;          // 그리퍼 대칭 → ±90° 안으로 접힘
  q = ik.solve(r, why);
  ASSERT_TRUE(q);
  EXPECT_NEAR((*q)[4], 2.0 - M_PI, 1e-9);
}

TEST(ArmIK, LimitViolationReported)
{
  JointLimits lim;
  Joints q{0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(lim.within(q));
  q[2] = 1.6;   // joint3 상한 1.5
  EXPECT_FALSE(lim.within(q));
  EXPECT_NE(lim.violation(q).find("joint3"), std::string::npos);
  EXPECT_TRUE(lim.within(q, 0.15));   // 여유 포함이면 통과
}
