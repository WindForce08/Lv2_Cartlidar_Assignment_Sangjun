// /arm_range 계산 시험 (arm_range.hpp): 태그 우선 · D435 보유 점 · 옆 거리 문턱 · 보유 시간
#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "perception/arm_range.hpp"

using namespace perception;

TEST(ArmRange, TagPreferredOverHold)
{
  ArmRangeConfig c;
  const auto e = arm_range({{0.150, 0.004, -0.002}}, P3{0.160, 0.010, 0.0}, 0.5, c);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->source, ArmRangeEstimate::Source::TAG);
  EXPECT_NEAR(e->range, 0.135, 1e-9);            // 중심 0.150 − 반지름 0.015
}

TEST(ArmRange, NeighborTagIgnoredUsesHold)
{
  ArmRangeConfig c;
  // 이웃 퍽(옆 40 mm) 태그만 보임 → 쓰지 않고 D435 보유 점
  const auto e = arm_range({{0.15, 0.040, 0.0}}, P3{0.170, 0.008, 0.0}, 1.0, c);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->source, ArmRangeEstimate::Source::D435_HOLD);
  EXPECT_NEAR(e->range, 0.155, 1e-9);
}

TEST(ArmRange, ClosestTagOnAxis)
{
  ArmRangeConfig c;
  const auto e = arm_range({{0.15, 0.020, 0.0}, {0.18, 0.002, 0.0}}, std::nullopt, 0.0, c);
  ASSERT_TRUE(e);
  EXPECT_NEAR(e->range, 0.165, 1e-9);            // 팔 축에 더 가까운 퍽
}

TEST(ArmRange, HoldExpiresAndGates)
{
  ArmRangeConfig c;   // hold 3 s, seed_gate 0.05
  EXPECT_FALSE(arm_range({}, P3{0.17, 0.0, 0.0}, 3.5, c));     // 보유 시간 지남
  EXPECT_FALSE(arm_range({}, P3{0.17, 0.06, 0.0}, 1.0, c));    // 팔 축에서 너무 옆
  EXPECT_FALSE(arm_range({}, P3{-0.02, 0.0, 0.0}, 1.0, c));    // 팔 뒤
  EXPECT_FALSE(arm_range({}, std::nullopt, 0.0, c));
}

TEST(ArmRange, ClampsInsideSurface)
{
  ArmRangeConfig c;
  const auto e = arm_range({{0.010, 0.0, 0.0}}, std::nullopt, 0.0, c);
  ASSERT_TRUE(e);
  EXPECT_DOUBLE_EQ(e->range, 0.0);               // 끝단이 표면을 지남 → 0
}
