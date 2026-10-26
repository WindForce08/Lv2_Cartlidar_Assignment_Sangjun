// ARM_ALIGN 목표 퍽 유지 시험 (arm_target_lock.hpp). 정규화 좌표, 빨강 0 · 초록 1
#include <gtest/gtest.h>

#include "perception/arm_target_lock.hpp"

using namespace perception;

namespace
{
constexpr int RED = 0, GREEN = 1;
ArmBox box(double x, int cls, double score, double w = 0.10) {return {x, 0.0, w, cls, score};}
}  // namespace

// 퍽 A(왼쪽) · B(오른쪽, 0.3 옆). 처음에 A 점수가 높아 A 를 잡고, 뒤에 B 점수가 높아져도 A 를 유지
TEST(ArmLock, KeepsTargetWhenScoresSwap)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(-0.12, RED, 0.5), box(0.17, RED, 0.95)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.18, RED, 0.95), box(-0.10, RED, 0.4)}, std::nullopt), 1);   // 목록 순서가 바뀌어도 A
}

// A 가 안 보이면 -1 (B 로 넘어가지 않음), 직전 자리 근처에 다시 보이면 이어 간다
TEST(ArmLock, LostTargetDoesNotSwitchAndReacquires)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9)}, std::nullopt), -1);
  EXPECT_EQ(lock.update({}, std::nullopt), -1);
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9), box(-0.10, RED, 0.5)}, std::nullopt), 1);
  EXPECT_TRUE(lock.locked());
  lock.reset();   // ARM_ALIGN 끄고 다시 켬 → 새로 고름
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9)}, std::nullopt), 0);
}

// 처음 고를 때 D435 목표 투영점(B 자리)에 가까운 박스. 점수가 높은 A 가 아님. 투영점 근처에 없으면 잠그지 않음
TEST(ArmLock, SeedPicksProjectedTarget)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, ArmPoint{0.13, 0.02}), 1);
  ArmTargetLock lock2;
  EXPECT_EQ(lock2.update({box(-0.15, RED, 0.9)}, ArmPoint{0.40, 0.0}), -1);
  EXPECT_FALSE(lock2.locked());
}

// 바로 옆이라도 다른 색은 잡지 않는다
TEST(ArmLock, DifferentClassNotTaken)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.02, GREEN, 0.9)}, std::nullopt), -1);
}

// 폭이 2 배 넘게 달라지면 다른 물체로 본다 (가까워지며 조금씩 커지는 것은 따라감)
TEST(ArmLock, SizeJumpNotTaken)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9, 0.10)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.01, RED, 0.9, 0.13)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.02, RED, 0.9, 0.30)}, std::nullopt), -1);
}
