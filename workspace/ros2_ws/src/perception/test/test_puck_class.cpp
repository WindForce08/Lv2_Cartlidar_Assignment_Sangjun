// 검출 클래스 번호 → 퍽 색 · 모양 시험 (puck_class.hpp)
#include <gtest/gtest.h>

#include "perception/puck_class.hpp"

using namespace perception;

namespace
{
constexpr uint8_t UNK = 0, RED = 1, GREEN = 2, BLUE = 3, CYL = 1, CUB = 2;
}  // namespace

// 이름이 없으면 색 검출기 번호(0 빨강 · 1 초록 · 2 파랑), 모양은 모름
TEST(PuckClass, ColorDetectorNumbersWithoutNames)
{
  EXPECT_EQ(puck_class(0, {}).color, RED);
  EXPECT_EQ(puck_class(1, {}).color, GREEN);
  EXPECT_EQ(puck_class(2, {}).color, BLUE);
  EXPECT_EQ(puck_class(3, {}).color, UNK);
  EXPECT_EQ(puck_class(-1, {}).color, UNK);
  EXPECT_EQ(puck_class(0, {}).shape, UNK);
}

// YOLO 4 클래스: 번호 순서 그대로 색 · 모양. 예전 번호 대응이면 2 → 파랑 · 3 → 모름이 되던 것
TEST(PuckClass, YoloFourClasses)
{
  const std::vector<std::string> n{"red_cylinder", "green_cylinder", "red_cuboid", "green_cuboid"};
  const uint8_t want[4][2] = {{RED, CYL}, {GREEN, CYL}, {RED, CUB}, {GREEN, CUB}};
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(puck_class(i, n).color, want[i][0]) << i;
    EXPECT_EQ(puck_class(i, n).shape, want[i][1]) << i;
  }
  EXPECT_EQ(puck_class(4, n).color, UNK);
  EXPECT_EQ(puck_class(-1, n).shape, UNK);
}

// 낱말이 없으면 그 값만 모름
TEST(PuckClass, MissingWordIsUnknown)
{
  EXPECT_EQ(puck_class(0, {"cylinder"}).color, UNK);
  EXPECT_EQ(puck_class(0, {"cylinder"}).shape, CYL);
  EXPECT_EQ(puck_class(0, {"green"}).shape, UNK);
  EXPECT_EQ(puck_class(0, {"green"}).color, GREEN);
}

// 엔진 클래스 수와 class_names 대조: 색 검출기(-1)는 늘 통과, 이름 없는 엔진은 3 클래스만
TEST(PuckClass, ClassNamesMismatch)
{
  const std::vector<std::string> n4{"red_cylinder", "green_cylinder", "red_cuboid", "green_cuboid"};
  EXPECT_TRUE(class_names_mismatch(-1, {}).empty());
  EXPECT_TRUE(class_names_mismatch(-1, n4).empty());
  EXPECT_TRUE(class_names_mismatch(4, n4).empty());
  EXPECT_TRUE(class_names_mismatch(3, {}).empty());
  EXPECT_FALSE(class_names_mismatch(4, {}).empty());
  EXPECT_FALSE(class_names_mismatch(3, n4).empty());
  EXPECT_FALSE(class_names_mismatch(5, n4).empty());
}
