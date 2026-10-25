// 선반 위 3×3 주사위 확인 시험 (판정 기준안 v1 U3, 영상 없음)
//   칸 간격 0.05 m 는 시험용 값 (실제 칸 크기는 아직 모름 → 칸 중심은 요청으로 받음)
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "perception/dice_check.hpp"

using namespace perception;

namespace
{
constexpr double PITCH = 0.05, X0 = 1.0, Y0 = 2.0;   // 가운데 칸 (2,2) 중심

// 주사위 눈 배치 (칸 번호 0~8, 행 우선). 시험 자료일 뿐, 어느 방향으로 놓을지는 판단이 정한다
const std::vector<std::vector<int>> PIPS = {
  {}, {4}, {0, 8}, {0, 4, 8}, {0, 2, 6, 8}, {0, 2, 4, 6, 8}, {0, 3, 6, 2, 5, 8}};

DiceRequest request(int pips, uint8_t expect, uint8_t yellow_shape = SHAPE_CYLINDER)
{
  DiceRequest q;
  q.yellow_shape = yellow_shape;
  for (int i = 0; i < 9; ++i) {
    q.cells[i].x = X0 + (i / 3 - 1) * PITCH;
    q.cells[i].y = Y0 + (i % 3 - 1) * PITCH;
    q.cells[i].expect = DiceCell::EMPTY;
  }
  for (int i : PIPS[pips]) {q.cells[i].expect = expect;}
  return q;
}

// 요청대로 맞게 놓은 퍽. 빨강 · 초록 칸은 모양 무작위, 노랑 칸은 색 무작위
std::vector<SeenPuck> correct(const DiceRequest & q, std::mt19937 & rng, double jitter = 0.010)
{
  std::uniform_real_distribution<double> j(-jitter / std::sqrt(2.0), jitter / std::sqrt(2.0));
  std::uniform_int_distribution<int> pick(1, 2);
  std::vector<SeenPuck> v;
  for (const auto & c : q.cells) {
    if (c.expect == DiceCell::EMPTY) {continue;}
    SeenPuck p;
    p.x = c.x + j(rng);
    p.y = c.y + j(rng);
    if (c.expect == DiceCell::YELLOW) {
      p.color = static_cast<uint8_t>(pick(rng));
      p.shape = q.yellow_shape;
    } else {
      p.color = c.expect == DiceCell::RED ? COLOR_RED : COLOR_GREEN;
      p.shape = static_cast<uint8_t>(pick(rng));
    }
    v.push_back(p);
  }
  return v;
}
}  // namespace

// 눈 1~6 × 빨강 · 초록 · 노랑(원기둥 · 직육면체) 맞는 배치는 모두 통과
TEST(DiceCheck, U3_CorrectPatternsPass)
{
  std::mt19937 rng(7);
  for (int pips = 1; pips <= 6; ++pips) {
    for (uint8_t e : {DiceCell::RED, DiceCell::GREEN}) {
      const auto q = request(pips, e);
      for (int k = 0; k < 20; ++k) {
        const auto r = check_dice(q, correct(q, rng));
        EXPECT_TRUE(r.passed) << "눈 " << pips << " 색 " << int(e);
        EXPECT_TRUE(r.wrong.empty());
      }
    }
    for (uint8_t shape : {SHAPE_CYLINDER, SHAPE_CUBOID}) {
      const auto q = request(pips, DiceCell::YELLOW, shape);
      for (int k = 0; k < 20; ++k) {
        EXPECT_TRUE(check_dice(q, correct(q, rng)).passed) << "노랑 눈 " << pips << " 모양 " << int(shape);
      }
    }
  }
}

TEST(DiceCheck, U3_WrongColor)
{
  std::mt19937 rng(8);
  const auto q = request(4, DiceCell::RED);
  auto v = correct(q, rng);
  v[1].color = COLOR_GREEN;          // 칸 2 (1,3)
  const auto r = check_dice(q, v);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.wrong, std::vector<int>({2}));
  EXPECT_EQ(r.cells[2].error, CellResult::WRONG_COLOR);
}

TEST(DiceCheck, U3_YellowMixedShapes)
{
  std::mt19937 rng(9);
  const auto q = request(2, DiceCell::YELLOW, SHAPE_CUBOID);
  auto v = correct(q, rng);
  v[1].shape = SHAPE_CYLINDER;       // 칸 8 만 다른 모양
  const auto r = check_dice(q, v);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.wrong, std::vector<int>({8}));
  EXPECT_EQ(r.cells[8].error, CellResult::WRONG_SHAPE);
}

TEST(DiceCheck, U3_MissingAndUnexpected)
{
  std::mt19937 rng(10);
  const auto q = request(5, DiceCell::GREEN);
  auto v = correct(q, rng);
  v.erase(v.begin() + 2);            // 칸 4 (가운데) 빠짐
  SeenPuck extra;
  extra.x = q.cells[1].x;
  extra.y = q.cells[1].y;
  extra.color = COLOR_GREEN;
  v.push_back(extra);                // 비워야 할 칸 1 에 퍽
  const auto r = check_dice(q, v);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.wrong, std::vector<int>({1, 4}));
  EXPECT_EQ(r.cells[1].error, CellResult::UNEXPECTED);
  EXPECT_EQ(r.cells[4].error, CellResult::MISSING);
}

TEST(DiceCheck, U3_TwoPucksInOneCell)
{
  std::mt19937 rng(11);
  const auto q = request(1, DiceCell::RED);
  auto v = correct(q, rng);
  SeenPuck p = v[0];
  p.x += 0.008;
  v.push_back(p);
  const auto r = check_dice(q, v);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.wrong, std::vector<int>({4}));
  EXPECT_EQ(r.cells[4].error, CellResult::MULTIPLE);
  EXPECT_EQ(r.cells[4].count, 2);
}

TEST(DiceCheck, U3_OffsetBeyondTolerance)
{
  std::mt19937 rng(12);
  const auto q = request(3, DiceCell::RED);      // 칸 0 · 4 · 8, 허용 15 mm
  auto v = correct(q, rng, 0.0);
  v[0].x += 0.020;                               // 칸 0 에서 20 mm
  v[2].y -= 0.014;                               // 칸 8 에서 14 mm (허용 안)
  const auto r = check_dice(q, v);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.wrong, std::vector<int>({0}));
  EXPECT_EQ(r.cells[0].error, CellResult::OFFSET);
  EXPECT_NEAR(r.cells[0].offset, 0.020, 1e-9);
  EXPECT_EQ(r.cells[8].error, CellResult::OK);
}

TEST(DiceCheck, U3_UnknownAttributesFail)
{
  std::mt19937 rng(13);
  auto q = request(1, DiceCell::RED);
  auto v = correct(q, rng);
  v[0].color = COLOR_UNKNOWN;
  EXPECT_EQ(check_dice(q, v).cells[4].error, CellResult::WRONG_COLOR);
  q = request(1, DiceCell::YELLOW);
  v = correct(q, rng);
  v[0].shape = SHAPE_UNKNOWN;
  EXPECT_EQ(check_dice(q, v).cells[4].error, CellResult::WRONG_SHAPE);
}

TEST(DiceCheck, U3_EmptyShelf)
{
  const auto q = request(6, DiceCell::GREEN);
  const auto r = check_dice(q, {});
  EXPECT_FALSE(r.passed);
  EXPECT_EQ(r.wrong, std::vector<int>({0, 2, 3, 5, 6, 8}));
}
