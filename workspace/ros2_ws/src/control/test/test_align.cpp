// /drive/align 제어 법칙 시험: 부호, 포화, 정착 판정, 간단한 폐루프 시뮬레이션
#include <gtest/gtest.h>

#include <cmath>

#include "control/align_controller.hpp"

using namespace control;

TEST(Align, SignsAndSaturation)
{
  AlignController c;
  auto o = c.step(0.5, 0.40, 0.15, 0.01);   // 물체가 오른쪽 · 멀리
  EXPECT_LT(o.w, 0.0);                       // 오른쪽(시계 방향)으로 회전
  EXPECT_GE(o.w, -0.6);
  EXPECT_GT(o.v, 0.0);                       // 앞으로
  EXPECT_LE(o.v, 0.08);
  o = c.step(-0.5, 0.12, 0.15, 0.01);       // 왼쪽 · 너무 가까움
  EXPECT_GT(o.w, 0.0);
  EXPECT_LT(o.v, 0.0);
  EXPECT_FALSE(o.done);
}

TEST(Align, SettlesOnlyAfterConsecutiveCycles)
{
  AlignGains g;
  g.settle_cycles = 5;
  AlignController c(g);
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(c.step(0.0, 0.15, 0.15, 0.01).done);}
  EXPECT_FALSE(c.step(0.2, 0.15, 0.15, 0.01).done);   // 한 번 벗어나면 다시 셈
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(c.step(0.0, 0.151, 0.15, 0.01).done);}
  const auto o = c.step(0.0, 0.15, 0.15, 0.01);
  EXPECT_TRUE(o.done);
  EXPECT_DOUBLE_EQ(o.v, 0.0);
  EXPECT_DOUBLE_EQ(o.w, 0.0);
}

TEST(Align, ClosedLoopConverges)
{
  // 단순 모델: 가로 오차는 회전에 비례해 줄고(화면 0.7 rad 폭), 거리는 전진 속도로 줄어듦
  AlignController c;
  double lateral = 0.6, dist = 0.45;
  const double dt = 0.05;
  bool done = false;
  for (int k = 0; k < 400 && !done; ++k) {
    const auto o = c.step(lateral, dist, 0.15, 0.01);
    done = o.done;
    lateral += o.w * dt / 0.7 * 2.0;   // w<0(오른쪽) → 오른쪽 오차 감소
    dist -= o.v * dt;
  }
  EXPECT_TRUE(done);
  EXPECT_NEAR(dist, 0.15, 0.01);
  EXPECT_NEAR(lateral, 0.0, 0.04);
}

TEST(CoarseAlign, TurnsFirstThenDrivesToBumperDistance)
{
  CoarseAlign c;
  auto o = c.step(1.0, 0.8, 0.2, 0.4, 0.02);   // 물체가 왼쪽 38°
  EXPECT_GT(o.w, 0.0);
  EXPECT_DOUBLE_EQ(o.v, 0.0);                   // 각도가 크면 제자리 회전 먼저
  o = c.step(1.0, 0.0, 0.2, 0.4, 0.02);         // 정면 · 범퍼 기준 0.8 m → 0.4 m 까지 전진
  EXPECT_GT(o.v, 0.0);
  o = c.step(0.5, 0.0, 0.2, 0.4, 0.02);         // 범퍼 기준 0.3 m (더 가까움) → 후진
  EXPECT_LT(o.v, 0.0);
  CoarseAlign d;
  bool done = false;
  for (int i = 0; i < 10 && !done; ++i) {done = d.step(0.6, 0.0, 0.2, 0.4, 0.02).done;}
  EXPECT_TRUE(done);
}

TEST(ScanSafety, FrontAndRearSectors)
{
  // 360 개, angle_min -π, 1° 간격
  std::vector<float> r(360, 5.0f);
  const double amin = -M_PI, inc = M_PI / 180;
  EXPECT_FALSE(scan_blocked(r, amin, inc, 0.0, 0.349, 0.9));
  r[180 + 10] = 0.5f;                             // 전방 +10° 에 0.5 m
  EXPECT_TRUE(scan_blocked(r, amin, inc, 0.0, 0.349, 0.9));
  EXPECT_FALSE(scan_blocked(r, amin, inc, M_PI, 0.349, 0.3));   // 후방은 비어 있음
  r[5] = 0.2f;                                    // 후방 (-175°) 0.2 m
  EXPECT_TRUE(scan_blocked(r, amin, inc, M_PI, 0.349, 0.3));
}
