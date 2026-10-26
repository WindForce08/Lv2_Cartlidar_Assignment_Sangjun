// 라이다 합치기 · 비상 정지 판정 시험
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "perception/obstacle_stop.hpp"
#include "perception/scan_merge.hpp"

using namespace perception;

TEST(ScanBinner, BinsByAngleAndKeepsNearest)
{
  MergeConfig c;   // 360 칸, -π 부터
  ScanBinner b(c);
  EXPECT_NEAR(b.increment(), M_PI / 180.0, 1e-12);
  EXPECT_EQ(b.bin_of(-M_PI), 0);
  EXPECT_EQ(b.bin_of(0.0), 180);                 // 앞
  EXPECT_EQ(b.bin_of(M_PI / 2), 270);            // 왼쪽
  EXPECT_EQ(b.bin_of(M_PI - 1e-9), 0);           // +π 는 -π 칸과 같은 방향
  EXPECT_TRUE(b.add(2.0, 0.0));
  EXPECT_TRUE(b.add(1.0, 0.0));                  // 같은 칸에 더 가까운 점
  EXPECT_FLOAT_EQ(b.ranges()[180], 1.0f);
  EXPECT_TRUE(std::isinf(b.ranges()[90]));       // 점이 없는 칸
}

TEST(ScanBinner, DropsSelfBoxAndOutOfRange)
{
  MergeConfig c;
  c.range_max = 5.0;
  c.self_box = {-0.2285, 0.2285, -0.1626, 0.1626};
  ScanBinner b(c);
  EXPECT_FALSE(b.add(0.2, 0.1));                 // 차체 안
  EXPECT_FALSE(b.add(6.0, 0.0));                 // 범위 밖
  EXPECT_FALSE(b.add(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_TRUE(b.add(0.0, 0.5));                  // 차체 밖 왼쪽
  EXPECT_FLOAT_EQ(b.ranges()[270], 0.5f);
}

// 뒤집힌(roll π) 라이다의 점을 TF 대신 손으로 바꿔 넣어도 같은 칸에 들어가는지 (앞 라이다 위치 x 0.1855, y 0.05)
TEST(ScanBinner, UpsideDownLidarMirrorsAngle)
{
  MergeConfig c;
  ScanBinner b(c);
  const double r = 1.0, a = 0.5;                 // 센서 기준 +0.5 rad 방향 점
  // roll π: (x, y, z) → (x, -y, -z). 그다음 장착 위치만큼 이동
  const double x = 0.1855 + r * std::cos(a), y = 0.05 - r * std::sin(a);
  EXPECT_TRUE(b.add(x, y));
  EXPECT_EQ(b.bin_of(std::atan2(y, x)), b.bin_of(std::atan2(0.05 - std::sin(0.5), 0.1855 + std::cos(0.5))));
  EXPECT_LT(std::atan2(y, x), 0.0);              // 센서 기준 왼쪽(+) 이 차체 기준 오른쪽(-) 으로
}

TEST(ObstacleStop, StartsStoppedAndReleasesAfterCount)
{
  StopConfig c;   // stop 0.20, release 0.30, count 3, near 0.30, stale 0.2
  ObstacleStop s(c);
  EXPECT_TRUE(s.stopped());                      // 첫 값 전에는 정지
  EXPECT_TRUE(s.update(0.00, 1.0));
  EXPECT_TRUE(s.update(0.02, 1.0));
  EXPECT_FALSE(s.update(0.04, 1.0));             // 연속 3번 → 해제
  EXPECT_EQ(s.reason(), "clear");
}

TEST(ObstacleStop, StopsAndHysteresis)
{
  StopConfig c;
  ObstacleStop s(c);
  for (int i = 0; i < 3; ++i) {s.update(0.02 * i, 1.0);}
  ASSERT_FALSE(s.stopped());
  EXPECT_FALSE(s.update(0.10, 0.25));            // 0.20 ~ 0.30 사이: 아직 정지 아님
  EXPECT_TRUE(s.update(0.12, 0.19));             // 0.20 미만 → 정지
  EXPECT_EQ(s.reason(), "obstacle");
  EXPECT_TRUE(s.update(0.14, 0.25));             // 사이 값은 정지 유지
  EXPECT_TRUE(s.update(0.16, 0.35));
  EXPECT_TRUE(s.update(0.18, 0.35));
  EXPECT_TRUE(s.update(0.19, 0.25));             // 연속이 끊김
  EXPECT_TRUE(s.update(0.20, 0.35));
  EXPECT_TRUE(s.update(0.21, 0.35));
  EXPECT_FALSE(s.update(0.22, 0.35));            // 다시 연속 3번 → 해제
}

TEST(ObstacleStop, InvalidAfterNearMeansTooClose)
{
  StopConfig c;
  ObstacleStop s(c);
  for (int i = 0; i < 3; ++i) {s.update(0.02 * i, 1.0);}
  s.update(0.10, 0.15);                          // 가까움 → 정지
  EXPECT_TRUE(s.update(0.12, std::nullopt));     // 최소 거리 안으로 들어가 무효 → 정지 유지
  EXPECT_EQ(s.reason(), "too_close");
  EXPECT_TRUE(s.update(0.14, std::nullopt));
}

TEST(ObstacleStop, InvalidWhenFarMeansClear)
{
  StopConfig c;
  ObstacleStop s(c);
  for (int i = 0; i < 3; ++i) {s.update(0.02 * i, 2.0);}
  EXPECT_FALSE(s.update(0.10, std::nullopt));    // 직전 2.0 m → 앞이 비었음
  EXPECT_EQ(s.reason(), "no_return");
}

TEST(ObstacleStop, StaleInputStops)
{
  StopConfig c;
  ObstacleStop s(c);
  for (int i = 0; i < 3; ++i) {s.update(0.02 * i, 1.0);}
  EXPECT_FALSE(s.check_stale(0.15));
  EXPECT_TRUE(s.check_stale(0.30));              // 마지막 값 0.04 s 뒤 0.26 s 없음
  EXPECT_EQ(s.reason(), "stale");
  EXPECT_TRUE(s.update(0.31, 1.0));              // 다시 들어와도 연속 3번 전에는 정지
  EXPECT_TRUE(s.update(0.33, 1.0));
  EXPECT_FALSE(s.update(0.35, 1.0));
}
