// 입력 영상 시각 규칙 시험 (StampGate · NearestCache)
#include <gtest/gtest.h>

#include <string>

#include "perception/frame_gate.hpp"

using namespace perception;

TEST(StampGate, DropsDuplicateAndOlderFrames)
{
  StampGate g(1'000'000'000);
  EXPECT_EQ(g.check(5'000'000'000), GateResult::Accept);
  EXPECT_EQ(g.check(5'000'000'000), GateResult::Duplicate);   // 같은 영상 재전송
  EXPECT_EQ(g.check(4'900'000'000), GateResult::Duplicate);   // 0.1 s 과거
  EXPECT_EQ(g.check(5'033'000'000), GateResult::Accept);
}

TEST(StampGate, AcceptsClockRestart)
{
  StampGate g(1'000'000'000);
  EXPECT_EQ(g.check(60'000'000'000), GateResult::Accept);
  EXPECT_EQ(g.check(100'000'000), GateResult::Restart);   // 시뮬레이션 리셋: 60 s → 0.1 s
  EXPECT_EQ(g.check(133'000'000), GateResult::Accept);    // 재시작 뒤 영상은 계속 받는다
  g.reset();
  EXPECT_EQ(g.check(1), GateResult::Accept);
}

TEST(NearestCache, PicksClosestWithinTolerance)
{
  NearestCache<std::string> c(3);
  c.push(1000, "a");
  c.push(2000, "b");
  c.push(3000, "c");
  c.push(4000, "d");                     // 용량 3: "a" 가 빠진다
  EXPECT_EQ(c.size(), 3u);
  auto n = c.nearest(2900, 200);
  ASSERT_TRUE(n.has_value());
  EXPECT_EQ(n->second, "c");
  EXPECT_FALSE(c.nearest(1000, 500).has_value());   // 가장 가까운 2000 도 허용 차이 밖
  c.clear();
  EXPECT_FALSE(c.nearest(3000, 1000).has_value());
}
