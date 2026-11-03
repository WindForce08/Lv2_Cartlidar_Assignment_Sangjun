// 입력 영상 시각 규칙 시험 (StampGate · NearestCache · pick_tf_frame)
#include <gtest/gtest.h>

#include <string>
#include <vector>

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

// 30 Hz 영상 3 장(0 · 33 · 66 ms), 지금 70 ms, 기다림 50 ms
TEST(PickTfFrame, NewestWithBodyTf)
{
  const std::vector<int64_t> ns{0, 33'000'000, 66'000'000};
  const int64_t now = 70'000'000, wait = 50'000'000;
  EXPECT_EQ(pick_tf_frame(ns, -1, now, 66'000'000, wait), std::optional<size_t>(2));   // 가장 새 장의 TF 가 옴
  EXPECT_EQ(pick_tf_frame(ns, -1, now, 50'000'000, wait), std::optional<size_t>(1));   // EKF 가 16 ms 늦음 → 한 장 앞
  EXPECT_EQ(pick_tf_frame(ns, 33'000'000, now, 50'000'000, wait), std::optional<size_t>(1));   // 같은 장(새 결과 아님)
  EXPECT_FALSE(pick_tf_frame(ns, 66'000'000, now, 50'000'000, wait).has_value());   // 이미 쓴 장보다 앞은 고르지 않음 → 기다림
  EXPECT_FALSE(pick_tf_frame({33'000'000}, -1, 35'000'000, 20'000'000, wait).has_value());   // 첫 장뿐, TF 가 아직 그 장 앞
}

// 기다리지 않고 가장 새 장: TF 없음 · TF 멈춤 · 가장 새 장이 wait 넘게 지남 · 기다림 끔
TEST(PickTfFrame, FallsBackToNewest)
{
  const std::vector<int64_t> ns{0, 33'000'000, 66'000'000};
  const int64_t wait = 50'000'000;
  EXPECT_EQ(pick_tf_frame(ns, -1, 70'000'000, -1, wait), std::optional<size_t>(2));
  EXPECT_EQ(pick_tf_frame(ns, -1, 70'000'000, 10'000'000, wait), std::optional<size_t>(2));   // 56 ms 뒤짐 = 멈춤
  EXPECT_EQ(pick_tf_frame(ns, -1, 120'000'000, 50'000'000, wait), std::optional<size_t>(2));
  EXPECT_EQ(pick_tf_frame(ns, -1, 70'000'000, 50'000'000, 0), std::optional<size_t>(2));
  EXPECT_FALSE(pick_tf_frame({}, -1, 0, 0, wait).has_value());
}
