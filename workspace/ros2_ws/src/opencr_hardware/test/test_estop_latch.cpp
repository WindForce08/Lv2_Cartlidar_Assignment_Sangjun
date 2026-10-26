// 잠금식 E-stop 판단 시험
#include <gtest/gtest.h>

#include "opencr_hardware/estop_latch.hpp"

using opencr_hardware::EstopInputs;
using opencr_hardware::EstopLatch;
using opencr_hardware::RearmPulse;

static EstopInputs ok()
{
  EstopInputs in;
  in.joy_fresh = true;
  in.link_ok = true;
  return in;
}

TEST(EstopLatch, StartsLatchedNeedsReset)
{
  EstopLatch l;
  EXPECT_TRUE(l.update(ok()));               // 시작은 잠김
  auto in = ok();
  in.reset_button = true;
  EXPECT_FALSE(l.update(in));                // 리셋 버튼 + 정상 → 풀림
}

TEST(EstopLatch, EachCauseLatchesAndResetNeedsAllClear)
{
  for (int cause = 0; cause < 4; ++cause) {
    EstopLatch l(false);
    auto in = ok();
    if (cause == 0) {in.joy_fresh = false;}
    if (cause == 1) {in.estop_button = true;}
    if (cause == 2) {in.physical_pressed = true;}
    if (cause == 3) {in.link_ok = false;}
    EXPECT_TRUE(l.update(in)) << cause;
    in.reset_button = true;
    EXPECT_TRUE(l.update(in)) << "원인이 남아 있으면 리셋 불가 " << cause;
    EXPECT_FALSE(l.blocking().empty());
    auto clear = ok();
    EXPECT_TRUE(l.update(clear)) << "원인이 사라져도 리셋 버튼 전까지 잠김 " << cause;
    clear.reset_button = true;
    EXPECT_FALSE(l.update(clear)) << cause;
  }
}

TEST(EstopLatch, ReasonKeepsFirstCause)
{
  EstopLatch l(false);
  auto in = ok();
  in.physical_pressed = true;
  l.update(in);
  in.joy_fresh = false;
  l.update(in);
  EXPECT_EQ(l.reason(), "물리 E-stop 스위치");
}

TEST(EstopLatch, JoyNotRequiredAutoReleases)
{
  EstopLatch l;
  EstopInputs in;
  in.joy_required = false;
  in.link_ok = true;
  EXPECT_FALSE(l.update(in));                // 조이스틱 없는 벤치: 원인이 없으면 풀림
  in.physical_pressed = true;
  EXPECT_TRUE(l.update(in));
}

TEST(EstopLatch, TripLatchesButDoesNotBlockReset)
{
  EstopLatch l(false);
  auto in = ok();
  in.trip = true;
  EXPECT_TRUE(l.update(in));
  EXPECT_EQ(l.reason(), "OpenCR 팔 HOLD");
  EXPECT_TRUE(l.blocking().empty()) << "trip 은 원인으로 남지 않음";
  in.trip = false;
  EXPECT_TRUE(l.update(in)) << "리셋 전까지 잠김";
  in.reset_button = true;
  EXPECT_FALSE(l.update(in));
}

TEST(EstopLatch, TripHoldsForOneCycleWithoutJoy)
{
  EstopLatch l(false);
  EstopInputs in;
  in.joy_required = false;
  in.link_ok = true;
  in.trip = true;
  EXPECT_TRUE(l.update(in)) << "같은 주기에는 풀지 않음 (/e_stop 이 한 번은 잠김으로 나감)";
  in.trip = false;
  EXPECT_FALSE(l.update(in));
}

TEST(RearmRequest, ResetPressOrReleaseByReset)
{
  using opencr_hardware::rearm_request;
  // (was_latched, now_latched, reset, prev_reset, auto_rearm)
  EXPECT_TRUE(rearm_request(true, false, true, false, false)) << "리셋 버튼으로 잠금이 풀린 순간";
  EXPECT_TRUE(rearm_request(true, false, true, true, false)) << "버튼을 누른 채 원인이 사라져 풀린 순간 (리뷰 R4)";
  EXPECT_TRUE(rearm_request(false, false, true, false, false)) << "풀린 뒤 리셋을 새로 누름 (재시도)";
  EXPECT_FALSE(rearm_request(false, false, true, true, false)) << "계속 누르고 있는 것만으로는 아님";
  EXPECT_FALSE(rearm_request(true, true, true, false, false)) << "아직 잠김";
  EXPECT_FALSE(rearm_request(true, false, false, false, false)) << "joy 없는 자동 해제는 REARM 아님";
  EXPECT_TRUE(rearm_request(true, false, false, false, true)) << "auto_rearm (벤치 전용)";
  EXPECT_FALSE(rearm_request(false, false, false, false, true)) << "auto_rearm 도 풀리는 순간만";
}

TEST(RearmPulse, NeedsRequestAndArmHold)
{
  RearmPulse p(3);
  EXPECT_FALSE(p.update(false, false, true, false)) << "요청 없이는 보내지 않음";
  EXPECT_FALSE(p.update(false, true, false, false)) << "팔이 HOLD 가 아니면 대기";
  int ones = 0;
  for (int i = 0; i < 10; ++i) {ones += p.update(false, false, true, false);}
  EXPECT_EQ(ones, 3) << "HOLD 가 보이면 width 주기만 1";
  ones = 0;
  for (int i = 0; i < 10; ++i) {ones += p.update(false, false, true, false);}
  EXPECT_EQ(ones, 0) << "요청 한 번 = 펄스 한 번";
}

TEST(RearmPulse, LatchOrFaultDropsRequest)
{
  RearmPulse p(3);
  p.update(false, true, false, false);
  EXPECT_FALSE(p.update(true, false, true, false)) << "잠기면 대기 요청을 버림";
  EXPECT_FALSE(p.update(false, false, true, false));
  p.update(false, true, false, false);
  EXPECT_FALSE(p.update(false, false, true, true)) << "FAULT 이면 버림";
  EXPECT_FALSE(p.update(false, false, true, false));
  EXPECT_TRUE(p.update(false, true, true, false));
  EXPECT_FALSE(p.update(true, false, true, false)) << "펄스 중 잠기면 바로 멈춤";
  EXPECT_FALSE(p.update(false, false, true, false));
}
