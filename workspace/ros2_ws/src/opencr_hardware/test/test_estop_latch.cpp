// 잠금식 E-stop 판단 시험
#include <gtest/gtest.h>

#include "opencr_hardware/estop_latch.hpp"

using opencr_hardware::EstopInputs;
using opencr_hardware::EstopLatch;

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
