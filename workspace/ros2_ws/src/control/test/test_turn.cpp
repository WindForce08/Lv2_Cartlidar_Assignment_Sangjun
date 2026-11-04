// 제자리 회전 skill (turn_skill.hpp, #77)
#include <gtest/gtest.h>

#include <cmath>

#include "control/turn_skill.hpp"

using control::TurnController;
using control::TurnGains;
using control::TurnGeometry;

TEST(Turn, ParseName)
{
  EXPECT_DOUBLE_EQ(*control::parse_turn_deg("turn_p17"), 17.0);
  EXPECT_DOUBLE_EQ(*control::parse_turn_deg("turn_m17"), -17.0);
  EXPECT_DOUBLE_EQ(*control::parse_turn_deg("turn_0"), 0.0);
  EXPECT_DOUBLE_EQ(*control::parse_turn_deg("turn_p17.06"), 17.06);
  for (const char * bad : {"turn_", "turn_p", "turn_x5", "turn_p-3", "turn_p1e2", "turn_p.5", "turn_p90", "turn_m0",
      "toppick_tilt", "turn_p17_x"})
  {
    EXPECT_FALSE(control::parse_turn_deg(bad).has_value()) << bad;
  }
}

TEST(Turn, AngleKeepsJ1LateralShift)
{
  TurnGeometry g;
  // 회전 중심 = 계획기 가정 → 이름 각 그대로
  EXPECT_NEAR(control::turn_angle_rad(17.06, g), 17.06 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(control::turn_angle_rad(-17.06, g), -17.06 * M_PI / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(control::turn_angle_rad(0.0, g), 0.0);
  // 계획기: J1 옆 40 mm (poses.turn_deg)
  const double deg = std::asin(0.040 / (g.j1_x - g.center_x)) * 180.0 / M_PI;
  EXPECT_NEAR(deg, 17.06, 0.01);
  // ICR 이 뒤 (모의 -0.0658) → 덜 돌아도 J1 옆 이동 40 mm 그대로
  g.icr_x = -0.0658;
  const double a = control::turn_angle_rad(deg, g);
  EXPECT_NEAR((g.j1_x - g.icr_x) * std::sin(a), 0.040, 1e-9);
  EXPECT_NEAR(a * 180.0 / M_PI, 14.31, 0.02);
}

TEST(Turn, SaturationAndMinimumSpeed)
{
  TurnController c(TurnGains{});
  EXPECT_DOUBLE_EQ(c.step(1.0).w, 0.6);
  EXPECT_DOUBLE_EQ(c.step(-1.0).w, -0.6);
  EXPECT_DOUBLE_EQ(c.step(0.03).w, 0.15);    // 2.0 × 0.03 = 0.06 < 최소 0.15
  EXPECT_DOUBLE_EQ(c.step(-0.03).w, -0.15);
  EXPECT_DOUBLE_EQ(c.step(0.01).w, 0.0);     // 1° 안
  EXPECT_DOUBLE_EQ(c.step(2 * M_PI + 0.2).w, 0.4);   // ±π 로 접음
}

TEST(Turn, SettlesOnlyAfterConsecutiveCycles)
{
  TurnController c(TurnGains{});
  for (int i = 0; i < 5; ++i) {EXPECT_FALSE(c.step(0.0).done);}
  EXPECT_FALSE(c.step(0.1).done);             // 허용 밖 → 다시 셈
  for (int i = 0; i < 5; ++i) {EXPECT_FALSE(c.step(0.0).done);}
  EXPECT_TRUE(c.step(0.0).done);
}

TEST(Turn, ClosedLoopConverges)
{
  TurnController c(TurnGains{});
  const double target = 14.31 * M_PI / 180.0, dt = 0.05;
  double yaw = 0.0;
  bool done = false;
  for (int i = 0; i < 200 && !done; ++i) {
    const auto cmd = c.step(target - yaw);
    done = cmd.done;
    yaw += cmd.w * dt;
  }
  EXPECT_TRUE(done);
  EXPECT_LT(std::fabs(target - yaw), 1.5 * M_PI / 180.0);
}
