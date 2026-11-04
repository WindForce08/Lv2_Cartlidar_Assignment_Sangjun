// SagComp 시험: 시간표 시간 (manipulator_node sendWaypoints 와 같은 규칙) · 보간, 처짐 보정 수렴 · 한도
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "control/sag_comp.hpp"

using control::Arm5;
using control::JointTimeline;
using control::SagIntegrator;

TEST(SagComp, TimelineTimingMatchesSendWaypoints)
{
  JointTimeline tl(Arm5{0, 0, 0, 0, 0}, -0.7);
  tl.hold(1.5, -0.7);
  // 첫 점 max(min_duration 1.0, 0.4/0.8 = 0.5) = 1.0, 둘째 0.8/0.8 = 1.0, 셋째 max(0.02, 0.001/0.8) = 0.02
  tl.moveThrough({Arm5{0.4, 0, 0, 0, 0}, Arm5{0.4, 0.8, 0, 0, 0}, Arm5{0.4, 0.8, 0.001, 0, 0}}, 0.8, 0.9, 1.0);
  tl.hold(0.3, 0.9);
  tl.hold(1.0, 0.08);
  EXPECT_NEAR(tl.duration(), 1.5 + 1.0 + 1.0 + 0.02 + 0.3 + 1.0, 1e-12);
  // 보간: settle 중에는 시작 자세, 첫 구간 절반에서 joint1 0.2 · 그리퍼 반쯤
  EXPECT_NEAR(tl.at(1.0).q[0], 0.0, 1e-12);
  const auto mid = tl.at(2.0);
  EXPECT_NEAR(mid.q[0], 0.2, 1e-12);
  EXPECT_NEAR(mid.grip, (-0.7 + 0.9) / 2, 1e-12);
  // 그리퍼 바꾸는 구간 끝 = 0.08, 그 뒤는 마지막 값 유지
  EXPECT_NEAR(tl.at(tl.duration()).grip, 0.08, 1e-12);
  EXPECT_NEAR(tl.at(tl.duration() + 5.0).q[1], 0.8, 1e-12);
}

TEST(SagComp, IntegratorRemovesSteadySag)
{
  // 처지는 팔 모형: 측정 = 명령 − 처짐 (위치 P 만, 정상 상태). Isaac 과 같은 ki 4, settle 1.5 s 면 처짐의 99 % 이상 메움
  const Arm5 plan{0.1, -1.0, 0.7, 1.4, 0.0};
  const Arm5 sag{0.0, -0.05, 0.03, -0.02, 0.0};
  SagIntegrator s(4.0, 0.15);
  const double dt = 0.02;
  for (int k = 0; k < static_cast<int>(1.5 / dt); ++k) {
    const Arm5 cmd = s.command(plan);
    Arm5 meas{};
    for (size_t j = 0; j < 5; ++j) {meas[j] = cmd[j] - sag[j];}
    s.update(plan, meas, dt);
  }
  for (size_t j = 0; j < 5; ++j) {
    EXPECT_NEAR(s.bias()[j], sag[j], 0.01 * 0.05) << "joint" << j + 1;
  }
}

TEST(SagComp, BiasClamped)
{
  SagIntegrator s(4.0, 0.15);
  for (int k = 0; k < 1000; ++k) {s.update(Arm5{1, 1, 1, 1, 1}, Arm5{0, 0, 0, 0, 0}, 0.02);}
  for (double b : s.bias()) {EXPECT_NEAR(b, 0.15, 1e-12);}
  SagIntegrator off(0.0, 0.15);
  off.update(Arm5{1, 1, 1, 1, 1}, Arm5{0, 0, 0, 0, 0}, 0.02);
  for (double b : off.bias()) {EXPECT_EQ(b, 0.0);}
}
