// 상태머신 · 안전 래치 시험 (v1.2 전이표)
#include <gtest/gtest.h>

#include "control/fsm.hpp"

using namespace control;

TEST(MissionFsm, PauseResumeAndRejects)
{
  using S = MissionState;
  auto f = make_mission_fsm();
  int rejected = 0;
  f.on_reject([&](S, S, const std::string &) {++rejected;});
  EXPECT_FALSE(f.transition(S::PAUSED));            // IDLE 에서 PAUSED 불가
  EXPECT_EQ(rejected, 1);
  ASSERT_TRUE(f.transition(S::RUNNING));
  ASSERT_TRUE(f.transition(S::PAUSED));             // E-stop
  EXPECT_FALSE(f.transition(S::DONE));              // PAUSED 에서 바로 완료 불가
  ASSERT_TRUE(f.transition(S::RUNNING));            // 시작 버튼 → 현재 단계부터 재개
  ASSERT_TRUE(f.transition(S::FAILED));
  ASSERT_TRUE(f.transition(S::RUNNING));            // 실패 뒤 새로 시작
  ASSERT_TRUE(f.transition(S::DONE));
  ASSERT_TRUE(f.transition(S::IDLE));
}

TEST(SkillFsm, PathsFromDesign)
{
  auto f = make_skill_fsm();
  EXPECT_FALSE(f.transition(SkillState::RUNNING));            // PRECHECK 없이 실행 불가
  ASSERT_TRUE(f.transition(SkillState::PRECHECK));
  ASSERT_TRUE(f.transition(SkillState::RUNNING));
  EXPECT_FALSE(f.transition(SkillState::DONE));               // VERIFY 없이 DONE 불가
  ASSERT_TRUE(f.transition(SkillState::HOLD));
  EXPECT_FALSE(f.transition(SkillState::RUNNING));            // HOLD 에서 재개 불가
  ASSERT_TRUE(f.transition(SkillState::FAILED));
  ASSERT_TRUE(f.transition(SkillState::IDLE));
  ASSERT_TRUE(f.transition(SkillState::PRECHECK));
  ASSERT_TRUE(f.transition(SkillState::REJECTED));
  ASSERT_TRUE(f.transition(SkillState::IDLE));
}

TEST(SafetyLatch, EstopOnlyByResetHoldByNewAction)
{
  SafetyLatch l;
  EXPECT_TRUE(l.hold("관절 한계"));
  EXPECT_FALSE(l.release_hold(false));                         // 자세가 아직 범위 밖
  EXPECT_TRUE(l.release_hold(true));
  EXPECT_EQ(l.state(), SafetyState::NORMAL);

  EXPECT_TRUE(l.estop("/e_stop"));
  EXPECT_FALSE(l.hold("x"));                                   // ESTOP 이 HOLD 로 내려가지 않음
  EXPECT_FALSE(l.release_hold(true));                          // 새 action 으로 풀리지 않음
  std::string why;
  EXPECT_FALSE(l.reset_estop(true, true, why));                // /e_stop 이 아직 true
  EXPECT_FALSE(l.reset_estop(false, false, why));              // 관절 상태 없음
  EXPECT_TRUE(l.reset_estop(false, true, why));
  EXPECT_EQ(l.state(), SafetyState::NORMAL);
  EXPECT_FALSE(l.reset_estop(false, true, why));               // 이미 NORMAL
}
