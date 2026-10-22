// 미션표 문법 · 검증 · pick 재시도 위치 시험 (robot_nodes.html 예제 그대로)
#include <gtest/gtest.h>

#include "center/mission_table.hpp"

using namespace center;

static TableContext ctx()
{
  TableContext c;
  c.locations = {"home", "station_A"};
  return c;
}

TEST(MissionTable, ParsesDocumentExamples)
{
  std::vector<Step> s;
  std::string err;
  ASSERT_TRUE(parse_steps({"navigate station_A", "search", "approach 0.15", "skill manipulator pick_cup_v1 fallback=pick",
      "navigate home"}, ctx(), s, err)) << err;
  ASSERT_EQ(s.size(), 5u);
  EXPECT_EQ(s[2].type, StepType::APPROACH);
  EXPECT_DOUBLE_EQ(s[2].value, 0.15);
  EXPECT_EQ(s[3].type, StepType::SKILL);
  EXPECT_EQ(s[3].target, "manipulator");
  EXPECT_EQ(s[3].arg, "pick_cup_v1");
  ASSERT_TRUE(s[3].fallback);
  EXPECT_EQ(s[3].fallback->type, StepType::PICK);
  EXPECT_FALSE(s[3].has_mode);

  ASSERT_TRUE(parse_steps({"skill drive slalom_v1 40 mode=SEARCH_FRONT fallback=navigate:station_A", "pose STOW", "navigate home"},
      ctx(), s, err)) << err;
  EXPECT_DOUBLE_EQ(s[0].value, 40.0);
  EXPECT_TRUE(s[0].has_mode);
  EXPECT_EQ(s[0].mode, mode_bits::SEARCH_FRONT);
  ASSERT_TRUE(s[0].fallback);
  EXPECT_EQ(s[0].fallback->type, StepType::NAVIGATE);
  EXPECT_EQ(s[0].fallback->arg, "station_A");
}

TEST(MissionTable, RejectsTyposBeforeRunning)
{
  std::vector<Step> s;
  std::string err;
  EXPECT_FALSE(parse_steps({"navigate station_B"}, ctx(), s, err));        // 모르는 장소
  EXPECT_FALSE(parse_steps({"pose HOME"}, ctx(), s, err));                 // 모르는 자세
  EXPECT_FALSE(parse_steps({"approach 0.05"}, ctx(), s, err));             // TFmini 최소 미만
  EXPECT_FALSE(parse_steps({"skill arm p1"}, ctx(), s, err));              // 대상 오류
  EXPECT_FALSE(parse_steps({"skill drive p1 mode=3"}, ctx(), s, err));     // 숫자 모드 불가
  EXPECT_FALSE(parse_steps({"skill drive p1 fallback=skill:drive:p2"}, ctx(), s, err));
  EXPECT_FALSE(parse_steps({"jump"}, ctx(), s, err));
  EXPECT_FALSE(parse_steps({}, ctx(), s, err));
}

TEST(MissionTable, ModeNames)
{
  uint8_t m;
  std::string err;
  ASSERT_TRUE(parse_mode("SEARCH_FRONT+ARM_ALIGN", m, err));
  EXPECT_EQ(m, 3);
  ASSERT_TRUE(parse_mode("OFF", m, err));
  EXPECT_EQ(m, 0);
  EXPECT_FALSE(parse_mode("SEARCH", m, err));
}

TEST(MissionTable, PickRetriesFromPreviousSearch)
{
  std::vector<Step> s;
  std::string err;
  ASSERT_TRUE(parse_steps({"navigate station_A", "search", "approach 0.15", "pick", "navigate home"}, ctx(), s, err));
  EXPECT_EQ(retry_index_for_pick(s, 3), 1u);
  ASSERT_TRUE(parse_steps({"pick"}, ctx(), s, err));
  EXPECT_EQ(retry_index_for_pick(s, 0), 0u);
}
