// manipulator.yaml 관절 한계 · 자세 · 팔레트 궤적이 URDF (= 펌웨어 config.h) 한계와 맞는지 (#83)
//   펌웨어는 범위 밖 CMD 를 통째로 거부하므로 세 곳이 어긋나면 팔이 움직이지 않는다
#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// robot.urdf.xacro 의 <xacro:arm_joint name="joint1" ... lower="-3.14159" upper="3.14159"/>
std::map<std::string, std::pair<double, double>> urdfLimits()
{
  const std::string text = readFile(URDF_XACRO);
  std::map<std::string, std::pair<double, double>> out;
  // 패턴 안에 )" 가 있어 원시 문자열 구분자 re 를 붙임
  const std::regex re(R"re(<xacro:arm_joint\s+name="(\w+)"[^>]*lower="([-0-9.]+)"\s+upper="([-0-9.]+)")re");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), re); it != std::sregex_iterator(); ++it) {
    out[(*it)[1]] = {std::stod((*it)[2]), std::stod((*it)[3])};
  }
  return out;
}

YAML::Node params()
{
  return YAML::LoadFile(MANIPULATOR_YAML)["manipulator_node"]["ros__parameters"];
}
}  // namespace

TEST(ConfigLimits, YamlLimitsEqualUrdf)
{
  const auto urdf = urdfLimits();
  const auto p = params();
  const auto names = p["arm"]["joint_names"].as<std::vector<std::string>>();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  ASSERT_EQ(names.size(), 6u);
  for (size_t i = 0; i < names.size(); ++i) {
    ASSERT_TRUE(urdf.count(names[i])) << names[i] << " not in " << URDF_XACRO;
    EXPECT_NEAR(lo[i], urdf.at(names[i]).first, 1e-4) << names[i];
    EXPECT_NEAR(hi[i], urdf.at(names[i]).second, 1e-4) << names[i];
  }
}

TEST(ConfigLimits, PosesWithinLimits)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  for (const auto & name : p["poses"]["names"].as<std::vector<std::string>>()) {
    const auto q = p["poses"][name].as<std::vector<double>>();
    ASSERT_EQ(q.size(), 6u) << name;
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_GE(q[i], lo[i]) << name << "[" << i << "]";
      EXPECT_LE(q[i], hi[i]) << name << "[" << i << "]";
    }
  }
}

TEST(ConfigLimits, GripperPositionsWithinSixthJoint)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  if (p["gripper"]["mode"].as<std::string>() != "arm_joint") {GTEST_SKIP();}
  for (const char * k : {"open_position", "close_position"}) {
    const double v = p["gripper"][k].as<double>();
    EXPECT_GE(v, lo[5]) << k;
    EXPECT_LE(v, hi[5]) << k;
  }
}

TEST(ConfigLimits, PalletWaypointsWithinLimits)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  const YAML::Node doc = YAML::LoadFile(PALLET_JSON);
  ASSERT_EQ(doc["slots"].size(), 12u);
  for (const auto & slot : doc["slots"]) {
    for (const auto & seg : slot["segments"]) {
      for (const auto & q : seg["q"].as<std::vector<std::vector<double>>>()) {
        ASSERT_EQ(q.size(), 5u);
        for (size_t i = 0; i < 5; ++i) {
          EXPECT_GE(q[i], lo[i]) << "row " << slot["row"].as<int>() << " slot " << slot["index"].as<int>();
          EXPECT_LE(q[i], hi[i]) << "row " << slot["row"].as<int>() << " slot " << slot["index"].as<int>();
        }
      }
    }
  }
}
