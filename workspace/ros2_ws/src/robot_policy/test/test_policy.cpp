// policies.yaml 파싱 · 관측 배치 · fill · 행동 clamp 시험 (학습 쪽 예제 벡터와 같은 규칙)
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include "robot_policy/policy.hpp"

using namespace robot_policy;

static std::string write_tmp(const std::string & name, const std::string & text)
{
  const std::string p = std::string(::testing::TempDir()) + name;
  std::ofstream(p) << text;
  return p;
}

TEST(Policy, ParsesShippedYamlAndChecksObsDim)
{
  std::map<std::string, PolicySpec> specs;
  std::string err;
  ASSERT_TRUE(load_specs(POLICIES_YAML, specs, err)) << err;
  ASSERT_EQ(specs.count("pick_cup_v1"), 1u);
  const auto & p = specs["pick_cup_v1"];
  EXPECT_EQ(p.obs_dim, 12);
  EXPECT_EQ(layout_dim(p), 12);
  EXPECT_EQ(p.action_kind, "joint_target");
  EXPECT_EQ(p.action_dim, 6);
  EXPECT_EQ(specs["slalom_v1"].obs_dim, 367);
  EXPECT_NE(p.model.find("models/pick_cup_v1.onnx"), std::string::npos);
}

TEST(Policy, RejectsObsDimMismatch)
{
  const auto path = write_tmp("bad.yaml", R"(
bad:
  model: m.onnx
  rate_hz: 20
  obs_dim: 11
  observation:
    - {name: joint_pos, size: 6, required: true}
    - {name: arm_offset, size: 3, required: false, fill: [0, 0, 0]}
    - {name: arm_range, size: 1, required: false, fill: [12.0]}
  action: joint_target(6)
)");
  std::map<std::string, PolicySpec> specs;
  std::string err;
  EXPECT_FALSE(load_specs(path, specs, err));
  EXPECT_NE(err.find("obs_dim"), std::string::npos);
}

TEST(Policy, LayoutValidFlagsAndFill)
{
  std::map<std::string, PolicySpec> specs;
  std::string err;
  ASSERT_TRUE(load_specs(POLICIES_YAML, specs, err));
  ObservationBuilder b(specs["pick_cup_v1"]);
  ObsValues v;
  v["joint_pos"] = std::vector<float>{1, 2, 3, 4, 5, 6};
  v["arm_offset"] = std::vector<float>{0.1f, -0.2f, 0.05f};
  v["arm_range"] = std::nullopt;   // TFmini 무효 → fill 12.0, valid 0
  std::vector<float> o;
  ASSERT_TRUE(b.build(v, o, err));
  const std::vector<float> expect{1, 2, 3, 4, 5, 6, 0.1f, -0.2f, 0.05f, 1, 12.0f, 0};
  ASSERT_EQ(o.size(), expect.size());
  for (size_t i = 0; i < o.size(); ++i) {EXPECT_FLOAT_EQ(o[i], expect[i]) << i;}

  v["joint_pos"] = std::nullopt;   // 필수 무효 → 실패 (노드: 0 속도 후 abort)
  EXPECT_FALSE(b.build(v, o, err));
  EXPECT_EQ(err, "joint_pos");
  v["joint_pos"] = std::vector<float>{1, 2, 3, 4, 5, NAN};
  EXPECT_FALSE(b.build(v, o, err));
}

TEST(Policy, NormalizationAndActionClamp)
{
  const auto path = write_tmp("norm.yaml", R"(
p:
  model: m.onnx
  rate_hz: 10
  obs_dim: 3
  observation:
    - {name: a, size: 2, required: true, offset: [1.0, 0.0], scale: [2.0, 0.5]}
    - {name: b, size: 0, required: false, fill: []}
  action: base_twist(2)
  action_low: [-0.3, -1.0]
  action_high: [0.3, 1.0]
)");
  std::map<std::string, PolicySpec> specs;
  std::string err;
  ASSERT_TRUE(load_specs(path, specs, err)) << err;
  ObservationBuilder b(specs["p"]);
  std::vector<float> o;
  ASSERT_TRUE(b.build({{"a", std::vector<float>{2.0f, 4.0f}}}, o, err));
  EXPECT_FLOAT_EQ(o[0], 2.0f);
  EXPECT_FLOAT_EQ(o[1], 2.0f);
  EXPECT_FLOAT_EQ(o[2], 0.0f);
  ActionMapper m(specs["p"]);
  const auto a = m.map({5.0f, NAN});
  EXPECT_FLOAT_EQ(a[0], 0.3f);
  EXPECT_FLOAT_EQ(a[1], 0.0f);
}

TEST(Policy, ActionStringAndRunnerAvailability)
{
  std::string k;
  int d;
  EXPECT_TRUE(parse_action("joint_target(6)", k, d));
  EXPECT_EQ(d, 6);
  EXPECT_FALSE(parse_action("velocity(2)", k, d));
  std::string err;
  if (!PolicyLibrary::onnx_available()) {
    EXPECT_EQ(make_onnx_runner("x.onnx", err), nullptr);
    EXPECT_FALSE(err.empty());
  }
}
