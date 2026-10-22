// robot_policy — 강화학습 정책 CPU 추론 라이브러리 (노드 아님, drive · manipulator 가 링크)
//   PolicySpec          policies.yaml 한 항목 (model · rate_hz · obs_dim · observation · action)
//   ObservationBuilder  observation 순서대로 float 벡터. 필수 = [값...], 선택 = [값..., valid] (무효면 fill + 0)
//                       시작 시 obs_dim 과 배치 길이 비교 → 다르면 로드 실패
//   ActionMapper        출력 → 관절 목표(rad) 또는 속도(m/s, rad/s), 범위 clamp
//   PolicyRunner        .onnx 로드 · run(obs) (ONNX Runtime CPU, USE_ONNXRUNTIME 일 때만)
//   PolicyLibrary       노드 시작 시 모든 정책 로드 — 실행 중 로드 없음, GPU 안 씀
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robot_policy
{

struct ObsItem
{
  std::string name;
  int size = 1;
  bool required = true;
  std::vector<float> fill;     // 선택 항목이 무효일 때 (학습 때 안 보일 때 넣은 값과 같게)
  std::vector<float> offset;   // 정규화: (x - offset) * scale  (없으면 그대로)
  std::vector<float> scale;
};

struct PolicySpec
{
  std::string name;
  std::string model;           // .onnx 경로 (yaml 파일 기준 상대 경로 허용)
  double rate_hz = 20.0;
  int obs_dim = 0;
  std::vector<ObsItem> observation;
  std::string action_kind;     // joint_target · base_twist
  int action_dim = 0;
  std::vector<float> action_low, action_high;   // clamp (없으면 kind 기본값 → 노드 한계로 다시 clamp)
  bool finish_on_timeout = false;   // true: goal.timeout 까지 돌면 성공 (예: 정해진 시간 동안 하는 주행)
                                    // false: timeout = 실패 (0 속도 후 abort, mission 이 fallback)
};

// yaml 파일 → 정책 목록. 형식 오류 · obs_dim 불일치면 false (err 에 이유)
bool load_specs(const std::string & yaml_path, std::map<std::string, PolicySpec> & out, std::string & err);
// "joint_target(6)" → kind, dim
bool parse_action(const std::string & s, std::string & kind, int & dim);
// observation 배치 길이 (선택 항목은 +1)
int layout_dim(const PolicySpec & s);

using ObsValues = std::map<std::string, std::optional<std::vector<float>>>;   // nullopt = 무효

class ObservationBuilder
{
public:
  explicit ObservationBuilder(const PolicySpec & s)
  : spec_(s) {}
  // 필수 관측이 무효면 false (err = 항목 이름) → 노드는 0 속도 · 정지 후 abort
  bool build(const ObsValues & v, std::vector<float> & out, std::string & err) const;

private:
  PolicySpec spec_;
};

class ActionMapper
{
public:
  explicit ActionMapper(const PolicySpec & s)
  : spec_(s) {}
  std::vector<float> map(const std::vector<float> & raw) const;

private:
  PolicySpec spec_;
};

class PolicyRunner
{
public:
  virtual ~PolicyRunner() = default;
  virtual bool run(const std::vector<float> & obs, std::vector<float> & action) = 0;
  virtual int input_dim() const = 0;
  virtual int output_dim() const = 0;
};

// ONNX 모델 로드. ONNX Runtime 없이 빌드됐으면 nullptr + err
std::unique_ptr<PolicyRunner> make_onnx_runner(const std::string & model_path, std::string & err);

struct LoadedPolicy
{
  PolicySpec spec;
  std::unique_ptr<PolicyRunner> runner;
};

class PolicyLibrary
{
public:
  // yaml 의 모든 정책을 로드. 실패한 정책은 errors 에 남기고 건너뜀
  void load(const std::string & yaml_path, std::vector<std::string> & errors);
  LoadedPolicy * find(const std::string & name);
  std::vector<std::string> names() const;
  static bool onnx_available();

private:
  std::map<std::string, LoadedPolicy> policies_;
};

}  // namespace robot_policy
