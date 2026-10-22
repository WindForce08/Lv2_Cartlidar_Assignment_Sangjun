#include "robot_policy/policy.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <regex>
#include <sstream>

namespace robot_policy
{

bool parse_action(const std::string & s, std::string & kind, int & dim)
{
  static const std::regex re(R"(^\s*([A-Za-z_]+)\s*\(\s*(\d+)\s*\)\s*$)");
  std::smatch m;
  if (!std::regex_match(s, m, re)) {return false;}
  kind = m[1];
  dim = std::stoi(m[2]);
  return (kind == "joint_target" || kind == "base_twist") && dim > 0;
}

int layout_dim(const PolicySpec & s)
{
  int n = 0;
  for (const auto & o : s.observation) {n += o.size + (o.required ? 0 : 1);}
  return n;
}

static std::vector<float> floats(const YAML::Node & n)
{
  std::vector<float> v;
  if (n && n.IsSequence()) {for (const auto & x : n) {v.push_back(x.as<float>());}}
  return v;
}

bool load_specs(const std::string & yaml_path, std::map<std::string, PolicySpec> & out, std::string & err)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    err = yaml_path + ": " + e.what();
    return false;
  }
  const std::string dir = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
  for (const auto & kv : root) {
    PolicySpec s;
    s.name = kv.first.as<std::string>();
    const YAML::Node & n = kv.second;
    try {
      s.model = n["model"].as<std::string>();
      if (!s.model.empty() && s.model[0] != '/') {s.model = dir + s.model;}
      s.rate_hz = n["rate_hz"].as<double>();
      s.obs_dim = n["obs_dim"].as<int>();
      for (const auto & o : n["observation"]) {
        ObsItem it;
        it.name = o["name"].as<std::string>();
        it.size = o["size"].as<int>();
        it.required = o["required"] ? o["required"].as<bool>() : true;
        it.fill = floats(o["fill"]);
        it.offset = floats(o["offset"]);
        it.scale = floats(o["scale"]);
        if (!it.required && static_cast<int>(it.fill.size()) != it.size) {
          err = s.name + "/" + it.name + ": 선택 항목은 fill 길이가 size 와 같아야 함";
          return false;
        }
        if ((!it.offset.empty() && static_cast<int>(it.offset.size()) != it.size) ||
          (!it.scale.empty() && static_cast<int>(it.scale.size()) != it.size))
        {
          err = s.name + "/" + it.name + ": offset · scale 길이 오류";
          return false;
        }
        s.observation.push_back(it);
      }
      if (!parse_action(n["action"].as<std::string>(), s.action_kind, s.action_dim)) {
        err = s.name + ": action 형식은 joint_target(N) 또는 base_twist(2)";
        return false;
      }
      s.finish_on_timeout = n["finish_on_timeout"] ? n["finish_on_timeout"].as<bool>() : false;
      s.action_low = floats(n["action_low"]);
      s.action_high = floats(n["action_high"]);
    } catch (const std::exception & e) {
      err = s.name + ": " + e.what();
      return false;
    }
    if (layout_dim(s) != s.obs_dim) {
      std::ostringstream os;
      os << s.name << ": obs_dim " << s.obs_dim << " ≠ observation 배치 " << layout_dim(s);
      err = os.str();
      return false;
    }
    out[s.name] = s;
  }
  return true;
}

bool ObservationBuilder::build(const ObsValues & v, std::vector<float> & out, std::string & err) const
{
  out.clear();
  out.reserve(spec_.obs_dim);
  for (const auto & o : spec_.observation) {
    auto it = v.find(o.name);
    const bool valid = it != v.end() && it->second && static_cast<int>(it->second->size()) == o.size &&
      std::all_of(it->second->begin(), it->second->end(), [](float x) {return std::isfinite(x);});
    if (o.required && !valid) {err = o.name; return false;}
    const std::vector<float> & src = valid ? *it->second : o.fill;
    for (int i = 0; i < o.size; ++i) {
      float x = src[i];
      if (valid) {   // fill 은 학습 때 넣은 값 그대로 (정규화 안 함)
        if (!o.offset.empty()) {x -= o.offset[i];}
        if (!o.scale.empty()) {x *= o.scale[i];}
      }
      out.push_back(x);
    }
    if (!o.required) {out.push_back(valid ? 1.f : 0.f);}
  }
  return true;
}

std::vector<float> ActionMapper::map(const std::vector<float> & raw) const
{
  std::vector<float> a(spec_.action_dim, 0.f);
  for (int i = 0; i < spec_.action_dim && i < static_cast<int>(raw.size()); ++i) {
    float x = std::isfinite(raw[i]) ? raw[i] : 0.f;
    if (i < static_cast<int>(spec_.action_low.size())) {x = std::max(x, spec_.action_low[i]);}
    if (i < static_cast<int>(spec_.action_high.size())) {x = std::min(x, spec_.action_high[i]);}
    a[i] = x;
  }
  return a;
}

void PolicyLibrary::load(const std::string & yaml_path, std::vector<std::string> & errors)
{
  std::map<std::string, PolicySpec> specs;
  std::string err;
  if (!load_specs(yaml_path, specs, err)) {errors.push_back(err); return;}
  for (auto & kv : specs) {
    std::string e;
    auto r = make_onnx_runner(kv.second.model, e);
    if (!r) {errors.push_back(kv.first + ": " + e); continue;}
    if (r->input_dim() != kv.second.obs_dim || r->output_dim() != kv.second.action_dim) {
      std::ostringstream os;
      os << kv.first << ": ONNX 입출력 " << r->input_dim() << "/" << r->output_dim() << " ≠ obs_dim " <<
        kv.second.obs_dim << " / action " << kv.second.action_dim;
      errors.push_back(os.str());
      continue;
    }
    policies_[kv.first] = LoadedPolicy{kv.second, std::move(r)};
  }
}

LoadedPolicy * PolicyLibrary::find(const std::string & name)
{
  auto it = policies_.find(name);
  return it == policies_.end() ? nullptr : &it->second;
}

std::vector<std::string> PolicyLibrary::names() const
{
  std::vector<std::string> n;
  for (const auto & kv : policies_) {n.push_back(kv.first);}
  return n;
}

}  // namespace robot_policy
