// 미션표 (mission_table.yaml → mission_node ROS 파라미터) 단계 문법 해석 · 검증 — ROS 의존 없음
//   navigate <장소>                                  주행 + 팔 STOW (병렬)
//   search                                           SEARCH_FRONT | SEARCH_REAR, 못 찾으면 회전
//   approach <거리 m>                                 STOW 로 먼 거리 → READY → 정밀 <거리>
//   pick                                             /manipulator/pick
//   pose <STOW|READY|PLACE|TAG_VIEW>
//   skill <drive|manipulator> <정책> [timeout_s] [mode=이름+이름] [fallback=단계]
//        fallback 단계 안의 공백은 ':' 로 (예: fallback=navigate:station_A)
//   mode 이름: OFF · SEARCH_FRONT · ARM_ALIGN · READ_TAG · SEARCH_REAR (숫자 불가)
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace center
{

enum class StepType { NAVIGATE, SEARCH, APPROACH, PICK, POSE, SKILL };

struct Step
{
  StepType type = StepType::SEARCH;
  std::string text;            // 원문 (상태 표시용)
  std::string arg;             // 장소 · 자세 이름 · 정책 이름
  std::string target;          // skill: drive | manipulator
  double value = 0.0;          // approach 거리 · skill timeout
  bool has_mode = false;
  uint8_t mode = 0;            // skill 의 perception 모드
  std::shared_ptr<Step> fallback;
};

namespace mode_bits
{
constexpr uint8_t OFF = 0, SEARCH_FRONT = 1, ARM_ALIGN = 2, READ_TAG = 4, SEARCH_REAR = 8;
}

// "SEARCH_FRONT+ARM_ALIGN" → 3. 숫자 · 모르는 이름이면 false
inline bool parse_mode(const std::string & s, uint8_t & out, std::string & err)
{
  static const std::map<std::string, uint8_t> names = {{"OFF", 0}, {"SEARCH_FRONT", 1}, {"ARM_ALIGN", 2},
    {"READ_TAG", 4}, {"SEARCH_REAR", 8}};
  out = 0;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, '+')) {
    auto it = names.find(tok);
    if (it == names.end()) {err = "모드 이름 '" + tok + "' 모름 (숫자 불가)"; return false;}
    out |= it->second;
  }
  if (s.empty()) {err = "mode= 값이 비어 있음"; return false;}
  return true;
}

struct TableContext
{
  std::set<std::string> locations;
  std::set<std::string> poses{"STOW", "READY", "PLACE", "TAG_VIEW"};
};

inline bool parse_step(const std::string & text, const TableContext & ctx, Step & st, std::string & err)
{
  std::stringstream ss(text);
  std::vector<std::string> t;
  for (std::string w; ss >> w;) {t.push_back(w);}
  st = Step{};
  st.text = text;
  if (t.empty()) {err = "빈 단계"; return false;}
  const std::string & k = t[0];
  auto need = [&](size_t n) {
      if (t.size() != n) {err = "'" + text + "': 인자 수 오류"; return false;}
      return true;
    };
  if (k == "navigate") {
    if (!need(2)) {return false;}
    if (!ctx.locations.count(t[1])) {err = "'" + text + "': 장소 '" + t[1] + "' 가 locations 에 없음"; return false;}
    st.type = StepType::NAVIGATE;
    st.arg = t[1];
  } else if (k == "search") {
    if (!need(1)) {return false;}
    st.type = StepType::SEARCH;
  } else if (k == "approach") {
    if (!need(2)) {return false;}
    st.type = StepType::APPROACH;
    try {st.value = std::stod(t[1]);} catch (...) {err = "'" + text + "': 거리 숫자 아님"; return false;}
    if (st.value < 0.10) {err = "'" + text + "': 정밀 거리는 0.10 m 이상 (TFmini 최소)"; return false;}
  } else if (k == "pick") {
    if (!need(1)) {return false;}
    st.type = StepType::PICK;
  } else if (k == "pose") {
    if (!need(2)) {return false;}
    if (!ctx.poses.count(t[1])) {err = "'" + text + "': 자세 '" + t[1] + "' 모름"; return false;}
    st.type = StepType::POSE;
    st.arg = t[1];
  } else if (k == "skill") {
    if (t.size() < 3) {err = "'" + text + "': skill <drive|manipulator> <정책>"; return false;}
    st.type = StepType::SKILL;
    st.target = t[1];
    st.arg = t[2];
    if (st.target != "drive" && st.target != "manipulator") {err = "'" + text + "': 대상은 drive 또는 manipulator"; return false;}
    st.value = 30.0;
    for (size_t i = 3; i < t.size(); ++i) {
      const std::string & w = t[i];
      if (w.rfind("mode=", 0) == 0) {
        if (!parse_mode(w.substr(5), st.mode, err)) {err = "'" + text + "': " + err; return false;}
        st.has_mode = true;
      } else if (w.rfind("fallback=", 0) == 0) {
        std::string fb = w.substr(9);
        for (char & c : fb) {if (c == ':') {c = ' ';}}
        auto f = std::make_shared<Step>();
        if (!parse_step(fb, ctx, *f, err)) {err = "'" + text + "' fallback: " + err; return false;}
        if (f->type == StepType::SKILL) {err = "'" + text + "': fallback 은 skill 이 아니어야 함"; return false;}
        st.fallback = f;
      } else {
        try {st.value = std::stod(w);} catch (...) {err = "'" + text + "': '" + w + "' 모름"; return false;}
        if (st.value <= 0) {err = "'" + text + "': timeout 은 양수"; return false;}
      }
    }
  } else {
    err = "'" + text + "': 단계 종류 '" + k + "' 모름";
    return false;
  }
  return true;
}

inline bool parse_steps(const std::vector<std::string> & texts, const TableContext & ctx, std::vector<Step> & out,
  std::string & err)
{
  out.clear();
  for (const auto & s : texts) {
    Step st;
    if (!parse_step(s, ctx, st, err)) {return false;}
    out.push_back(st);
  }
  if (out.empty()) {err = "단계가 없음"; return false;}
  return true;
}

// pick 실패 · grasped=false → 직전 search 부터 다시. search 가 없으면 pick 자신
inline size_t retry_index_for_pick(const std::vector<Step> & steps, size_t pick_index)
{
  for (size_t i = pick_index + 1; i-- > 0;) {
    if (steps[i].type == StepType::SEARCH) {return i;}
  }
  return pick_index;
}

}  // namespace center
