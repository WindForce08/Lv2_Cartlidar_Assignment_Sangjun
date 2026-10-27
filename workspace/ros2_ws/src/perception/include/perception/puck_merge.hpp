// 카메라마다 따로 낸 퍽 관측에서 같은 퍽을 하나로 묶는다 (/perception/pucks 발행 전, ROS 의존 없음 → 단위 시험)
//
//   묶는 조건: 서로 다른 카메라의 관측, 색이 맞음(같은 색 · 한쪽이 모름), 중심 3D 거리 < merge_dist
//   배정: 조건을 만족하는 쌍을 거리가 가까운 것부터 일대일로 묶는다(탐욕). 한 관측은 다른 카메라의 관측 하나와만 묶인다
//     → 35 mm 옆 다른 퍽은 더 가까운 같은 퍽 관측이 먼저 묶여 남는다
//   1단계: D435 관측(tag_id < 0)끼리 카메라 사이로 묶는다. 묶음에서는 confidence 가 가장 높은 것(같으면 입력에서 앞선 것)을 남긴다
//   2단계: 태그 퍽(tag_id ≥ 0, 번호 · 모양 · 자세가 있음)과 1단계 묶음을 일대일로 묶는다. 거리는 묶음 구성원 중 가장 가까운 것,
//     색은 구성원 모두와 맞아야 한다. 묶인 D435 묶음은 통째로 빼고 태그 퍽을 남긴다
//   같은 카메라 관측끼리는 묶지 않는다 (D435 는 깊이 나누기로, 팔 태그는 번호로 이미 하나씩)
//   한계: 위치 · 색만 본다. D435 가 퍽 A 를 못 보고 30~40 mm 옆 같은 색 B 만 보면 B 가 태그 A 와 묶여 빠진다.
//     앞 · 뒤 관측이 서로 merge_dist 이상 떨어지면(각각은 태그와 가까워도) 1단계에서 안 묶여 하나가 남는다
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

namespace perception
{

struct PuckObs
{
  double x = 0, y = 0, z = 0;   // base_link 기준 퍽 중심 [m]
  int color = 0;                // 0 모름 · 1 빨강 · 2 초록 · 3 파랑 (Puck.COLOR_* 와 같음)
  int tag_id = -1;              // 태그로 읽은 번호, D435 관측은 -1
  int camera = 0;               // 카메라 번호 (Puck.CAMERA_*). 같은 번호끼리는 묶지 않음
  double confidence = 0;
};

namespace detail
{
inline bool color_match(int a, int b) {return a == b || a == 0 || b == 0;}

inline double dist(const PuckObs & a, const PuckObs & b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
}  // namespace detail

// 남길 관측의 번호 (입력 순서)
inline std::vector<size_t> merge_pucks(const std::vector<PuckObs> & obs, double merge_dist)
{
  using detail::color_match;
  using detail::dist;
  std::vector<size_t> tags, cams;
  for (size_t i = 0; i < obs.size(); ++i) {(obs[i].tag_id >= 0 ? tags : cams).push_back(i);}

  // 1단계: D435 관측 → 묶음. 묶음끼리 카메라가 겹치지 않을 때만 합친다 (카메라 둘이면 일대일)
  std::vector<std::vector<size_t>> groups;
  std::vector<size_t> group_of(obs.size());
  for (size_t i : cams) {group_of[i] = groups.size(); groups.push_back({i});}
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t a = 0; a < cams.size(); ++a) {
    for (size_t b = a + 1; b < cams.size(); ++b) {
      const PuckObs & p = obs[cams[a]], & q = obs[cams[b]];
      if (p.camera == q.camera || !color_match(p.color, q.color)) {continue;}
      const double d = dist(p, q);
      if (d < merge_dist) {pairs.emplace_back(d, cams[a], cams[b]);}
    }
  }
  std::stable_sort(pairs.begin(), pairs.end(), [](const auto & l, const auto & r) {return std::get<0>(l) < std::get<0>(r);});
  for (const auto & [d, i, j] : pairs) {
    auto & gi = groups[group_of[i]];
    auto & gj = groups[group_of[j]];
    if (group_of[i] == group_of[j]) {continue;}
    bool overlap = false;
    for (size_t m : gi) {
      for (size_t n : gj) {overlap = overlap || obs[m].camera == obs[n].camera || !color_match(obs[m].color, obs[n].color);}
    }
    if (overlap) {continue;}
    const size_t to = group_of[i];
    for (size_t n : gj) {group_of[n] = to;}
    gi.insert(gi.end(), gj.begin(), gj.end());
    gj.clear();
  }

  // 2단계: 태그 퍽 ↔ 묶음 일대일
  std::vector<std::tuple<double, size_t, size_t>> tag_pairs;   // (거리, 태그 관측, 묶음)
  for (size_t t : tags) {
    for (size_t g = 0; g < groups.size(); ++g) {
      if (groups[g].empty()) {continue;}
      double best = merge_dist;
      bool ok = true;
      for (size_t m : groups[g]) {
        if (obs[m].camera == obs[t].camera || !color_match(obs[m].color, obs[t].color)) {ok = false; break;}
        best = std::min(best, dist(obs[m], obs[t]));
      }
      if (ok && best < merge_dist) {tag_pairs.emplace_back(best, t, g);}
    }
  }
  std::stable_sort(tag_pairs.begin(), tag_pairs.end(), [](const auto & l, const auto & r) {return std::get<0>(l) < std::get<0>(r);});
  std::vector<bool> tag_used(obs.size(), false), group_used(groups.size(), false);
  for (const auto & [d, t, g] : tag_pairs) {
    if (tag_used[t] || group_used[g]) {continue;}
    tag_used[t] = true;
    group_used[g] = true;
  }

  std::vector<size_t> keep(tags);
  for (size_t g = 0; g < groups.size(); ++g) {
    if (groups[g].empty() || group_used[g]) {continue;}
    size_t rep = groups[g].front();
    for (size_t m : groups[g]) {
      if (obs[m].confidence > obs[rep].confidence || (obs[m].confidence == obs[rep].confidence && m < rep)) {rep = m;}
    }
    keep.push_back(rep);
  }
  std::sort(keep.begin(), keep.end());
  return keep;
}

}  // namespace perception
