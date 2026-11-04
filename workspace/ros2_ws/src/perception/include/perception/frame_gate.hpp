// 입력 영상의 시각 규칙 (ROS 의존 없음 → 단위 시험)
//   StampGate    : 같은·과거 stamp(재전송)는 버린다. 단 restart 보다 크게 뒤로 가면 시각이 다시 시작된 것
//                  (시뮬레이션 리셋 · bag 반복 재생 · 카메라 재시작)으로 보고 받아들인다.
//                  그대로 두면 재시작 뒤의 영상을 모두 버리게 된다.
//   NearestCache : 컬러 영상과 stamp가 가장 가까운 깊이 영상을 찾는다. 허용 차이를 넘으면 없음.
//                  컬러를 깊이 도착까지 기다리지 않는다(깊이가 늦거나 빠져도 컬러 처리가 멈추지 않음).
//   pick_tf_frame: 최근 컬러 몇 장 중 촬영 시각의 차체 TF(odom → base)가 이미 온 가장 새 장을 고른다.
//                  EKF 가 영상보다 조금(모의 0.008~0.025 s) 늦게 나와, 가장 새 장은 그 시각 TF 가 아직 없는 때가 많다.
//                  TF 가 없거나 멈췄거나 가장 새 장이 wait 보다 오래되면 가장 새 장(최근 TF 로 대신, lookup_near).
#pragma once

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace perception
{

enum class GateResult {Accept, Duplicate, Restart};

class StampGate
{
public:
  explicit StampGate(int64_t restart_ns = 1'000'000'000) : restart_ns_(restart_ns) {}

  GateResult check(int64_t ns)
  {
    if (!has_ || ns > last_) {
      has_ = true;
      last_ = ns;
      return GateResult::Accept;
    }
    if (last_ - ns > restart_ns_) {
      last_ = ns;
      return GateResult::Restart;
    }
    return GateResult::Duplicate;
  }

  void reset() {has_ = false;}

private:
  int64_t restart_ns_;
  int64_t last_ = 0;
  bool has_ = false;
};

template<class T>
class NearestCache
{
public:
  explicit NearestCache(size_t capacity = 5) : capacity_(capacity) {}

  void set_capacity(size_t capacity)
  {
    capacity_ = capacity;
    while (items_.size() > capacity_) {items_.pop_front();}
  }

  void push(int64_t ns, T v)
  {
    items_.emplace_back(ns, std::move(v));
    while (items_.size() > capacity_) {items_.pop_front();}
  }

  // tol_ns 안에서 ns와 가장 가까운 항목
  std::optional<std::pair<int64_t, T>> nearest(int64_t ns, int64_t tol_ns) const
  {
    const std::pair<int64_t, T> * best = nullptr;
    int64_t best_dt = 0;
    for (const auto & it : items_) {
      const int64_t dt = std::llabs(it.first - ns);
      if (!best || dt < best_dt) {best = &it; best_dt = dt;}
    }
    if (!best || best_dt > tol_ns) {return std::nullopt;}
    return *best;
  }

  void clear() {items_.clear();}
  size_t size() const {return items_.size();}

private:
  size_t capacity_;
  std::deque<std::pair<int64_t, T>> items_;
};

// ns: 최근 컬러 stamp, 오래된 → 새 순. used_ns: 지난 결과에 쓴 stamp(그보다 오래된 장은 고르지 않음)
// tf_latest_ns: odom → base 의 가장 최근 TF stamp(< 0 이면 TF 없음). 돌려준 값이 없으면 이번 주기는 기다린다
inline std::optional<size_t> pick_tf_frame(
  const std::vector<int64_t> & ns, int64_t used_ns, int64_t now_ns, int64_t tf_latest_ns, int64_t wait_ns)
{
  if (ns.empty()) {return std::nullopt;}
  const size_t last = ns.size() - 1;
  if (tf_latest_ns < 0 || tf_latest_ns >= ns[last] || now_ns - ns[last] >= wait_ns || ns[last] - tf_latest_ns >= wait_ns) {
    return last;
  }
  for (size_t i = last; i-- > 0;) {
    if (ns[i] < used_ns) {break;}
    if (tf_latest_ns >= ns[i]) {return i;}
  }
  return std::nullopt;
}

}  // namespace perception
