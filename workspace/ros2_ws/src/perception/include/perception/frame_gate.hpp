// 입력 영상의 시각 규칙 (ROS 의존 없음 → 단위 시험)
//   StampGate    : 같은·과거 stamp(재전송)는 버린다. 단 restart 보다 크게 뒤로 가면 시각이 다시 시작된 것
//                  (시뮬레이션 리셋 · bag 반복 재생 · 카메라 재시작)으로 보고 받아들인다.
//                  그대로 두면 재시작 뒤의 영상을 모두 버리게 된다.
//   NearestCache : 컬러 영상과 stamp가 가장 가까운 깊이 영상을 찾는다. 허용 차이를 넘으면 없음.
//                  컬러를 깊이 도착까지 기다리지 않는다(깊이가 늦거나 빠져도 컬러 처리가 멈추지 않음).
#pragma once

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <optional>
#include <utility>

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

}  // namespace perception
