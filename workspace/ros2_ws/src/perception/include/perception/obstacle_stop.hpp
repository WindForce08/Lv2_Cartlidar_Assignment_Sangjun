// 앞쪽 거리 센서(TFmini Plus, 1층 정면 바닥)로 비상 정지 여부를 정한다 (ROS 의존 없음 → 단위 시험)
//
//   정지: 유효 거리 < stop_m
//   해제: 정지 중에 "앞이 비었다"는 측정(유효 거리 ≥ release_m, 또는 아래의 비어 있음 판정)이 release_count 번
//     연속으로 들어올 때만 (가까운 값이 흔들려도 깜빡이지 않게). 켤 때와 입력 끊김 뒤에도 같은 규칙으로 푼다
//   무효 값(신호 약함 · 측정 범위 밖 = +inf): TFmini 는 "너무 가까움(최소 거리 미만)"과 "앞에 아무것도 없음"을
//     같은 무효로 낸다. 그래서 직전 유효 거리가 near_m 보다 가까웠으면 너무 가까운 것으로 보고 정지를 유지하고,
//     멀었으면 앞이 비어 있는 것으로 본다
//   입력 끊김: stale_s 동안 값이 없으면 정지 (센서·케이블 고장 때 멈추는 쪽)
//   박스 접근 (set_approach): TFmini 는 퍽이 놓인 박스 앞면을 잰다. drive FINE 정렬이 박스 앞 목표 거리까지 다가가는 동안
//     정지 거리를 그 목표보다 조금 짧은 값(approach_stop_m)으로 바꾼다 — "목표보다 더 가까워지면 정지"는 그대로 남는다.
//     해제 · 너무 가까움 판정 거리는 approach_stop_m + approach_hysteresis 로 함께 바꾼다
#pragma once

#include <cmath>
#include <optional>
#include <string>

namespace perception
{

struct StopConfig
{
  double stop_m = 0.20;      // 이보다 가까우면 정지 [m]
  double release_m = 0.30;   // 정지를 풀 거리 [m] (stop_m 보다 커야 함)
  int release_count = 3;     // 해제에 필요한 연속 유효 횟수
  double near_m = 0.30;      // 무효 값을 "너무 가까움"으로 볼 직전 거리 상한 [m]
  double stale_s = 0.2;      // 입력이 이만큼 없으면 정지 [s]
  double approach_hysteresis = 0.02;   // 박스 접근 중 해제 · 너무 가까움 거리 = approach_stop_m + 이 값 [m]
};

class ObstacleStop
{
public:
  explicit ObstacleStop(const StopConfig & c) : c_(c) {}

  // 측정 하나. range 가 nullopt 이면 무효 값. t = 받은 시각 [s]
  bool update(double t, std::optional<double> range)
  {
    last_t_ = t;
    have_input_ = true;
    if (range && std::isfinite(*range)) {
      const double r = *range;
      last_valid_ = r;
      if (r < stop_m()) {
        set(true, "obstacle");
        ok_count_ = 0;
      } else if (r >= release_m()) {
        clear_evidence("clear");
      } else if (stopped_) {
        ok_count_ = 0;   // stop_m ~ release_m 사이: 정지 중이면 그대로 유지
      } else {
        set(false, "clear");
      }
      return stopped_;
    }
    if (last_valid_ && *last_valid_ < near_m()) {
      set(true, "too_close");   // 최소 측정 거리 안으로 들어온 것으로 본다
      ok_count_ = 0;
    } else {
      clear_evidence("no_return");   // 앞이 비어 있음
    }
    return stopped_;
  }

  // 주기적으로 불러 입력 끊김을 본다. t = 지금 시각 [s]
  bool check_stale(double t)
  {
    if (!have_input_ || t - last_t_ > c_.stale_s) {set(true, "stale"); ok_count_ = 0;}
    return stopped_;
  }

  // 박스 접근: stop_m 을 approach_stop_m 으로 (nullopt = 평소 값으로 되돌림). 지금 상태는 다음 측정부터 새 거리로 판정
  void set_approach(std::optional<double> approach_stop_m) {approach_ = approach_stop_m;}
  std::optional<double> approach() const {return approach_;}

  double stop_m() const {return approach_ ? *approach_ : c_.stop_m;}
  double release_m() const {return approach_ ? *approach_ + c_.approach_hysteresis : c_.release_m;}
  double near_m() const {return approach_ ? *approach_ + c_.approach_hysteresis : c_.near_m;}

  bool stopped() const {return stopped_;}
  const std::string & reason() const {return reason_;}

private:
  void set(bool s, const char * why) {stopped_ = s; reason_ = why;}

  // 앞이 비었다는 측정 하나. 정지 중이면 release_count 번 연속이어야 푼다
  void clear_evidence(const char * why)
  {
    if (!stopped_) {set(false, why); return;}
    if (++ok_count_ >= c_.release_count) {set(false, why); ok_count_ = 0;}
  }

  StopConfig c_;
  std::optional<double> approach_;
  bool stopped_ = true;            // 첫 값이 오기 전에는 정지 (켜자마자 움직이지 않게)
  std::string reason_ = "stale";
  int ok_count_ = 0;
  bool have_input_ = false;
  double last_t_ = 0.0;
  std::optional<double> last_valid_;
};

}  // namespace perception
