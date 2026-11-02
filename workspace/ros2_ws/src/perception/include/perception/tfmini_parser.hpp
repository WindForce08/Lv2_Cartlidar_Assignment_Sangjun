// TFmini Plus 9바이트 프레임 파서 (ROS 의존 없음)
//   0x59 0x59 | 거리 (2B LE, 단위 cm 또는 mm) | 신호세기 (2B LE) | 온도 (2B LE, /8-256 °C) | 체크섬 (앞 8바이트 합의 하위 바이트)
//   단위: 기본 출력은 cm. 박스 거리를 mm 로 읽으려면 켤 때 TFMINI_CMD_OUTPUT_MM 을 보내고 TfminiParser(0.001) 로 읽는다
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception
{

struct TfminiSample
{
  double distance_m = 0.0;
  uint16_t strength = 0;
  double temperature_c = 0.0;
};

// 출력 형식 명령 (TFmini Plus 설명서 ID 0x05): 표준 9바이트 mm · cm. 저장(0x11)은 보내지 않으므로 켤 때마다 보낸다
inline constexpr uint8_t TFMINI_CMD_OUTPUT_MM[5] = {0x5A, 0x05, 0x05, 0x06, 0x6A};
inline constexpr uint8_t TFMINI_CMD_OUTPUT_CM[5] = {0x5A, 0x05, 0x05, 0x01, 0x65};

class TfminiParser
{
public:
  explicit TfminiParser(double unit_m = 0.01) : unit_m_(unit_m) {}

  // 프레임이 완성되고 체크섬이 맞으면 true
  bool push(uint8_t b, TfminiSample & out)
  {
    if (n_ == 0 && b != 0x59) {return false;}
    if (n_ == 1 && b != 0x59) {n_ = 0; return false;}
    buf_[n_++] = b;
    if (n_ < 9) {return false;}
    n_ = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 8; ++i) {sum = static_cast<uint8_t>(sum + buf_[i]);}
    if (sum != buf_[8]) {
      ++bad_;
      // 체크섬 실패: 버퍼 안에 다음 헤더가 있으면 그 자리에서 다시 시작
      for (int i = 1; i < 8; ++i) {
        if (buf_[i] == 0x59 && buf_[i + 1] == 0x59) {
          for (int k = i; k < 9; ++k) {buf_[n_++] = buf_[k];}
          break;
        }
      }
      return false;
    }
    const uint16_t dist = static_cast<uint16_t>(buf_[2] | (buf_[3] << 8));
    out.strength = static_cast<uint16_t>(buf_[4] | (buf_[5] << 8));
    const uint16_t temp = static_cast<uint16_t>(buf_[6] | (buf_[7] << 8));
    out.distance_m = dist * unit_m_;
    out.temperature_c = temp / 8.0 - 256.0;
    return true;
  }

  uint32_t bad_frames() const {return bad_;}

private:
  double unit_m_;
  uint8_t buf_[9]{};
  size_t n_ = 0;
  uint32_t bad_ = 0;
};

// 유효성 판정: 신호세기 < min_strength (또는 65535 = 과포화) 이거나 범위 밖이면 무효
inline bool tfmini_valid(const TfminiSample & s, uint16_t min_strength, double min_m, double max_m)
{
  return s.strength >= min_strength && s.strength != 65535 && s.distance_m >= min_m && s.distance_m <= max_m;
}

// 실물 보정 (박스 앞면 줄자 실측): 측정값 raw → 실제 거리. raw_m · true_m 은 같은 길이, raw_m 오름차순.
//   점이 없으면 offset_m 만 더한다. 1점이면 그 점의 차이를 오프셋으로, 2점 이상이면 구간 선형 보간(양 끝 밖은 끝 구간 기울기로 연장)
inline double tfmini_calibrate(double raw, const std::vector<double> & raw_m, const std::vector<double> & true_m,
  double offset_m = 0.0)
{
  const size_t n = std::min(raw_m.size(), true_m.size());
  if (n == 0) {return raw + offset_m;}
  if (n == 1) {return raw + (true_m[0] - raw_m[0]) + offset_m;}
  size_t i = 1;
  while (i < n - 1 && raw > raw_m[i]) {++i;}
  const double x0 = raw_m[i - 1], x1 = raw_m[i];
  const double t = x1 > x0 ? (raw - x0) / (x1 - x0) : 0.0;
  return true_m[i - 1] + t * (true_m[i] - true_m[i - 1]) + offset_m;
}

}  // namespace perception
