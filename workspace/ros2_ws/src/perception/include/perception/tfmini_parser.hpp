// TFmini Plus 9바이트 프레임 파서 (ROS 의존 없음)
//   0x59 0x59 | 거리 cm (2B LE) | 신호세기 (2B LE) | 온도 (2B LE, /8-256 °C) | 체크섬 (앞 8바이트 합의 하위 바이트)
#pragma once

#include <cstddef>
#include <cstdint>

namespace perception
{

struct TfminiSample
{
  double distance_m = 0.0;
  uint16_t strength = 0;
  double temperature_c = 0.0;
};

class TfminiParser
{
public:
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
    out.distance_m = dist / 100.0;
    out.temperature_c = temp / 8.0 - 256.0;
    return true;
  }

  uint32_t bad_frames() const {return bad_;}

private:
  uint8_t buf_[9]{};
  size_t n_ = 0;
  uint32_t bad_ = 0;
};

// 유효성 판정: 신호세기 < min_strength (또는 65535 = 과포화) 이거나 범위 밖이면 무효
inline bool tfmini_valid(const TfminiSample & s, uint16_t min_strength, double min_m, double max_m)
{
  return s.strength >= min_strength && s.strength != 65535 && s.distance_m >= min_m && s.distance_m <= max_m;
}

}  // namespace perception
