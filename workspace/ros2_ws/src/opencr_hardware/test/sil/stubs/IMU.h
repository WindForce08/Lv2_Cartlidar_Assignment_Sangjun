// OpenCR IMU 대역 (테스트 더블): opencr_firmware.ino 가 읽는 필드만, 고정값 (정지 · 수평)
#ifndef SIL_IMU_H_
#define SIL_IMU_H_

struct SilImuSensor
{
  int16_t gyroADC[3] = {0, 0, 0};
  int16_t accADC[3] = {0, 0, 0};
};

class cIMU
{
public:
  SilImuSensor SEN;
  float quat[4] = {1.f, 0.f, 0.f, 0.f};
  uint8_t begin(uint32_t = 200) {return 0;}
  uint16_t update(uint32_t = 0) {return 0;}
};

#endif  // SIL_IMU_H_
