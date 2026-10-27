// SIL (software-in-the-loop) 보드 — 실제 opencr_firmware.ino 를 호스트 g++ 로 컴파일해 돌리는 틀.
//   실제 코드: opencr_firmware.ino · config.h · opencr_protocol.h · safety_core.h (수정 없이 그대로)
//   테스트 더블: Dynamixel 버스 (SilMotor) · USB CDC (host_rx / host_tx) · 시계 · GPIO · 전원 전압 · IMU
//   더블의 동작은 Dynamixel2Arduino 0.8.2 · OpenCR 1.5.3 소스를 읽고 흉내 낸 모델이다 — 실물과 같다고 보장하지 않는다.
//   역할마다 다른 namespace 로 컴파일되므로 바퀴 · 팔 펌웨어가 한 시험 프로세스에 같이 있다 (sil_firmware.cpp).
#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <vector>

struct SilMotor
{
  bool present = true;          // ping · 상태 패킷에 응답
  uint16_t model = 1030;        // 시험이 config.h 의 WHEEL_MODEL · ARM_MODEL 로 바꿔 넣는다
  int32_t homing_offset = 0;
  int32_t min_position_limit = 0;      // EEPROM Min/Max Position Limit (공장값 0 · 4095)
  int32_t max_position_limit = 4095;
  int32_t pos_p_gain = 800;            // 위치 게인 (RAM, 공장값 800 · 0 · 0)
  int32_t pos_i_gain = 0;
  int32_t pos_d_gain = 0;
  bool ignore_gain_writes = false;     // 시험용: 게인 쓰기가 반영되지 않는 모터
  int32_t pwm_limit = 885;             // EEPROM PWM Limit (공장값 885 = 100 %). 시험이 그리퍼는 config.h GRIP_PWM_LIMIT_MAX 로 바꿔 넣는다
  // 막힘 · 속도 모델 (토크 ON + motors_follow_goals 일 때): SyncWrite 마다 목표 쪽으로 max_step tick 까지 (0 = 바로 도달),
  //   위치는 stop_min ~ stop_max 를 넘지 못함 (물체에 막힘 — 예: 그리퍼가 퍽을 쥠)
  int32_t max_step = 0;
  int32_t stop_min = std::numeric_limits<int32_t>::min();
  int32_t stop_max = std::numeric_limits<int32_t>::max();
  uint8_t hw_error = 0;         // Hardware Error Status
  uint8_t mode = 3;
  bool torque = false;
  int32_t watchdog = 0;
  int32_t goal_position = 2048;
  int32_t goal_velocity = 0;
  int32_t position = 2048;      // Present Position (tick)
  int32_t velocity = 0;
  int16_t current = 0;
};

struct SilWrite
{
  uint8_t item;    // ControlTableItem (대역의 enum 값)
  uint8_t id;
  int32_t value;
};

struct SilBoard
{
  std::map<uint8_t, SilMotor> motors;
  std::deque<uint8_t> host_rx;     // 호스트 → OpenCR
  std::vector<uint8_t> host_tx;    // OpenCR → 호스트
  std::vector<SilWrite> writes;    // 단일 Control Table 쓰기 기록 (SyncWrite 제외)
  int sync_writes = 0;
  int failed_reads = 0;            // 응답 없는 모터에 readControlTableItem (한 번에 timeout 만큼 막힘)
  uint64_t now_us = 0;
  bool estop_pin_low = false;
  float vin = 12.0f;
  bool motors_follow_goals = true;   // 토크 ON 모터가 목표에 즉시 도달 (이상적 모델)
  void (*setup)() = nullptr;
  void (*loop)() = nullptr;
};

// 각 역할 펌웨어의 보드 (setup · loop 포인터가 채워져 있음)
SilBoard & sil_wheel_board();
SilBoard & sil_arm_board();

// 대역 enum 값 — 쓰기 기록을 확인할 때 (stubs/Dynamixel2Arduino.h 와 같은 순서)
namespace sil_item
{
enum : uint8_t { HOMING_OFFSET, RETURN_DELAY_TIME, BUS_WATCHDOG, PROFILE_VELOCITY, PROFILE_ACCELERATION,
  HARDWARE_ERROR_STATUS, TORQUE_ENABLE, GOAL_POSITION, GOAL_VELOCITY, OPERATING_MODE, MIN_POSITION_LIMIT, MAX_POSITION_LIMIT,
  POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN, PWM_LIMIT };
}
