// OpenCR ↔ Jetson USB 시리얼 패킷 (opencr_hardware.cpp · opencr_firmware.ino 공용)
// 이 파일은 opencr_firmware/opencr_protocol.h 와 내용이 같아야 한다 (test_protocol 이 비교).
// 표준 라이브러리 없이 <stdint.h> · <string.h> 만 써서 Arduino(OpenCR)에서도 그대로 컴파일된다.
//
// 프레임: 0xFF 0xFD | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16(2, 리틀엔디안)
//   CRC 범위: 헤더 2바이트 ~ PAYLOAD 끝. Dynamixel 프로토콜 2.0과 같은 CRC-16 (다항식 0x8005, 초기값 0).
//   모든 다바이트 값은 리틀엔디안, 실수는 IEEE-754 float32.
// CMD   0x01  Jetson → OpenCR 100 Hz  : role · wheel_vel[6] · arm_pos[6] · gripper_pos · flags · seq   (55 B)
// STATE 0x81  OpenCR → Jetson 100 Hz  : role · pos[12] · vel[12] · effort[12] · quat[4] · gyro[3] · accel[3]
//                                       · battery_v · error · fw_version · seq                         (192 B)
//   fw_version = PROTOCOL_VERSION — 호스트는 다르면 활성화를 거부 (Xmen 의 펌웨어 버전 확인과 같은 목적)
//   role = ROLE_WHEEL(1) 또는 ROLE_ARM(2). 각 OpenCR은 자기 역할의 배열 칸만 채우거나 적용한다.
//   관절 순서(12): 바퀴 슬롯 0~5 → 0~5, 팔 슬롯 0~5 → 6~11. 쓰지 않는 칸은 0.
//   v5 = v4 와 길이 · 오프셋이 같고 FLAG_REARM · ERR_HOLD · ERR_FAULT 비트만 추가. 버전 값으로만 구분된다.
#ifndef OPENCR_PROTOCOL_H_
#define OPENCR_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace opencr_proto
{

static const uint8_t HDR0 = 0xFF;
static const uint8_t HDR1 = 0xFD;
static const uint8_t TYPE_CMD = 0x01;
static const uint8_t TYPE_STATE = 0x81;
static const uint8_t PROTOCOL_VERSION = 5;   // v4: 역할 필드 추가 (v3 와 길이가 다름). v5: REARM · HOLD · FAULT 비트

// 보드 역할은 물리 USB 포트 번호와 무관하게 프레임 안에서 확인한다.
static const uint8_t ROLE_WHEEL = 1;
static const uint8_t ROLE_ARM = 2;

static const int MAX_WHEELS = 6;
static const int ARM_JOINTS = 6;          // joint1~5 + gripper_roll
static const int STATE_JOINTS = MAX_WHEELS + ARM_JOINTS;

// CMD flags
static const uint8_t FLAG_TORQUE_ON = 0x01;
static const uint8_t FLAG_ESTOP = 0x02;
static const uint8_t FLAG_GRIPPER_VALID = 0x04;   // 그리퍼 개폐 액추에이터가 정해지면 사용
static const uint8_t FLAG_REARM = 0x08;   // 0→1 상승 에지에서만 HOLD → ACTIVE 재활성화 요청 (계속 1 이면 한 번만)

// STATE error bits
static const uint8_t ERR_SYNCREAD = 0x01;   // SyncRead 응답 부족
static const uint8_t ERR_CMD_TIMEOUT = 0x02;   // 200 ms 동안 CMD 없음 → 안전 정지 중
static const uint8_t ERR_HW_ESTOP = 0x04;   // 물리 E-stop 눌림
static const uint8_t ERR_MOTOR_HW = 0x08;   // 모터 Hardware Error Status ≠ 0
static const uint8_t ERR_MOTOR_CONFIG = 0x10;   // 부팅 점검 실패: 응답 없음 · 모델 다름 · Homing Offset ≠ 0
static const uint8_t ERR_ROLE_MISMATCH = 0x20;  // 호스트 CMD의 역할이 이 보드와 다름
static const uint8_t ERR_HOLD = 0x40;   // 팔이 유지 목표를 붙잡고 새 목표를 받지 않는 중 — REARM 상승 에지 필요
static const uint8_t ERR_FAULT = 0x80;  // 래치된 오류 (현재: 부팅 점검 실패) — 토크를 켜지 않고 재부팅으로만 해제

static const size_t CMD_PAYLOAD = (MAX_WHEELS + ARM_JOINTS + 1) * 4 + 3;            // role + values + flags + seq = 55
static const size_t STATE_PAYLOAD = (STATE_JOINTS * 3 + 4 + 3 + 3 + 1) * 4 + 4;     // role + error + version + seq = 192
static const size_t FRAME_OVERHEAD = 2 + 1 + 1 + 2;
static const size_t MAX_PAYLOAD = STATE_PAYLOAD;
static const size_t MAX_FRAME = MAX_PAYLOAD + FRAME_OVERHEAD;

struct Cmd
{
  uint8_t role;                  // ROLE_WHEEL 또는 ROLE_ARM
  float wheel_vel[MAX_WHEELS];   // rad/s
  float arm_pos[ARM_JOINTS];     // rad
  float gripper_pos;             // rad 또는 m (액추에이터 미정, FLAG_GRIPPER_VALID 일 때만 사용)
  uint8_t flags;
  uint8_t seq;
};

struct State
{
  uint8_t role;                  // ROLE_WHEEL 또는 ROLE_ARM
  float pos[STATE_JOINTS];       // rad
  float vel[STATE_JOINTS];       // rad/s
  float effort[STATE_JOINTS];    // A (Present Current)
  float quat[4];                 // w, x, y, z
  float gyro[3];                 // rad/s
  float accel[3];                // m/s^2
  float battery_v;               // 전원 입력 전압 (V)
  uint8_t error;
  uint8_t fw_version;            // PROTOCOL_VERSION
  uint8_t seq;
};

// Dynamixel 2.0 CRC-16 (CRC-16/BUYPASS: 다항식 0x8005, 초기값 0, 반사 없음)
inline uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline size_t put_floats(uint8_t * p, const float * v, int n)
{
  memcpy(p, v, (size_t)n * 4);
  return (size_t)n * 4;
}
inline size_t get_floats(const uint8_t * p, float * v, int n)
{
  memcpy(v, p, (size_t)n * 4);
  return (size_t)n * 4;
}

// payload → 완성된 프레임. out 은 MAX_FRAME 이상. 반환: 프레임 길이
inline size_t encode_frame(uint8_t type, const uint8_t * payload, size_t len, uint8_t * out)
{
  out[0] = HDR0;
  out[1] = HDR1;
  out[2] = type;
  out[3] = (uint8_t)len;
  memcpy(out + 4, payload, len);
  const uint16_t c = crc16(out, 4 + len);
  out[4 + len] = (uint8_t)(c & 0xFF);
  out[5 + len] = (uint8_t)(c >> 8);
  return len + FRAME_OVERHEAD;
}

inline size_t encode_cmd(const Cmd & c, uint8_t * out)
{
  uint8_t pl[CMD_PAYLOAD];
  size_t i = 0;
  pl[i++] = c.role;
  i += put_floats(pl + i, c.wheel_vel, MAX_WHEELS);
  i += put_floats(pl + i, c.arm_pos, ARM_JOINTS);
  i += put_floats(pl + i, &c.gripper_pos, 1);
  pl[i++] = c.flags;
  pl[i++] = c.seq;
  return encode_frame(TYPE_CMD, pl, i, out);
}

inline bool decode_cmd(const uint8_t * pl, size_t len, Cmd & c)
{
  if (len != CMD_PAYLOAD) {return false;}
  size_t i = 0;
  c.role = pl[i++];
  i += get_floats(pl + i, c.wheel_vel, MAX_WHEELS);
  i += get_floats(pl + i, c.arm_pos, ARM_JOINTS);
  i += get_floats(pl + i, &c.gripper_pos, 1);
  c.flags = pl[i++];
  c.seq = pl[i++];
  return true;
}

inline size_t encode_state(const State & s, uint8_t * out)
{
  uint8_t pl[STATE_PAYLOAD];
  size_t i = 0;
  pl[i++] = s.role;
  i += put_floats(pl + i, s.pos, STATE_JOINTS);
  i += put_floats(pl + i, s.vel, STATE_JOINTS);
  i += put_floats(pl + i, s.effort, STATE_JOINTS);
  i += put_floats(pl + i, s.quat, 4);
  i += put_floats(pl + i, s.gyro, 3);
  i += put_floats(pl + i, s.accel, 3);
  i += put_floats(pl + i, &s.battery_v, 1);
  pl[i++] = s.error;
  pl[i++] = s.fw_version;
  pl[i++] = s.seq;
  return encode_frame(TYPE_STATE, pl, i, out);
}

inline bool decode_state(const uint8_t * pl, size_t len, State & s)
{
  if (len != STATE_PAYLOAD) {return false;}
  size_t i = 0;
  s.role = pl[i++];
  i += get_floats(pl + i, s.pos, STATE_JOINTS);
  i += get_floats(pl + i, s.vel, STATE_JOINTS);
  i += get_floats(pl + i, s.effort, STATE_JOINTS);
  i += get_floats(pl + i, s.quat, 4);
  i += get_floats(pl + i, s.gyro, 3);
  i += get_floats(pl + i, s.accel, 3);
  i += get_floats(pl + i, &s.battery_v, 1);
  s.error = pl[i++];
  s.fw_version = pl[i++];
  s.seq = pl[i++];
  return true;
}

// 바이트 단위 수신 파서. push() 가 true 를 돌려주면 type() · payload() · length() 가 유효하다.
// CRC 가 틀리면 버리고 다음 헤더를 찾는다 (bad_frames 증가).
class FrameParser
{
public:
  FrameParser()
  : state_(0), type_(0), len_(0), idx_(0), bad_frames_(0) {}

  bool push(uint8_t b)
  {
    switch (state_) {
      case 0:
        if (b == HDR0) {buf_[0] = b; state_ = 1;}
        return false;
      case 1:
        if (b == HDR1) {buf_[1] = b; state_ = 2;} else if (b != HDR0) {state_ = 0;}
        return false;
      case 2:
        type_ = b; buf_[2] = b; state_ = 3;
        return false;
      case 3:
        len_ = b; buf_[3] = b; idx_ = 0;
        if (len_ > MAX_PAYLOAD) {state_ = 0; ++bad_frames_; return false;}
        state_ = (len_ == 0) ? 5 : 4;
        return false;
      case 4:
        buf_[4 + idx_++] = b;
        if (idx_ >= len_) {state_ = 5; idx_ = 0;}
        return false;
      case 5:
        crc_lo_ = b; state_ = 6;
        return false;
      default: {
          state_ = 0;
          const uint16_t rx = (uint16_t)(crc_lo_ | ((uint16_t)b << 8));
          if (rx != crc16(buf_, 4 + len_)) {++bad_frames_; return false;}
          return true;
        }
    }
  }

  uint8_t type() const {return type_;}
  size_t length() const {return len_;}
  const uint8_t * payload() const {return buf_ + 4;}
  uint32_t bad_frames() const {return bad_frames_;}

private:
  uint8_t buf_[MAX_FRAME];
  int state_;
  uint8_t type_;
  size_t len_;
  size_t idx_;
  uint8_t crc_lo_ = 0;
  uint32_t bad_frames_;
};

}  // namespace opencr_proto

#endif  // OPENCR_PROTOCOL_H_
