// opencr_firmware.ino — OpenCR 보드 (Firmware 팀)
// Jetson(opencr_hardware) ↔ USB CDC ↔ role별 OpenCR ↔ RS-485 1 Mbps ↔ 해당 Dynamixel 6개
// OPENCR_BOARD_ROLE=1: 바퀴 ID 1~6 / Velocity Mode. =2: 팔 ID 11~16 / Position Mode.
//
// loop (100 Hz):
//   receiveHost()  헤더 · 길이 · CRC 검증 후 CMD 해석
//   handleCmd()    역할이 일치하는 명령만 받아 rad/s 또는 rad → 모터 목표로 변환
//   applySafety()  200 ms 무수신 · ESTOP 비트 · 물리 스위치 → 바퀴 0 또는 팔 현재 위치 유지
//   writeGoals()   역할에 해당하는 SyncWrite만 수행
//   readState()    SyncRead addr 126 부터 10 B, 응답 부족 시 오류 비트
//   sendState()    상태 + IMU + 배터리 전압 + 오류 비트
//
// 부팅 점검 (Xmen opencr_control.ino prepareAxis 참고): ping · 모델 번호 · Homing Offset 0 · Return Delay 0
//   하나라도 어긋나면 ERR_MOTOR_CONFIG → 호스트가 활성화를 거부. 토크 ON 때 Bus Watchdog 500 ms 를 켬.
// 필요 라이브러리: OpenCR 보드 패키지 (Arduino IDE), Dynamixel2Arduino ≥ 0.6, OpenCR IMU (보드 패키지 내장)
#include <Dynamixel2Arduino.h>
#include <IMU.h>

#include "config.h"
#include "opencr_protocol.h"

using namespace ControlTableItem;
using opencr_proto::Cmd;
using opencr_proto::State;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
cIMU imu;

// ---- 상태 ----
static const uint8_t BOARD_ROLE = OPENCR_BOARD_ROLE;
static const int N_MOTORS = BOARD_ROLE == opencr_proto::ROLE_WHEEL ? WHEEL_COUNT : ARM_COUNT;
static uint8_t g_ids[N_MOTORS];

struct __attribute__((packed)) PresentData { int16_t current; int32_t velocity; int32_t position; };
static PresentData g_present[N_MOTORS];

static int32_t g_wheel_goal_raw[WHEEL_COUNT];
static int32_t g_arm_goal_raw[ARM_COUNT];
static int32_t g_gripper_goal_raw = 0;

static Cmd g_cmd;
static bool g_have_cmd = false;
static uint32_t g_last_cmd_ms = 0;
static bool g_torque_on = false;
static bool g_safe_hold = false;     // ESTOP · 무수신으로 팔을 붙잡은 상태
static uint8_t g_error = 0;
static uint8_t g_state_seq = 0;
static uint8_t g_hw_error_mask = 0;
static float g_battery_v = 0.f;      // 저역 통과한 전원 전압
static bool g_config_error = false;  // 부팅 점검 실패 (래치 — 재부팅으로만 지움)  // 모터별 HW 오류 여부를 모아 둠 (비트 1개로 보고)

static opencr_proto::FrameParser g_parser;

// SyncRead / SyncWrite 정보 (Dynamixel2Arduino 0.6 API)
static const uint16_t PKT_BUF_CAP = 256;
static uint8_t g_pkt_buf[PKT_BUF_CAP];
static DYNAMIXEL::InfoSyncReadInst_t g_sr;
static DYNAMIXEL::XELInfoSyncRead_t g_sr_xels[N_MOTORS];
static DYNAMIXEL::InfoSyncWriteInst_t g_sw_wheel, g_sw_arm;
static DYNAMIXEL::XELInfoSyncWrite_t g_sw_wheel_xels[WHEEL_COUNT], g_sw_arm_xels[ARM_COUNT];

// ---- 단위 변환 ----
static int32_t radToTick(float rad, int8_t dir)
{
  int32_t t = TICK_CENTER + (int32_t)lroundf(dir * rad / RAD_PER_TICK);
  if (t < 0) {t = 0;}
  if (t > 4095) {t = 4095;}
  return t;
}
static float tickToRad(int32_t tick, int8_t dir) {return dir * (tick - TICK_CENTER) * RAD_PER_TICK;}

static int32_t wheelRadsToRaw(float wheel_rads, int8_t dir)
{
  const float motor_rads = wheel_rads / WHEEL_GEAR;
  int32_t raw = (int32_t)lroundf(dir * motor_rads / RADS_PER_VEL_RAW);
  if (raw > WHEEL_VEL_LIMIT_RAW) {raw = WHEEL_VEL_LIMIT_RAW;}
  if (raw < -WHEEL_VEL_LIMIT_RAW) {raw = -WHEEL_VEL_LIMIT_RAW;}
  return raw;
}

static float clampf(float v, float lo, float hi) {return v < lo ? lo : (v > hi ? hi : v);}

// ---- 초기화 ----
static void setupMotor(uint8_t id, uint8_t mode, bool is_arm, uint16_t model)
{
  if (!dxl.ping(id)) {g_config_error = true; return;}   // 응답 없음
  if (dxl.getModelNumber(id) != model) {g_config_error = true;}
  dxl.torqueOff(id);
  // 각도 변환은 Homing Offset 0 을 가정 — 0 이 아니면 위치가 어긋나므로 점검 실패로 알림 (값은 바꾸지 않음)
  if (dxl.readControlTableItem(HOMING_OFFSET, id) != 0) {g_config_error = true;}
  dxl.writeControlTableItem(RETURN_DELAY_TIME, id, 0);   // SyncRead 응답 지연 최소
  dxl.writeControlTableItem(BUS_WATCHDOG, id, 0);        // 이전 Watchdog 오류 해제
  dxl.setOperatingMode(id, mode);
  if (is_arm) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 0);   // 궤적 보간은 arm_controller 가 함
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 0);
  }
}

static void setupSync()
{
  g_sr.packet.p_buf = g_pkt_buf;
  g_sr.packet.buf_capacity = PKT_BUF_CAP;
  g_sr.packet.is_completed = false;
  g_sr.addr = ADDR_PRESENT_START;
  g_sr.addr_length = LEN_PRESENT;
  g_sr.p_xels = g_sr_xels;
  g_sr.xel_count = 0;
  for (int i = 0; i < N_MOTORS; ++i) {
    g_sr_xels[i].id = g_ids[i];
    g_sr_xels[i].p_recv_buf = (uint8_t *)&g_present[i];
    g_sr.xel_count++;
  }
  g_sr.is_info_changed = true;

  g_sw_wheel.packet.p_buf = nullptr;
  g_sw_wheel.packet.is_completed = false;
  g_sw_wheel.addr = ADDR_GOAL_VELOCITY;
  g_sw_wheel.addr_length = 4;
  g_sw_wheel.p_xels = g_sw_wheel_xels;
  g_sw_wheel.xel_count = 0;
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      g_sw_wheel_xels[i].id = WHEEL_IDS[i];
      g_sw_wheel_xels[i].p_data = (uint8_t *)&g_wheel_goal_raw[i];
      g_sw_wheel.xel_count++;
    }
  }
  g_sw_wheel.is_info_changed = true;

  g_sw_arm.packet.p_buf = nullptr;
  g_sw_arm.packet.is_completed = false;
  g_sw_arm.addr = ADDR_GOAL_POSITION;
  g_sw_arm.addr_length = 4;
  g_sw_arm.p_xels = g_sw_arm_xels;
  g_sw_arm.xel_count = 0;
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {
    for (int i = 0; i < ARM_COUNT; ++i) {
      g_sw_arm_xels[i].id = ARM_IDS[i];
      g_sw_arm_xels[i].p_data = (uint8_t *)&g_arm_goal_raw[i];
      g_sw_arm.xel_count++;
    }
  }
  g_sw_arm.is_info_changed = true;
}

// ---- 상태 읽기 ----
static void readState()
{
  const uint8_t recv = dxl.syncRead(&g_sr);
  if (recv < N_MOTORS) {
    g_error |= opencr_proto::ERR_SYNCREAD;
  } else {
    g_error &= ~opencr_proto::ERR_SYNCREAD;
  }
  // HW 오류 상태: 한 주기에 모터 하나씩 확인
  static uint32_t last_poll = 0;
  static int k = 0;
  if (millis() - last_poll >= HW_ERR_POLL_MS) {
    last_poll = millis();
    const int32_t st = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, g_ids[k]);
    if (st > 0) {g_hw_error_mask |= 1;} else if (k == 0) {g_hw_error_mask = 0;}
    k = (k + 1) % N_MOTORS;
  }
  if (g_hw_error_mask) {g_error |= opencr_proto::ERR_MOTOR_HW;} else {g_error &= ~opencr_proto::ERR_MOTOR_HW;}
}

static void holdArmAtPresent()
{
  if (BOARD_ROLE != opencr_proto::ROLE_ARM) {return;}
  for (int i = 0; i < ARM_COUNT; ++i) {
    g_arm_goal_raw[i] = g_present[i].position;
  }
}

// ---- 호스트 수신 ----
static void receiveHost()
{
  while (HOST_SERIAL.available() > 0) {
    const uint8_t b = (uint8_t)HOST_SERIAL.read();
    if (g_parser.push(b) && g_parser.type() == opencr_proto::TYPE_CMD) {
      Cmd c;
      if (opencr_proto::decode_cmd(g_parser.payload(), g_parser.length(), c)) {
        if (c.role != BOARD_ROLE) {
          g_error |= opencr_proto::ERR_ROLE_MISMATCH;
          continue;
        }
        g_error &= ~opencr_proto::ERR_ROLE_MISMATCH;
        g_cmd = c;
        g_have_cmd = true;
        g_last_cmd_ms = millis();
      }
    }
  }
}

// ---- 명령 적용 ----
static void handleCmd()
{
  if (!g_have_cmd) {return;}
  const bool want_torque = (g_cmd.flags & opencr_proto::FLAG_TORQUE_ON) != 0;

  if (want_torque && !g_torque_on) {
    // 토크 ON 순간: 팔 목표 = 현재 위치, 바퀴 = 0 (튀지 않게)
    holdArmAtPresent();
    if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
      for (int i = 0; i < WHEEL_COUNT; ++i) {g_wheel_goal_raw[i] = 0;}
    }
    if (g_config_error) {return;}   // 부팅 점검 실패면 토크를 켜지 않음
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_BROADCAST_ID, 1);
#if BUS_WATCHDOG_UNIT > 0
    // 100 Hz SyncWrite 가 계속 Watchdog 을 초기화 — OpenCR 이 멈추면 500 ms 뒤 모터가 스스로 정지
    for (int i = 0; i < N_MOTORS; ++i) {dxl.writeControlTableItem(BUS_WATCHDOG, g_ids[i], BUS_WATCHDOG_UNIT);}
#endif
    g_torque_on = true;
    return;   // 이번 주기는 현재 위치 유지
  }
  if (!want_torque && g_torque_on) {
    for (int i = 0; i < N_MOTORS; ++i) {dxl.writeControlTableItem(BUS_WATCHDOG, g_ids[i], 0);}
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_BROADCAST_ID, 0);
    g_torque_on = false;
  }
  if (!g_torque_on || g_safe_hold) {return;}

  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      g_wheel_goal_raw[i] = wheelRadsToRaw(g_cmd.wheel_vel[i], WHEEL_DIR[i]);
    }
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {
      const float q = clampf(g_cmd.arm_pos[i], ARM_MIN_RAD[i], ARM_MAX_RAD[i]);
      g_arm_goal_raw[i] = radToTick(q, ARM_DIR[i]);
    }
  }
#if GRIPPER_ID != 0
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {
  if (g_cmd.flags & opencr_proto::FLAG_GRIPPER_VALID) {
    g_gripper_goal_raw = radToTick(g_cmd.gripper_pos * GRIPPER_RAD_PER_UNIT, 1);
  }
  }
#endif
}

// ---- 안전 ----
static void applySafety()
{
  const bool timeout = !g_have_cmd || (millis() - g_last_cmd_ms > CMD_TIMEOUT_MS);
  const bool host_estop = g_have_cmd && (g_cmd.flags & opencr_proto::FLAG_ESTOP);
  bool hw_estop = false;
#if ESTOP_PIN_ENABLE
  hw_estop = digitalRead(ESTOP_PIN) == LOW;
#endif
  if (timeout) {g_error |= opencr_proto::ERR_CMD_TIMEOUT;} else {g_error &= ~opencr_proto::ERR_CMD_TIMEOUT;}
  if (hw_estop) {g_error |= opencr_proto::ERR_HW_ESTOP;} else {g_error &= ~opencr_proto::ERR_HW_ESTOP;}

  const bool stop = timeout || host_estop || hw_estop;
  if (stop) {
    if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
      for (int i = 0; i < WHEEL_COUNT; ++i) {g_wheel_goal_raw[i] = 0;}
    }
    // 무수신: 팔은 마지막 목표 유지. ESTOP: 현재 자세에서 멈춤. 토크는 끄지 않는다 (카트리지 낙하 방지)
    if ((host_estop || hw_estop) && !g_safe_hold) {holdArmAtPresent();}
    g_safe_hold = true;
  } else {
    g_safe_hold = false;
  }
}

static void writeGoals()
{
  if (!g_torque_on) {return;}
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    g_sw_wheel.is_info_changed = true;
    dxl.syncWrite(&g_sw_wheel);
  } else {
    g_sw_arm.is_info_changed = true;
    dxl.syncWrite(&g_sw_arm);
  }
#if GRIPPER_ID != 0
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {dxl.writeControlTableItem(GOAL_POSITION, GRIPPER_ID, g_gripper_goal_raw);}
#endif
}

// ---- 상태 송신 ----
static void sendState()
{
  State s;
  memset(&s, 0, sizeof(s));
  s.role = BOARD_ROLE;
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      const PresentData & p = g_present[i];
      s.pos[i] = tickToRad(p.position, WHEEL_DIR[i]) * WHEEL_GEAR;
      s.vel[i] = WHEEL_DIR[i] * p.velocity * RADS_PER_VEL_RAW * WHEEL_GEAR;
      s.effort[i] = WHEEL_DIR[i] * p.current * AMP_PER_CUR_RAW;
    }
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {
      const PresentData & p = g_present[i];
      const int k = opencr_proto::MAX_WHEELS + i;
      s.pos[k] = tickToRad(p.position, ARM_DIR[i]);
      s.vel[k] = ARM_DIR[i] * p.velocity * RADS_PER_VEL_RAW;
      s.effort[k] = ARM_DIR[i] * p.current * AMP_PER_CUR_RAW;
    }
  }
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    s.quat[0] = imu.quat[0];
    s.quat[1] = imu.quat[1];
    s.quat[2] = imu.quat[2];
    s.quat[3] = imu.quat[3];
    for (int i = 0; i < 3; ++i) {
      s.gyro[i] = imu.SEN.gyroADC[i] * GYRO_FACTOR;
      s.accel[i] = imu.SEN.accADC[i] * ACCEL_FACTOR;
    }
    s.battery_v = g_battery_v;
  }
  if (g_config_error) {g_error |= opencr_proto::ERR_MOTOR_CONFIG;}
  s.error = g_error;
  s.fw_version = opencr_proto::PROTOCOL_VERSION;
  s.seq = g_state_seq++;
  uint8_t frame[opencr_proto::MAX_FRAME];
  const size_t n = opencr_proto::encode_state(s, frame);
  HOST_SERIAL.write(frame, n);
}

void setup()
{
  HOST_SERIAL.begin(115200);   // USB CDC — 보드레이트 무관
#if ESTOP_PIN_ENABLE
  pinMode(ESTOP_PIN, INPUT_PULLUP);
#endif
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {g_ids[i] = WHEEL_IDS[i]; g_wheel_goal_raw[i] = 0;}
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {g_ids[i] = ARM_IDS[i];}
  }

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {setupMotor(WHEEL_IDS[i], OP_VELOCITY, false, WHEEL_MODEL);}
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {setupMotor(ARM_IDS[i], OP_POSITION, true, ARM_MODEL);}
  }
#if GRIPPER_ID != 0
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {setupMotor(GRIPPER_ID, OP_POSITION, true, ARM_MODEL);}
#endif
  setupSync();
  readState();
  holdArmAtPresent();

  imu.begin();
}

void loop()
{
  static uint32_t last_us = micros();
  receiveHost();
  imu.update();
  const uint32_t now = micros();
  if ((uint32_t)(now - last_us) >= CONTROL_PERIOD_US) {
    last_us += CONTROL_PERIOD_US;
    if ((uint32_t)(now - last_us) > 5 * CONTROL_PERIOD_US) {last_us = now;}   // 크게 밀리면 따라잡지 않음
    readState();
    const float vin = getPowerInVoltage();
    g_battery_v = (g_battery_v <= 0.f) ? vin : 0.9f * g_battery_v + 0.1f * vin;
    handleCmd();
    applySafety();
    writeGoals();
    sendState();
  }
}
