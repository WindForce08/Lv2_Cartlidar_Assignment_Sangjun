// opencr_firmware.ino — OpenCR 보드 (Firmware 팀)
// Jetson(opencr_hardware) ↔ USB CDC ↔ 역할별 OpenCR ↔ RS-485 1 Mbps ↔ 해당 Dynamixel
// 같은 소스를 역할별로 따로 빌드한다 (README.md, tools/build_firmware.sh wheel|arm):
//   OPENCR_BOARD_ROLE=1  주행 보드: 바퀴 ID 1~4 / Velocity Mode / IMU · 배터리
//   OPENCR_BOARD_ROLE=2  팔 보드:   팔 ID 11~16 / Position Mode
//
// loop (100 Hz):
//   receiveHost()  헤더 · 길이 · CRC 검증 후 CMD 를 safety_core 에 넘김 (역할 · 유한값 · 팔 범위 · seq 검사)
//   readState()    SyncRead addr 126 부터 10 B, 응답한 모터 값만 반영. 응답 부족 시 오류 비트
//   applySafety()  200 ms 무수신 · ESTOP 비트 · 물리 스위치 → 바퀴 0, 팔 HOLD (safety_core.h)
//   handleCmd()    HOLD 가 아니면 승인된 명령을 rad/s 또는 rad → 모터 목표로 변환
//   writeGoals()   역할에 해당하는 SyncWrite만 수행
//   sendState()    상태 + IMU + 배터리 전압 + 오류 · HOLD · FAULT 비트
//
// 부팅 점검 (Xmen opencr_control.ino prepareAxis 참고): ping · 모델 번호 · Homing Offset 0 · Return Delay 0
//   · (팔) 모터 Min/Max Position Limit 가 config 범위를 덮는지 · (바퀴) Drive Mode 반전 0 · Velocity Limit ≥ WHEEL_VEL_LIMIT_RAW (다르면 맞춰 씀, prepareWheelConfig). 하나라도 어긋나면 ERR_MOTOR_CONFIG · ERR_FAULT → 호스트가 활성화를 거부. 토크 ON 때 Bus Watchdog 500 ms 를 켬.
// 필요 라이브러리: OpenCR 보드 패키지 (Arduino IDE), Dynamixel2Arduino ≥ 0.6, OpenCR IMU (보드 패키지 내장)

// 역할은 빌드 옵션으로만 정한다 (기본값 없음 — 팔 보드에 주행 펌웨어가 잘못 올라가는 것을 막음)
#ifndef OPENCR_BOARD_ROLE
#error "OPENCR_BOARD_ROLE 을 지정하세요: 주행 보드 = 1, 팔 보드 = 2 (tools/build_firmware.sh wheel|arm, README.md 참고)"
#endif
#if OPENCR_BOARD_ROLE != 1 && OPENCR_BOARD_ROLE != 2
#error "OPENCR_BOARD_ROLE 은 1 (주행 보드) 또는 2 (팔 보드)"
#endif

#include <Dynamixel2Arduino.h>
#include <IMU.h>

#include "config.h"
#include "opencr_protocol.h"
#include "safety_core.h"

using namespace ControlTableItem;
using opencr_proto::Cmd;
using opencr_proto::State;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
cIMU imu;

// ---- 상태 ----
static const uint8_t BOARD_ROLE = OPENCR_BOARD_ROLE;
static const int N_MOTORS = BOARD_ROLE == opencr_proto::ROLE_WHEEL ? WHEEL_COUNT : ARM_COUNT;
static_assert(WHEEL_COUNT <= opencr_proto::MAX_WHEELS, "바퀴 수가 패킷 바퀴 칸보다 많음");   // 남는 칸은 STATE 0
static_assert(ARM_COUNT == opencr_proto::ARM_JOINTS, "팔 칸 수는 패킷과 같아야 함");
static uint8_t g_ids[N_MOTORS];

struct __attribute__((packed)) PresentData { int16_t current; int32_t velocity; int32_t position; };
static PresentData g_present[N_MOTORS];   // 마지막으로 응답한 값
static PresentData g_rx[N_MOTORS];        // SyncRead 수신 칸 — 응답이 끊긴 칸에는 다른 모터 값 · 일부만 받은 값이 들어갈 수 있음
static bool g_positions_valid = false;    // 이번 SyncRead 에서 모든 모터가 응답

static int32_t g_wheel_goal_raw[WHEEL_COUNT];
static int32_t g_arm_goal_raw[ARM_COUNT];
static int32_t g_gripper_goal_raw = 0;

static opencr_safety::SafetyCore g_safety(BOARD_ROLE, ARM_MIN_RAD, ARM_MAX_RAD, CMD_TIMEOUT_MS, ARM_TORQUE_OFF_CMDS);
static bool g_torque_on = false;
static uint8_t g_error = 0;          // SyncRead · 물리 E-stop · 모터 HW · 모터 설정 (무수신 · 역할 · HOLD · FAULT 는 g_safety)
static uint8_t g_state_seq = 0;
static uint8_t g_hw_error_mask = 0;  // 모터별 HW 오류 비트 (k 번째 모터 = 1 << k), STATE 에는 비트 1개로 보고
static float g_battery_v = 0.f;      // 저역 통과한 전원 전압
static bool g_config_error = false;  // 부팅 점검 실패 (래치 — 재부팅으로만 지움)

static opencr_proto::FrameParser g_parser;
static const int RX_BYTES_PER_CALL = 4 * (int)opencr_proto::MAX_FRAME;   // receiveHost 한 번에 읽는 최대 바이트

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
// 반환: 토크를 켜 둔 채로 두었는가 (D5-1, 팔만)
static bool setupMotor(uint8_t id, uint8_t mode, bool is_arm, uint16_t model)
{
  if (!dxl.ping(id)) {g_config_error = true; return false;}   // 응답 없음
  if (dxl.getModelNumber(id) != model) {g_config_error = true;}
  // 팔: OpenCR 만 재부팅되고 모터가 토크 ON 으로 버티는 중이면 끄지 않는다 (처짐 방지).
  //   바퀴는 저장된 Goal Velocity 로 다시 돌 수 있어 기존대로 끈다.
  const bool keep = is_arm && dxl.readControlTableItem(TORQUE_ENABLE, id) == 1;
  if (!keep) {dxl.torqueOff(id);}
  // 각도 변환은 Homing Offset 0 을 가정 — 0 이 아니면 위치가 어긋나므로 점검 실패로 알림 (값은 바꾸지 않음)
  if (dxl.readControlTableItem(HOMING_OFFSET, id) != 0) {g_config_error = true;}
  dxl.writeControlTableItem(BUS_WATCHDOG, id, 0);        // 이전 Watchdog 오류 해제
  if (keep) {
    // 토크 ON 중에는 EEPROM (Return Delay · Operating Mode) 을 쓸 수 없어 읽어서 검사만 한다 (e-Manual 기준 ★ 실물 확인)
    if (dxl.readControlTableItem(RETURN_DELAY_TIME, id) != 0 || dxl.readControlTableItem(OPERATING_MODE, id) != mode) {
      g_config_error = true;
    }
  } else {
    dxl.writeControlTableItem(RETURN_DELAY_TIME, id, 0);   // SyncRead 응답 지연 최소
    dxl.setOperatingMode(id, mode);
  }
  if (is_arm) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 0);   // 궤적 보간은 arm_controller 가 함
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 0);
  }
  return keep;
}

// 팔: 모터 위치 한계 (EEPROM Min/Max Position Limit) 가 config 범위의 목표를 모두 받는지 (읽기만).
//   범위 밖 Goal Position 은 모터가 거부하고 SyncWrite 는 응답이 없어 펌웨어가 알 수 없다
//   (2026-10-26 실물: joint2 한계 2230~3033 → 영점 근처 목표가 모두 무시됨)
static void checkArmLimits(int i)
{
  const uint8_t id = ARM_IDS[i];
  if (!dxl.ping(id)) {return;}   // 응답 없음은 setupMotor 가 이미 알림
  const int32_t a = radToTick(ARM_MIN_RAD[i], ARM_DIR[i]);
  const int32_t b = radToTick(ARM_MAX_RAD[i], ARM_DIR[i]);
  const int32_t lo = dxl.readControlTableItem(MIN_POSITION_LIMIT, id);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) {g_config_error = true; return;}
  const int32_t hi = dxl.readControlTableItem(MAX_POSITION_LIMIT, id);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) {g_config_error = true; return;}
  if (lo > (a < b ? a : b) || hi < (a < b ? b : a)) {g_config_error = true;}
}

// 바퀴: 모터 EEPROM 설정을 펌웨어 가정에 맞춘다 (Return Delay · Operating Mode 를 setupMotor 가 쓰는 것과 같은 방식).
//   Drive Mode bit0 (Reverse) = 0: 방향 반전은 WHEEL_DIR 로만 한다. 모터에서도 반전하면 오른쪽이 거꾸로 돈다
//     → 켜져 있으면 bit0 만 끈다 (다른 비트는 그대로).
//   Velocity Limit ≥ WHEEL_VEL_LIMIT_RAW: 넘는 Goal Velocity 는 모터가 거부하고 SyncWrite 라 펌웨어가 알 수 없다
//     → 작으면 WHEEL_VEL_LIMIT_RAW 로 올린다 (크면 그대로).
//   EEPROM 은 토크 OFF 에서만 써진다 — 바퀴는 setupMotor 가 토크를 이미 껐다. 다를 때만 쓰고 (EEPROM 수명),
//   쓴 뒤 다시 읽어 맞지 않으면 (토크가 안 꺼짐 · 통신 실패) 부팅 점검 실패로 알린다.
static bool readWheelItem(uint8_t item, uint8_t id, int32_t & v)
{
  v = dxl.readControlTableItem(item, id);
  return dxl.getLastLibErrCode() == DXL_LIB_OK;
}

static void prepareWheelConfig(int i)
{
  const uint8_t id = WHEEL_IDS[i];
  if (!dxl.ping(id)) {return;}   // 응답 없음은 setupMotor 가 이미 알림
  int32_t drive_mode = 0;
  if (!readWheelItem(DRIVE_MODE, id, drive_mode)) {g_config_error = true; return;}
  if (drive_mode & 0x01) {
    dxl.writeControlTableItem(DRIVE_MODE, id, drive_mode & ~0x01);
    if (!readWheelItem(DRIVE_MODE, id, drive_mode) || (drive_mode & 0x01)) {g_config_error = true; return;}
  }
  int32_t vel_limit = 0;
  if (!readWheelItem(VELOCITY_LIMIT, id, vel_limit)) {g_config_error = true; return;}
  if (vel_limit < WHEEL_VEL_LIMIT_RAW) {
    dxl.writeControlTableItem(VELOCITY_LIMIT, id, WHEEL_VEL_LIMIT_RAW);
    if (!readWheelItem(VELOCITY_LIMIT, id, vel_limit) || vel_limit < WHEEL_VEL_LIMIT_RAW) {g_config_error = true; return;}
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
    g_sr_xels[i].p_recv_buf = (uint8_t *)&g_rx[i];
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
  // Dynamixel2Arduino 0.8 syncRead 는 ID 순서대로 받다가 처음 실패한 모터에서 멈춘다 → 앞의 recv 개만 유효
  const uint8_t recv = dxl.syncRead(&g_sr);
  for (int i = 0; i < recv && i < N_MOTORS; ++i) {g_present[i] = g_rx[i];}
  g_positions_valid = recv >= N_MOTORS;
  if (!g_positions_valid) {
    g_error |= opencr_proto::ERR_SYNCREAD;
  } else {
    g_error &= ~opencr_proto::ERR_SYNCREAD;
  }
  // HW 오류 상태: 한 주기에 모터 하나씩 확인. 읽기 실패도 0 을 돌려주므로 getLastLibErrCode 로 구분해 이전 비트를 유지.
  // 이번 SyncRead 에 응답하지 않은 모터는 건너뛴다 (응답 없는 읽기는 기본 100 ms 동안 루프를 막음)
  static uint32_t last_poll = 0;
  static int k = 0;
  if (millis() - last_poll >= HW_ERR_POLL_MS) {
    last_poll = millis();
    if (k < recv) {
      const int32_t st = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, g_ids[k]);
      if (dxl.getLastLibErrCode() == DXL_LIB_OK) {
        if (st > 0) {g_hw_error_mask |= (uint8_t)(1u << k);} else {g_hw_error_mask &= (uint8_t)~(1u << k);}
      }
    }
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
  for (int n = 0; n < RX_BYTES_PER_CALL && HOST_SERIAL.available() > 0; ++n) {
    const uint8_t b = (uint8_t)HOST_SERIAL.read();
    if (g_parser.push(b) && g_parser.type() == opencr_proto::TYPE_CMD) {
      Cmd c;
      if (opencr_proto::decode_cmd(g_parser.payload(), g_parser.length(), c)) {
        g_safety.on_cmd(c, millis());   // 거부한 CMD 는 적용하지 않고 무수신 타이머도 갱신하지 않음
      }
    }
  }
}

// ---- 명령 적용 ----
static void handleCmd()
{
  if (!g_safety.have_cmd()) {return;}
  const Cmd & cmd = g_safety.cmd();
  const bool want_torque = (cmd.flags & opencr_proto::FLAG_TORQUE_ON) != 0;

  if (want_torque && !g_torque_on && !g_safety.timeout()) {
    if (g_safety.fault()) {return;}   // 부팅 점검 실패면 토크를 켜지 않음
    // 팔은 토크 ON 목표가 현재 위치이므로, 모든 관절이 응답하지 않으면 켜지 않음
    if (BOARD_ROLE == opencr_proto::ROLE_ARM && !g_positions_valid) {return;}
    // 토크 ON 순간: 팔 목표 = 현재 위치 (REARM 까지 HOLD), 바퀴 = 0 (튀지 않게)
    holdArmAtPresent();
    if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
      for (int i = 0; i < WHEEL_COUNT; ++i) {g_wheel_goal_raw[i] = 0;}
    }
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_BROADCAST_ID, 1);
#if BUS_WATCHDOG_UNIT > 0
    // 100 Hz SyncWrite 가 계속 Watchdog 을 초기화 — OpenCR 이 멈추면 500 ms 뒤 모터가 스스로 정지
    for (int i = 0; i < N_MOTORS; ++i) {dxl.writeControlTableItem(BUS_WATCHDOG, g_ids[i], BUS_WATCHDOG_UNIT);}
#endif
    g_torque_on = true;
    g_safety.on_torque_on();
    return;   // 이번 주기는 현재 위치 유지
  }
  // 토크 OFF: 바퀴는 즉시, 팔은 HOLD 중 TORQUE_ON 없는 CMD 가 ARM_TORQUE_OFF_CMDS 번 연속일 때만 (D5-4)
  if (!want_torque && g_torque_on && g_safety.torque_off_allowed()) {
    for (int i = 0; i < N_MOTORS; ++i) {dxl.writeControlTableItem(BUS_WATCHDOG, g_ids[i], 0);}
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_BROADCAST_ID, 0);
    g_torque_on = false;
    g_safety.on_torque_off();
  }
  if (!g_torque_on || !g_safety.apply_targets()) {return;}

  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    // 한 바퀴라도 한계를 넘으면 모든 바퀴를 같은 비율로 줄여 좌우 속도비 (회전 반경) 를 유지한다.
    //   바퀴마다 따로 자르면 회전하며 달릴 때 바깥 바퀴만 잘려 명령보다 덜 돈다. 개별 clamp 는 안전망으로 남김
    const float lim = WHEEL_VEL_LIMIT_RAW * RADS_PER_VEL_RAW * WHEEL_GEAR;   // 바퀴 rad/s
    float peak = 0.f;
    for (int i = 0; i < WHEEL_COUNT; ++i) {peak = fmaxf(peak, fabsf(cmd.wheel_vel[i]));}
    const float k = (peak > lim) ? lim / peak : 1.f;
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      g_wheel_goal_raw[i] = wheelRadsToRaw(cmd.wheel_vel[i] * k, WHEEL_DIR[i]);
    }
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {
      const float q = clampf(cmd.arm_pos[i], ARM_MIN_RAD[i], ARM_MAX_RAD[i]);   // 범위 밖은 safety_core 가 이미 거부
      g_arm_goal_raw[i] = radToTick(q, ARM_DIR[i]);
    }
  }
#if GRIPPER_ID != 0
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {
  if (cmd.flags & opencr_proto::FLAG_GRIPPER_VALID) {
    g_gripper_goal_raw = radToTick(cmd.gripper_pos * GRIPPER_RAD_PER_UNIT, 1);
  }
  }
#endif
}

// ---- 안전 ----
static void applySafety()
{
  bool hw_estop = false;
#if ESTOP_PIN_ENABLE
  hw_estop = digitalRead(ESTOP_PIN) == LOW;
#endif
  if (hw_estop) {g_error |= opencr_proto::ERR_HW_ESTOP;} else {g_error &= ~opencr_proto::ERR_HW_ESTOP;}

  g_safety.tick(millis(), hw_estop, g_torque_on, g_positions_valid);
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL && (g_safety.timeout() || g_safety.estop())) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {g_wheel_goal_raw[i] = 0;}
  }
  // 무수신: 팔은 마지막 승인 목표 유지. ESTOP · 토크 OFF 요청: 현재 자세에서 멈춤 (위치를 모르면 마지막 목표 유지).
  // 토크는 끄지 않는다 (카트리지 낙하 방지). HOLD 는 REARM 상승 에지로만 풀린다
  if (g_safety.entered_stop_hold() && g_positions_valid) {holdArmAtPresent();}
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
  s.error = g_error | g_safety.status_bits();
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
  int torque_kept = 0;   // 토크 ON 으로 남겨 둔 팔 관절 수
  if (BOARD_ROLE == opencr_proto::ROLE_WHEEL) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      setupMotor(WHEEL_IDS[i], OP_VELOCITY, false, WHEEL_MODEL);
      prepareWheelConfig(i);
    }
  } else {
    for (int i = 0; i < ARM_COUNT; ++i) {
      torque_kept += setupMotor(ARM_IDS[i], OP_POSITION, true, ARM_MODEL) ? 1 : 0;
      checkArmLimits(i);
    }
  }
#if GRIPPER_ID != 0
  if (BOARD_ROLE == opencr_proto::ROLE_ARM) {setupMotor(GRIPPER_ID, OP_POSITION, true, ARM_MODEL);}
#endif
  setupSync();
  readState();
  holdArmAtPresent();
  // D5-1: 모든 팔 관절이 토크 ON 이고 위치를 읽었으면 토크 ON 상태로 이어간다 (현재 위치 유지, REARM 까지 HOLD).
  //   일부만 켜져 있거나 위치를 모르면 손대지 않고 FAULT — 켜져 있던 모터는 그대로 버틴다 (전원을 내려야 풀림)
  if (torque_kept > 0) {
    if (torque_kept == ARM_COUNT && g_positions_valid && !g_config_error) {
#if BUS_WATCHDOG_UNIT > 0
      for (int i = 0; i < N_MOTORS; ++i) {dxl.writeControlTableItem(BUS_WATCHDOG, g_ids[i], BUS_WATCHDOG_UNIT);}
#endif
      g_torque_on = true;
      g_safety.on_torque_on();
    } else {
      g_config_error = true;
    }
  }
  if (g_config_error) {g_safety.set_fault();}

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
    applySafety();   // 무수신 · ESTOP · HOLD 판단을 먼저 하고 명령을 적용
    handleCmd();
    writeGoals();
    sendState();
  }
}
