// opencr_firmware 설정 (Firmware 팀) — ID · 주소 · 단위 · 안전값
// ★ = 실물 확인 후 맞출 값
//
// 펌웨어 하나를 역할별로 따로 빌드해 두 OpenCR 에 올린다 (README.md):
//   OPENCR_BOARD_ROLE=1  주행 보드 — 바퀴 Dynamixel ID 1~4, Velocity Mode, IMU · 배터리   → 아래 "주행 보드 전용"
//   OPENCR_BOARD_ROLE=2  팔 보드   — 팔 Dynamixel ID 11~16, Position Mode              → 아래 "팔 보드 전용"
//   역할은 빌드 옵션으로만 정한다 (기본값 없음, opencr_firmware.ino 가 검사). tools/build_firmware.sh wheel|arm
#ifndef OPENCR_FIRMWARE_CONFIG_H_
#define OPENCR_FIRMWARE_CONFIG_H_

#include <stdint.h>

// ======================================================================== 공통
// ---- 통신 ----
#define DXL_SERIAL        Serial3          // OpenCR RS-485 (Dynamixel 포트)
#define DXL_DIR_PIN       84               // OpenCR DXL 방향 핀
#define DXL_BAUD          1000000          // 모터를 미리 1 Mbps 로 설정 (tools/dxl_setup)
#define HOST_SERIAL       Serial           // USB CDC (/dev/ttyACM* 후보 중 역할 응답으로 식별)

#define CONTROL_PERIOD_US 10000            // 100 Hz
#define CMD_TIMEOUT_MS    200              // 이 시간 동안 CMD 없으면 바퀴 0 · 팔 위치 유지
#define HW_ERR_POLL_MS    50               // 모터 하나씩 돌아가며 Hardware Error Status 확인
#define BUS_WATCHDOG_UNIT 25               // Bus Watchdog 25 × 20 ms = 500 ms — OpenCR 루프가 멈추면 모터가 스스로 정지
                                           // (Xmen opencr_control.ino 와 같은 값, 0 이면 끔)

// ---- 물리 E-stop (권장) ----
#define ESTOP_PIN_ENABLE  1                // 물리 E-stop 필수 (배선 전에는 핀이 풀업 → 해제로 읽힘)
#define ESTOP_PIN         50               // OpenCR GPIO ★ (INPUT_PULLUP, 눌리면 LOW)

// ---- Control Table (XM430, 프로토콜 2.0) ----
#define ADDR_OPERATING_MODE   11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_HW_ERROR_STATUS  70
#define ADDR_BUS_WATCHDOG     98
#define ADDR_PROFILE_ACCEL    108
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_VELOCITY    104
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_START    126              // Present Current(2) · Velocity(4) · Position(4)
#define LEN_PRESENT           10

// ---- 단위 ----
static const float RAD_PER_TICK     = 6.28318530718f / 4096.0f;
static const int32_t TICK_CENTER    = 2048;
static const float RADS_PER_VEL_RAW = 0.229f * 6.28318530718f / 60.0f;   // 0.229 rpm
static const float AMP_PER_CUR_RAW  = 0.00269f;                         // XM430: 2.69 mA
static const float GYRO_FACTOR      = 0.0010642f;     // ±2000 dps → rad/s (TurtleBot3 와 같음)
static const float ACCEL_FACTOR     = 0.000598550415f; // ±2 g → m/s^2

// ======================================================================== 주행 보드 전용 (역할 1)
// 바퀴 4개 (2026-10-26: 6륜에서 가운데 2개를 뺌). 패킷의 바퀴 칸은 6개 그대로 — 칸 4 · 5 는 쓰지 않고 STATE 는 0.
// 칸 순서 = URDF opencr.ros2_control.xacro (wheel_fl/rl/fr/rr_joint): 0 왼쪽 앞 · 1 왼쪽 뒤 · 2 오른쪽 앞 · 3 오른쪽 뒤
#define WHEEL_COUNT       4
static const uint8_t WHEEL_IDS[WHEEL_COUNT] = {1, 3, 2, 4};      // 칸 순서대로 그 자리의 모터 ID. 실물 장착 (2026-10-27): 왼앞 1 · 오앞 2 · 왼뒤 3 · 오뒤 4
                                                                 //   (spin 시험에서 ID 2 · 3 만 거꾸로 → 자리 맞바꿈 확인) ★
static const int8_t  WHEEL_DIR[WHEEL_COUNT] = {+1, +1, -1, -1};  // 왼쪽 +1 · 오른쪽 -1 (모터가 마주 보게 장착 가정) — 실물 회전 확인 필수 ★
//   URDF (urdfv2 teamrobot.urdf) 바퀴 축은 4개 모두 +y → + = 앞. 오른쪽 반전은 여기서만 한다.
//   모터 Drive Mode 의 반전 비트는 0 이어야 함 (둘 다 반전하면 다시 거꾸로) — 부팅 때 prepareWheelConfig 가 0 으로 맞춤
static const float   WHEEL_GEAR     = 1.0f;   // 바퀴 각속도 / 모터 각속도. 직결 (2026-10-26 실물 확인, urdfv2 와 같음)
static const int32_t WHEEL_VEL_LIMIT_RAW = 330;   // Goal Velocity 한계 (0.229 rpm 단위) = 75.6 rpm = 바퀴 7.91 rad/s ≈ 0.261 m/s
                                                  // (URDF velocity 8.06 rad/s = XM430-W210 12 V 무부하 77 rpm 안쪽) ★
                                                  // 모터 EEPROM Velocity Limit (주소 44) 은 이 값 이상이어야 함 — 작으면 부팅 때 이 값으로 올림
static const uint16_t WHEEL_MODEL = 1030;  // 부팅 점검: 바퀴 XM430-W210 (= 1030, 확정 2026-10-22). W350 이면 1020

// 배터리: OpenCR 코어의 getPowerInVoltage() (전원 입력 단자 분압 ADC). IMU: OpenCR 내장 (cIMU)

// ======================================================================== 팔 보드 전용 (역할 2)
#define ARM_COUNT         6                // 패킷의 팔 칸: joint1~5 + 6 번째 칸
#define ARM_TORQUE_OFF_CMDS 50             // 팔 토크 OFF: TORQUE_ON 없는 정상 CMD 연속 수 + HOLD (100 Hz 면 0.5 s, D5-4 임시값) ★
// 벤치 임시 매핑 (2026-10-26 실물 확인): joint4 = ID 16, joint5 (손목 롤) = ID 14, 6 번째 칸 = ID 15.
//   실물의 6 번째 모터는 gripper_roll 이 아니라 그리퍼 개폐. URDF · manipulator.yaml 은 gripper_roll 그대로 — 팀 확인 전까지 벤치용 ★
//   6 번째 칸 범위는 결합을 다시 맞춘 뒤의 측정 (닫힘 1440 = -0.93 rad, 열림 2945 = +1.38 rad) 기준.
//   그 뒤 결합이 바뀌어 지금 실물과 맞지 않음 (닫힘 611 · 열림 -1293, 0/4095 경계를 지남) → 다시 결합 · 측정 필요 ★
static const uint8_t ARM_IDS[ARM_COUNT]     = {11, 12, 13, 16, 14, 15};
//   2026-10-27 (arm_bench, 사용자 "펌웨어 범위를 수정하자"): 전원 끌 때 접어 두는 대기 자세 (J2 691 · J3 3057 · J4 835 · 그리퍼 603 tick) 에서
//   바로 시작하도록 J2 최소 -1.8 → -2.14 · J3 최대 1.5 → 1.60 · J4 최소 -1.8 → -1.92 (A안, 여유 약 3°),
//   6 번째 칸 (그리퍼): 2026-10-27 다시 결합 → 실측 (사용자) 끝까지 열림 1550 · 완전히 닫힘 2650 tick (닫을수록 tick 증가, 이동량 1100).
//   범위 1530 ~ 2670 tick = -0.7946 ~ 0.9541 rad (양 끝 20 tick 여유 - 닫힘 읽는 값이 조금씩 흔들려도 시작 검사 통과).
//   슬라이더 명령은 monitor/gripper_map.py 로 1550 ~ 2650 안에서만 나감. 모터 한계도 1530 ~ 2670 (dxl_setup m 15)
//   2026-10-27 사용자 "J1 범위를 -180 ~ +180 으로": J1 ±2.8 → ±3.14159 (모터 한계 0 ~ 4095 = 한 바퀴 전체). 배선 꼬임 · 차체 간섭은 확인 필요 ★
static const float   ARM_MIN_RAD[ARM_COUNT] = {-3.14159f, -2.14f, -1.6f, -1.92f, -2.8f, -0.7946f};   // URDF 와 같게 ★
static const float   ARM_MAX_RAD[ARM_COUNT] = { 3.14159f,  1.6f,  1.60f,  2.0f,  2.8f,  0.9541f};
//   모터 EEPROM Min/Max Position Limit 도 이 범위와 같게 (부팅 점검이 확인, tools/dxl_setup 의 m 명령)
static const int8_t  ARM_DIR[ARM_COUNT]     = {+1, +1, +1, +1, +1, +1};   // 2026-10-26 J1~J5 손으로 확인 (URDF + 방향과 같음)
static const uint16_t ARM_MODEL = 1020;    // XM430-W350 (2026-10-26 실물 확인: 팔 모터 6대 모두 W350). W210 이면 1030
// 위치 제어 게인 (모터 RAM — 모터 전원이 켜질 때마다 공장값으로 돌아가므로 부팅 때마다 쓰고 다시 읽어 확인. 0~16383).
//   e-Manual: KPP = P/128, KPI = I/65536, KPD = D/16. 공장값 P 800 · I 0 · D 0 에서는 중력만큼 처진 채 버티고 (J2 0.026 rad / 0.33 A),
//   목표를 현재 위치로 다시 잡을 때마다 더 내려간다 (docs/arm_bench_test_2026-10-26.md 4-2). 실물 튜닝으로 정함 ★
//   I 게인은 막혔을 때 미는 힘이 계속 커진다 (과부하 → 모터가 토크를 끄면 팔이 떨어짐). 6 번째 칸 (그리퍼 개폐) 은 I = 0 고정.
static const uint16_t ARM_POS_P_GAIN[ARM_COUNT] = {800, 800, 800, 800, 800, 800};
static const uint16_t ARM_POS_I_GAIN[ARM_COUNT] = {  0,   0,   0,   0,   0,   0};
static const uint16_t ARM_POS_D_GAIN[ARM_COUNT] = {  0,   0,   0,   0,   0,   0};

// 그리퍼 (6 번째 칸) 막힘 보호 (2026-10-27 arm_bench): 퍽을 쥐다가 MOTOR_HW (과부하 추정, 막힌 채 PWM 885 로 1.5 s) →
//   모터 EEPROM PWM Limit 400 (dxl_setup w 15 400) + 펌웨어가 막힘을 보고 약하게 쥔다 (safety_core.h GripGuard).
//   막힘 = 목표가 현재보다 GRIP_STALL_TICKS 넘게 앞서 있는데 GRIP_STALL_MS 동안 GRIP_PROGRESS_TICKS 도 못 움직임
//   → 목표 = 현재 + GRIP_SQUEEZE_TICKS (목표 쪽) 로 고정 (쥐기). 호스트 목표가 그 자리보다 덜 가는 쪽 (보통 열기) 이면 풀고 따른다.
//   ESTOP · 토크 OFF 요청 HOLD 에서도 그리퍼 목표가 현재에서 GRIP_STALL_TICKS 안이면 (쥐는 중) 그대로 둔다 (퍽을 놓치지 않음).
//   그리퍼 위치 P 800 기준 1 tick ≈ PWM 6.25: 쥐는 힘 33 tick ≈ PWM 206 (23 %), 막힘 문턱 45 tick ≈ 281 (32 %) — 오래 미는 힘의 상한.
#define GRIP_SLOT 5                                 // 그리퍼 개폐 = 팔 6 번째 칸 (ID 15)
static const int32_t  GRIP_STALL_TICKS    = 45;     // 이보다 큰 목표 - 현재 가 이어지면 막힘 후보 ★
static const int32_t  GRIP_PROGRESS_TICKS = 8;      // GRIP_STALL_MS 안에 이만큼도 못 움직이면 막힘 (빈 손 이동은 수십 tick 씩 움직임)
static const uint32_t GRIP_STALL_MS       = 300;    // 막힘 판정 시간 ★
static const int32_t  GRIP_SQUEEZE_TICKS  = 33;     // 막힌 자리에서 더 닫는 양 = 쥐는 힘 (슬라이더 기본값과 같음, 사용자 실측으로 확정 ★)
static const int32_t  GRIP_PWM_LIMIT_MAX  = 400;    // 부팅 점검: 그리퍼 모터 EEPROM PWM Limit (공장값 885) 가 이보다 크면 FAULT
static_assert(GRIP_SQUEEZE_TICKS < GRIP_STALL_TICKS, "쥐는 중 (오차 = 쥐는 힘) 을 다시 막힘으로 보지 않게");
static_assert(GRIP_SLOT < ARM_COUNT, "그리퍼 칸은 팔 칸 안");

// 그리퍼 개폐 전용 경로 (미정). 실물 개폐 모터는 지금 6 번째 칸 (위) 으로 다룸
#define GRIPPER_ID        0                // 0 = 없음. 정해지면 ID 와 아래 변환을 맞춤 ★
static const float   GRIPPER_RAD_PER_UNIT = 1.0f;

#endif  // OPENCR_FIRMWARE_CONFIG_H_
