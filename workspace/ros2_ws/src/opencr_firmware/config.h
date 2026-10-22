// opencr_firmware 설정 (Firmware 팀) — ID · 주소 · 단위 · 안전값
// ★ = 실물 확인 후 맞출 값
#ifndef OPENCR_FIRMWARE_CONFIG_H_
#define OPENCR_FIRMWARE_CONFIG_H_

#include <stdint.h>

// 보드별 펌웨어 빌드: 1=바퀴 보드, 2=팔 보드.
// Arduino 빌드 설정에서 OPENCR_BOARD_ROLE=2 로 지정해 팔 보드 이미지를 별도로 만든다.
#ifndef OPENCR_BOARD_ROLE
#define OPENCR_BOARD_ROLE 1
#endif
#if OPENCR_BOARD_ROLE != 1 && OPENCR_BOARD_ROLE != 2
#error "OPENCR_BOARD_ROLE must be 1 (wheel) or 2 (arm)"
#endif

// ---- 통신 ----
#define DXL_SERIAL        Serial3          // OpenCR RS-485 (Dynamixel 포트)
#define DXL_DIR_PIN       84               // OpenCR DXL 방향 핀
#define DXL_BAUD          1000000          // Dynamixel Wizard 2.0 으로 미리 1 Mbps 설정
#define HOST_SERIAL       Serial           // USB CDC (/dev/ttyACM* 후보 중 역할 응답으로 식별)

#define CONTROL_PERIOD_US 10000            // 100 Hz
#define CMD_TIMEOUT_MS    200              // 이 시간 동안 CMD 없으면 바퀴 0 · 팔 위치 유지
#define HW_ERR_POLL_MS    50               // 모터 하나씩 돌아가며 Hardware Error Status 확인
#define BUS_WATCHDOG_UNIT 25               // Bus Watchdog 25 × 20 ms = 500 ms — OpenCR 루프가 멈추면 모터가 스스로 정지
                                           // (Xmen opencr_control.ino 와 같은 값, 0 이면 끔)

// ---- 바퀴 (Velocity 모드) ----
// 슬롯 순서 = URDF opencr.ros2_control.xacro: 왼쪽 앞 → 뒤, 오른쪽 앞 → 뒤
#define WHEEL_COUNT       6                // 실제 구성: 6개 바퀴 모터 (ID 1~6)
static const uint8_t WHEEL_IDS[6]   = {1, 2, 3, 4, 5, 6};
static const int8_t  WHEEL_DIR[6]   = {+1, +1, +1, -1, -1, -1}; // 슬롯 방향: 실물 회전 확인 후 보정 필수
static const float   WHEEL_GEAR     = 1.5f;   // 바퀴 각속도 / 모터 각속도 (증속벨트 36T→24T). 직결이면 1.0
static const int32_t WHEEL_VEL_LIMIT_RAW = 330;   // Goal Velocity 한계 (0.229 rpm 단위, XM430-W210 12 V 무부하 ≈ 77 rpm) ★
static const uint16_t WHEEL_MODEL = 1030;  // 부팅 점검: 바퀴 XM430-W210 (= 1030, 확정 2026-10-22). W350 이면 1020

// ---- 팔 (Position 모드) ----
#define ARM_COUNT         6                // joint1~5 + gripper_roll
static const uint8_t ARM_IDS[ARM_COUNT]     = {11, 12, 13, 14, 15, 16};
static const float   ARM_MIN_RAD[ARM_COUNT] = {-2.8f, -1.8f, -1.6f, -1.8f, -2.8f, -2.8f};   // URDF 와 같게 ★
static const float   ARM_MAX_RAD[ARM_COUNT] = { 2.8f,  1.6f,  1.5f,  2.0f,  2.8f,  2.8f};
static const int8_t  ARM_DIR[ARM_COUNT]     = {+1, +1, +1, +1, +1, +1};                    // ★
static const uint16_t ARM_MODEL = 1030;    // XM430-W210

// ---- 그리퍼 개폐 (미정) ----
#define GRIPPER_ID        0                // 0 = 없음. 정해지면 ID 와 아래 변환을 맞춤 ★
static const float   GRIPPER_RAD_PER_UNIT = 1.0f;

// ---- 물리 E-stop (권장) ----
#define ESTOP_PIN_ENABLE  1                // 물리 E-stop 필수 (배선 전에는 핀이 풀업 → 해제로 읽힘)
#define ESTOP_PIN         50               // OpenCR GPIO ★ (INPUT_PULLUP, 눌리면 LOW)

// ---- 배터리 ----
// OpenCR 코어의 getPowerInVoltage() (전원 입력 단자 분압 ADC) 사용

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

#endif  // OPENCR_FIRMWARE_CONFIG_H_
