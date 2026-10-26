// SIL 시험: 실제 opencr_firmware.ino (바퀴 · 팔 두 이미지) 를 테스트 더블 위에서 돌린다 (sil/sil_board.hpp 참고).
//   펌웨어 = 실제 코드. Dynamixel 버스 · USB · 시계 = 테스트 더블 (실물과 같다고 보장하지 않음).
//   호스트 = 이 파일의 시험 드라이버 (프로토콜 헤더로 CMD 를 만들고 STATE 를 읽음) — 실제 OpenCRHardware 가 아니다.
//   시간은 가상 시계 (1 ms 단위) — 실제 OpenCR 의 루프 시간 · 100 Hz 를 검증하지 않는다.
//   펌웨어 static 상태가 시험 사이에 남지 않도록 각 시나리오를 fork 한 자식 프로세스에서 돌린다.
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "config.h"
#include "opencr_hardware/opencr_protocol.hpp"
#include "sil_board.hpp"

using namespace opencr_proto;

namespace
{

const float NaN = std::numeric_limits<float>::quiet_NaN();
const float Inf = std::numeric_limits<float>::infinity();

template<class F>
void isolated(F body)
{
  fflush(stdout);
  const pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    body();
    fflush(stdout);
    _exit(::testing::Test::HasFailure() ? 1 : 0);
  }
  int status = 0;
  waitpid(pid, &status, 0);
  ASSERT_TRUE(WIFEXITED(status)) << "자식 프로세스 비정상 종료";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "자식 프로세스의 실패 메시지 참고";
}

int32_t arm_tick(float rad) {return TICK_CENTER + static_cast<int32_t>(lroundf(rad / RAD_PER_TICK));}
int32_t wheel_raw(int slot, float wheel_rads)
{
  return static_cast<int32_t>(lroundf(WHEEL_DIR[slot] * (wheel_rads / WHEEL_GEAR) / RADS_PER_VEL_RAW));
}

// 시험용 호스트 (실제 OpenCRHardware 아님)
struct Host
{
  Host(SilBoard & board, uint8_t role_in)
  : b(board), role(role_in) {}

  Cmd make(uint8_t flags)
  {
    Cmd c{};
    c.role = role;
    c.flags = flags;
    for (int k = 0; k < ARM_JOINTS; ++k) {c.arm_pos[k] = arm[k];}
    for (int k = 0; k < MAX_WHEELS; ++k) {c.wheel_vel[k] = wheel[k];}
    c.seq = seq++;
    return c;
  }
  void send(const Cmd & c)
  {
    uint8_t f[MAX_FRAME];
    const size_t n = encode_cmd(c, f);
    b.host_rx.insert(b.host_rx.end(), f, f + n);
  }
  void send(uint8_t flags) {send(make(flags));}
  void send_bytes(const uint8_t * p, size_t n) {b.host_rx.insert(b.host_rx.end(), p, p + n);}
  void poll()
  {
    for (uint8_t x : b.host_tx) {
      if (parser.push(x) && parser.type() == TYPE_STATE) {
        State s;
        if (decode_state(parser.payload(), parser.length(), s)) {states.push_back(s);}
      }
    }
    b.host_tx.clear();
  }
  State last()
  {
    poll();
    return states.empty() ? State{} : states.back();
  }

  SilBoard & b;
  uint8_t role;
  uint8_t seq = 0;
  float arm[ARM_JOINTS] = {};
  float wheel[MAX_WHEELS] = {};
  FrameParser parser;
  std::vector<State> states;
};

const int NO_CMD = -1;

struct Bench
{
  Bench()
  : w(sil_wheel_board()), a(sil_arm_board()), hw(w, ROLE_WHEEL), ha(a, ROLE_ARM)
  {
    for (uint8_t id : WHEEL_IDS) {w.motors[id] = SilMotor{}; w.motors[id].model = WHEEL_MODEL;}
    for (uint8_t id : ARM_IDS) {a.motors[id] = SilMotor{}; a.motors[id].model = ARM_MODEL;}
  }
  void boot() {w.setup(); a.setup();}
  // 1 ms 씩 두 보드를 함께 돌림
  void run(int ms)
  {
    for (int i = 0; i < ms; ++i) {
      w.now_us += 1000;
      a.now_us += 1000;
      w.loop();
      a.loop();
    }
  }
  // 호스트가 10 ms 마다 각 보드에 CMD 하나 (NO_CMD 면 그 보드에는 보내지 않음)
  void drive(int ms, int wheel_flags, int arm_flags)
  {
    for (int t = 0; t < ms; t += 10) {
      if (wheel_flags != NO_CMD) {hw.send(static_cast<uint8_t>(wheel_flags));}
      if (arm_flags != NO_CMD) {ha.send(static_cast<uint8_t>(arm_flags));}
      run(10);
    }
    hw.poll();
    ha.poll();
  }
  // 팔: 토크 ON (HOLD) → REARM 펄스 → ACTIVE
  void arm_activate()
  {
    drive(50, NO_CMD, FLAG_TORQUE_ON);
    drive(30, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    drive(30, NO_CMD, FLAG_TORQUE_ON);
  }
  // 팔 슬롯 k 의 모터 (config.h ARM_IDS 순서 — ID 순서와 다를 수 있음)
  SilMotor & arm_motor(int k) {return a.motors[ARM_IDS[k]];}
  int32_t arm_goal(int k) {return arm_motor(k).goal_position;}
  // 바퀴 칸 k (0 ~ WHEEL_COUNT-1) 의 모터 (config.h WHEEL_IDS 순서)
  int32_t wheel_goal(int k) {return w.motors[WHEEL_IDS[k]].goal_velocity;}

  SilBoard & w;
  SilBoard & a;
  Host hw;
  Host ha;
};

}  // namespace

// 1. 정상 CMD/STATE 송수신 · 역할 · 버전 · 슬롯
TEST(FirmwareSil, NormalCmdState)
{
  isolated([] {
    Bench b;
    b.boot();
    b.run(1000);
    b.hw.poll();
    b.ha.poll();
    EXPECT_NEAR(static_cast<double>(b.hw.states.size()), 100.0, 1.0) << "가상 시계 기준 100 Hz STATE";
    EXPECT_NEAR(static_cast<double>(b.ha.states.size()), 100.0, 1.0);
    for (size_t i = 1; i < b.ha.states.size(); ++i) {
      EXPECT_EQ(static_cast<uint8_t>(b.ha.states[i].seq - b.ha.states[i - 1].seq), 1);
    }
    State sw = b.hw.last(), sa = b.ha.last();
    EXPECT_EQ(sw.role, ROLE_WHEEL);
    EXPECT_EQ(sa.role, ROLE_ARM);
    EXPECT_EQ(sw.fw_version, 5);
    EXPECT_EQ(sa.fw_version, 5);
    EXPECT_FLOAT_EQ(sw.quat[0], 1.f) << "IMU 는 바퀴 보드만";
    EXPECT_FLOAT_EQ(sa.quat[0], 0.f);
    EXPECT_TRUE(sa.error & ERR_CMD_TIMEOUT) << "CMD 전에는 타임아웃 상태";

    const int last = WHEEL_COUNT - 1;
    b.hw.wheel[0] = 2.0f;
    b.hw.wheel[last] = -1.0f;
    for (int k = WHEEL_COUNT; k < MAX_WHEELS; ++k) {b.hw.wheel[k] = 3.0f;}   // 바퀴 없는 칸: 값이 와도 쓰지 않음
    b.drive(100, FLAG_TORQUE_ON, NO_CMD);
    EXPECT_TRUE(b.w.motors[WHEEL_IDS[0]].torque);
    EXPECT_EQ(b.wheel_goal(0), wheel_raw(0, 2.0f));
    EXPECT_EQ(b.wheel_goal(last), wheel_raw(last, -1.0f));
    EXPECT_EQ(b.w.motors.size(), static_cast<size_t>(WHEEL_COUNT));
    sw = b.hw.last();
    EXPECT_NEAR(sw.vel[0], 2.0, 0.05);
    EXPECT_NEAR(sw.vel[last], -1.0, 0.05);
    for (int k = WHEEL_COUNT; k < MAX_WHEELS; ++k) {
      EXPECT_EQ(sw.pos[k], 0.f) << "바퀴 없는 칸 " << k << " 은 0";
      EXPECT_EQ(sw.vel[k], 0.f) << k;
      EXPECT_EQ(sw.effort[k], 0.f) << k;
    }
    EXPECT_FALSE(sw.error & ERR_CMD_TIMEOUT);

    b.arm_activate();
    b.ha.arm[0] = 0.5f;
    b.ha.arm[5] = -0.4f;
    b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.5f));
    EXPECT_EQ(b.arm_goal(5), arm_tick(-0.4f));
    sa = b.ha.last();
    EXPECT_NEAR(sa.pos[MAX_WHEELS + 0], 0.5, 0.002) << "팔은 슬롯 6~11";
    EXPECT_NEAR(sa.pos[MAX_WHEELS + 5], -0.4, 0.002);
    EXPECT_FALSE(sa.error & (ERR_HOLD | ERR_CMD_TIMEOUT));
  });
}

// 2. 프레임 CRC · 길이 · 타입 검증 — 잘못된 프레임은 타이머를 갱신하지 않음
TEST(FirmwareSil, RejectsBadCrcLengthAndType)
{
  isolated([] {
    Bench b;
    b.boot();
    for (int t = 0; t < 300; t += 10) {
      uint8_t f[MAX_FRAME];
      const size_t n = encode_cmd(b.ha.make(FLAG_TORQUE_ON), f);
      f[10] ^= 0x01;   // CRC 불일치
      b.ha.send_bytes(f, n);
      uint8_t v3[54] = {};   // v3 길이 (role 없음) — CRC 는 맞음
      const size_t n3 = encode_frame(TYPE_CMD, v3, sizeof(v3), f);
      b.ha.send_bytes(f, n3);
      State s{};
      s.role = ROLE_ARM;
      const size_t ns = encode_state(s, f);   // 잘못된 방향의 타입
      b.ha.send_bytes(f, ns);
      b.run(10);
    }
    State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_CMD_TIMEOUT) << "잘못된 프레임만 오면 계속 타임아웃";
    EXPECT_FALSE(b.a.motors[11].torque) << "잘못된 프레임으로 토크가 켜지면 안 됨";
    b.drive(30, NO_CMD, FLAG_TORQUE_ON);
    sa = b.ha.last();
    EXPECT_FALSE(sa.error & ERR_CMD_TIMEOUT) << "정상 프레임은 받음";
    EXPECT_TRUE(b.a.motors[11].torque);
  });
}

// 3. 잘못된 역할 · 범위 밖 명령
TEST(FirmwareSil, RejectsWrongRoleAndOutOfRange)
{
  isolated([] {
    Bench b;
    b.boot();
    Host wrong(b.a, ROLE_WHEEL);   // 팔 보드에 바퀴 역할 CMD
    for (int t = 0; t < 300; t += 10) {
      wrong.send(FLAG_TORQUE_ON | FLAG_REARM);
      b.run(10);
    }
    State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_ROLE_MISMATCH);
    EXPECT_TRUE(sa.error & ERR_CMD_TIMEOUT) << "다른 역할 CMD 는 타이머를 갱신하지 않음";
    EXPECT_FALSE(b.a.motors[11].torque);

    b.arm_activate();
    b.ha.arm[1] = 0.3f;
    b.drive(30, NO_CMD, FLAG_TORQUE_ON);
    ASSERT_EQ(b.arm_goal(1), arm_tick(0.3f));
    EXPECT_FALSE(b.ha.last().error & ERR_ROLE_MISMATCH) << "정상 CMD 가 오면 해제";
    b.ha.arm[1] = ARM_MAX_RAD[1] + 0.1f;
    b.drive(300, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(1), arm_tick(0.3f)) << "범위 밖 목표는 적용하지 않음";
    sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_HOLD) << "범위 밖 CMD 만 오면 타임아웃 → HOLD";

    // 바퀴 범위 초과는 D6 미결 — 현재 동작(clamp)을 기록
    b.hw.wheel[0] = 100.f;
    b.drive(50, FLAG_TORQUE_ON, NO_CMD);
    EXPECT_EQ(b.wheel_goal(0), WHEEL_DIR[0] * WHEEL_VEL_LIMIT_RAW);
  });
}

// 4. NaN · Inf 명령 거부
TEST(FirmwareSil, RejectsNanInf)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[2] = 0.2f;
    b.hw.wheel[1] = 1.0f;
    b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    ASSERT_EQ(b.arm_goal(2), arm_tick(0.2f));
    ASSERT_EQ(b.wheel_goal(1), wheel_raw(1, 1.0f));
    for (float bad : {NaN, Inf, -Inf}) {
      b.ha.arm[2] = bad;
      b.hw.wheel[1] = bad;
      b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
      EXPECT_EQ(b.arm_goal(2), arm_tick(0.2f)) << bad;
      EXPECT_EQ(b.wheel_goal(1), wheel_raw(1, 1.0f)) << "비유한 값은 모터로 가지 않음 " << bad;
    }
    b.drive(100, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    EXPECT_EQ(b.wheel_goal(1), 0) << "비유한 CMD 만 오면 타임아웃 → 바퀴 0";
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD);
  });
}

// 5. 팔 명령 타임아웃 → 마지막 승인 목표 유지 · 토크 유지 · SyncWrite 계속
TEST(FirmwareSil, ArmTimeoutHoldsLastTarget)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[3] = 0.7f;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    ASSERT_EQ(b.arm_goal(3), arm_tick(0.7f));
    const int writes_before = b.a.sync_writes;
    b.run(300);
    State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_CMD_TIMEOUT);
    EXPECT_TRUE(sa.error & ERR_HOLD);
    EXPECT_EQ(b.arm_goal(3), arm_tick(0.7f));
    for (uint8_t id = 11; id <= 16; ++id) {
      EXPECT_TRUE(b.a.motors[id].torque) << "타임아웃만으로 토크를 끄지 않음 " << int(id);
      EXPECT_EQ(b.a.motors[id].watchdog, BUS_WATCHDOG_UNIT);
    }
    EXPECT_GE(b.a.sync_writes - writes_before, 25) << "HOLD 중에도 SyncWrite 로 목표 · Watchdog 유지";
  });
}

// 6. 바퀴 명령 타임아웃 → 정지 명령 (0) 생성
TEST(FirmwareSil, WheelTimeoutWritesZero)
{
  isolated([] {
    Bench b;
    b.boot();
    for (int k = 0; k < MAX_WHEELS; ++k) {b.hw.wheel[k] = 3.0f;}
    b.drive(100, FLAG_TORQUE_ON, NO_CMD);
    ASSERT_NE(b.wheel_goal(0), 0);
    b.run(190);
    EXPECT_NE(b.wheel_goal(0), 0) << "200 ms 전에는 유지";
    b.run(30);
    for (int k = 0; k < WHEEL_COUNT; ++k) {EXPECT_EQ(b.wheel_goal(k), 0) << k;}
    EXPECT_TRUE(b.hw.last().error & ERR_CMD_TIMEOUT);
    EXPECT_TRUE(b.w.motors[1].torque);
  });
}

// 7. HOLD 유지 · FAULT 래치
TEST(FirmwareSil, HoldPersistsAndFaultLatched)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    for (int s = 0; s < 5; ++s) {
      b.run(1000);
      EXPECT_TRUE(b.ha.last().error & ERR_HOLD) << s << " s";
    }
  });
  isolated([] {
    Bench b;
    b.a.motors[13].model = static_cast<uint16_t>(ARM_MODEL + 1);   // 부팅 점검 실패 (config.h 와 다른 모델)
    b.boot();
    for (int t = 0; t < 1000; t += 100) {
      b.drive(50, NO_CMD, FLAG_TORQUE_ON);
      b.drive(50, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    }
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_FAULT);
    EXPECT_TRUE(sa.error & ERR_MOTOR_CONFIG);
    for (uint8_t id = 11; id <= 16; ++id) {EXPECT_FALSE(b.a.motors[id].torque) << int(id);}
  });
}

// 7b. 모터 위치 한계 (EEPROM) 가 config 범위보다 좁으면 부팅 점검 실패 — 범위 밖 목표를 모터가 응답 없이 버리기 때문
//     (2026-10-26 실물: joint2 한계 2230~3033 → 영점 근처 목표가 모두 무시됨)
TEST(FirmwareSil, ArmPositionLimitNarrowerThanConfigFaults)
{
  isolated([] {
    Bench b;
    b.arm_motor(1).min_position_limit = 2230;
    b.arm_motor(1).max_position_limit = 3033;
    b.boot();
    for (int t = 0; t < 1000; t += 100) {
      b.drive(50, NO_CMD, FLAG_TORQUE_ON);
      b.drive(50, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    }
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_FAULT);
    EXPECT_TRUE(sa.error & ERR_MOTOR_CONFIG);
    for (uint8_t id : ARM_IDS) {EXPECT_FALSE(b.a.motors[id].torque) << int(id);}
  });
  // config 범위와 꼭 같은 한계 (실물에 쓸 값) 는 통과하고, 범위 양 끝 목표를 모터가 받는다 (ARM_DIR 모두 +1 기준)
  isolated([] {
    Bench b;
    for (int k = 0; k < ARM_COUNT; ++k) {
      b.arm_motor(k).min_position_limit = arm_tick(ARM_MIN_RAD[k]);
      b.arm_motor(k).max_position_limit = arm_tick(ARM_MAX_RAD[k]);
    }
    b.boot();
    b.arm_activate();
    EXPECT_FALSE(b.ha.last().error & (ERR_FAULT | ERR_MOTOR_CONFIG));
    b.ha.arm[1] = ARM_MIN_RAD[1];
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(1), arm_tick(ARM_MIN_RAD[1]));
    b.ha.arm[1] = ARM_MAX_RAD[1];
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(1), arm_tick(ARM_MAX_RAD[1]));
  });
}

// 7c. 위치 게인 (RAM): 부팅 때 팔 모터마다 config 값을 쓰고 다시 읽어 확인. 바퀴 보드는 쓰지 않음
TEST(FirmwareSil, ArmGainsWrittenAtBoot)
{
  for (int k = 0; k < ARM_COUNT; ++k) {
    EXPECT_LE(ARM_POS_P_GAIN[k], 16383) << k;
    EXPECT_LE(ARM_POS_I_GAIN[k], 16383) << k;
    EXPECT_LE(ARM_POS_D_GAIN[k], 16383) << k;
  }
  EXPECT_EQ(ARM_POS_I_GAIN[ARM_COUNT - 1], 0) << "6 번째 칸 (그리퍼 개폐) 은 I = 0 — 닫힌 끝에 막히면 적분으로 계속 조임";

  isolated([] {
    Bench b;
    for (int k = 0; k < ARM_COUNT; ++k) {
      b.arm_motor(k).pos_p_gain = 123;
      b.arm_motor(k).pos_i_gain = 45;
      b.arm_motor(k).pos_d_gain = 6;
    }
    for (uint8_t id : WHEEL_IDS) {b.w.motors[id].pos_p_gain = 123;}
    b.boot();
    for (int k = 0; k < ARM_COUNT; ++k) {
      EXPECT_EQ(b.arm_motor(k).pos_p_gain, ARM_POS_P_GAIN[k]) << k;
      EXPECT_EQ(b.arm_motor(k).pos_i_gain, ARM_POS_I_GAIN[k]) << k;
      EXPECT_EQ(b.arm_motor(k).pos_d_gain, ARM_POS_D_GAIN[k]) << k;
    }
    for (uint8_t id : WHEEL_IDS) {EXPECT_EQ(b.w.motors[id].pos_p_gain, 123) << "바퀴 보드는 위치 게인을 쓰지 않음 " << int(id);}
    b.run(50);
    EXPECT_FALSE(b.ha.last().error & (ERR_FAULT | ERR_MOTOR_CONFIG));
  });
  // D5-1: OpenCR 만 재부팅되어 팔 토크가 켜져 있어도 게인을 쓰고, 토크는 그대로 둔다
  isolated([] {
    Bench b;
    for (int k = 0; k < ARM_COUNT; ++k) {
      b.arm_motor(k).torque = true;
      b.arm_motor(k).pos_i_gain = 45;
    }
    b.boot();
    for (int k = 0; k < ARM_COUNT; ++k) {
      EXPECT_TRUE(b.arm_motor(k).torque) << k;
      EXPECT_EQ(b.arm_motor(k).pos_i_gain, ARM_POS_I_GAIN[k]) << k;
    }
  });
  // 게인 쓰기가 반영되지 않는 모터 → 부팅 점검 실패 (FAULT, 토크 거부)
  isolated([] {
    Bench b;
    b.arm_motor(2).ignore_gain_writes = true;
    b.arm_motor(2).pos_p_gain = ARM_POS_P_GAIN[2] + 1;
    b.boot();
    for (int t = 0; t < 1000; t += 100) {
      b.drive(50, NO_CMD, FLAG_TORQUE_ON);
      b.drive(50, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    }
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_FAULT);
    EXPECT_TRUE(sa.error & ERR_MOTOR_CONFIG);
    for (uint8_t id : ARM_IDS) {EXPECT_FALSE(b.a.motors[id].torque) << int(id);}
  });
}

// 8. REARM 상승 에지
TEST(FirmwareSil, RearmRisingEdgeActivates)
{
  isolated([] {
    Bench b;
    b.boot();
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    ASSERT_TRUE(b.a.motors[11].torque);
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD) << "토크 ON 직후는 HOLD";
    b.ha.arm[0] = 0.4f;
    b.drive(100, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(0), TICK_CENTER) << "REARM 전에는 현재 위치 유지";
    b.drive(10, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    b.drive(20, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_FALSE(b.ha.last().error & ERR_HOLD);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.4f));
  });
}

// 9. REARM 이 계속 1 일 때 중복 복구 방지
TEST(FirmwareSil, RearmHeldHighDoesNotRepeat)
{
  isolated([] {
    Bench b;
    b.boot();
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    b.drive(100, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);   // 에지 한 번 뒤 계속 1
    ASSERT_FALSE(b.ha.last().error & ERR_HOLD);
    b.ha.arm[4] = 0.3f;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    ASSERT_EQ(b.arm_goal(4), arm_tick(0.3f));
    b.run(300);   // 끊김
    ASSERT_TRUE(b.ha.last().error & ERR_HOLD);
    b.ha.arm[4] = -0.3f;
    b.drive(500, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD) << "계속 1 인 REARM 으로는 다시 풀리지 않음";
    EXPECT_EQ(b.arm_goal(4), arm_tick(0.3f));
    b.drive(10, NO_CMD, FLAG_TORQUE_ON);
    b.drive(20, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    EXPECT_FALSE(b.ha.last().error & ERR_HOLD) << "0 → 1 이면 풀림";
    EXPECT_EQ(b.arm_goal(4), arm_tick(-0.3f));
  });
}

// 10. 통신 단절 · 재연결 (두 보드)
TEST(FirmwareSil, DisconnectAndReconnect)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.hw.wheel[2] = 1.5f;
    b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    const size_t nw = b.hw.states.size(), na = b.ha.states.size();
    b.run(1000);
    State sw = b.hw.last(), sa = b.ha.last();
    EXPECT_NEAR(static_cast<double>(b.hw.states.size() - nw), 100.0, 1.0) << "끊긴 동안에도 STATE 는 계속";
    EXPECT_NEAR(static_cast<double>(b.ha.states.size() - na), 100.0, 1.0);
    EXPECT_TRUE(sw.error & ERR_CMD_TIMEOUT);
    EXPECT_TRUE(sa.error & ERR_CMD_TIMEOUT);
    EXPECT_EQ(b.wheel_goal(2), 0);
    EXPECT_TRUE(sa.error & ERR_HOLD);
    b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    EXPECT_FALSE(b.hw.last().error & ERR_CMD_TIMEOUT);
    EXPECT_FALSE(b.ha.last().error & ERR_CMD_TIMEOUT);
  });
}

// 11. 재연결 후 REARM 없이 팔은 자동 재가동하지 않음
TEST(FirmwareSil, ArmNoAutoRestartAfterReconnect)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[1] = 0.2f;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    b.run(500);
    b.ha.arm[1] = -0.6f;   // 끊긴 사이 호스트 목표가 바뀜
    b.drive(2000, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(1), arm_tick(0.2f)) << "복구된 통신의 새 목표를 적용하지 않음";
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_HOLD);
    EXPECT_FALSE(sa.error & ERR_CMD_TIMEOUT);
  });
}

// 11'. 바퀴는 D3 미결 — 현재 동작(복구 후 자동 재개)을 기록. 정책이 정해지면 바꾼다
TEST(FirmwareSil, WheelCurrentlyResumesAfterReconnect)
{
  isolated([] {
    Bench b;
    b.boot();
    b.hw.wheel[0] = 1.0f;
    b.drive(50, FLAG_TORQUE_ON, NO_CMD);
    b.run(500);
    ASSERT_EQ(b.wheel_goal(0), 0);
    b.drive(50, FLAG_TORQUE_ON, NO_CMD);
    EXPECT_EQ(b.wheel_goal(0), wheel_raw(0, 1.0f));
    EXPECT_FALSE(b.hw.last().error & ERR_HOLD);
  });
}

// 12. v4 방식 호스트 (REARM 을 모름) 는 v5 팔 펌웨어를 움직일 수 없음 + 펌웨어는 v5 를 보고
TEST(FirmwareSil, V4StyleHostCannotMoveArm)
{
  isolated([] {
    Bench b;
    b.boot();
    EXPECT_NE(b.ha.last().fw_version, 4) << "v4 호스트의 버전 검사(fw_version == 4)는 통과하지 못함";
    b.ha.arm[0] = 0.8f;
    b.drive(2000, NO_CMD, FLAG_TORQUE_ON);   // v4 호스트가 버전 검사를 건너뛰었다고 가정해도
    EXPECT_TRUE(b.a.motors[11].torque);
    EXPECT_EQ(b.arm_goal(0), TICK_CENTER) << "REARM 없이는 현재 위치만 유지";
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD);
  });
}

// 13. 한쪽 보드만 끊김 — 다른 보드는 독립적으로 동작
TEST(FirmwareSil, OneBoardLinkLost)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.drive(50, FLAG_TORQUE_ON, FLAG_TORQUE_ON);
    b.hw.wheel[3] = 2.5f;
    b.drive(500, FLAG_TORQUE_ON, NO_CMD);
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD) << "팔만 끊김 → 팔 HOLD";
    EXPECT_FALSE(b.hw.last().error & ERR_CMD_TIMEOUT);
    EXPECT_EQ(b.wheel_goal(3), wheel_raw(3, 2.5f)) << "바퀴 펌웨어는 자기 CMD 만 봄 (호스트 쪽 ESTOP 전파는 루프백 시험)";
  });
}

// 14. 잘못된 순서 · 재전송 · 끊긴 뒤 오래된 명령
TEST(FirmwareSil, StaleAndOutOfOrderCommands)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[0] = 0.1f;
    const Cmd a = b.ha.make(FLAG_TORQUE_ON);
    b.ha.arm[0] = 0.9f;
    const Cmd later = b.ha.make(FLAG_TORQUE_ON);
    b.ha.send(later);
    b.run(10);
    ASSERT_EQ(b.arm_goal(0), arm_tick(0.9f));
    b.ha.send(a);   // 뒤 seq (오래된 명령)
    b.run(10);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.9f));
    Cmd dup = later;   // 같은 seq 재전송 (내용만 다름)
    dup.arm_pos[0] = -0.9f;
    b.ha.send(dup);
    b.run(10);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.9f));
    b.ha.arm[0] = 0.5f;
    b.drive(20, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.5f)) << "앞선 seq 는 받음";

    // 끊긴 사이 쌓였던 오래된 명령이 한꺼번에 도착 (REARM 없음)
    b.run(300);
    ASSERT_TRUE(b.ha.last().error & ERR_HOLD);
    for (int i = 0; i < 20; ++i) {
      b.ha.arm[0] = -0.05f * i;
      b.ha.send(FLAG_TORQUE_ON);
    }
    b.run(50);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.5f)) << "HOLD 중 쌓인 명령은 적용하지 않음";
  });
}

// SyncRead 응답이 끊긴 모터: 쓰레기 값을 쓰지 않고, 위치를 모르면 토크를 켜지 않음
TEST(FirmwareSil, SyncReadLossKeepsLastGoodAndBlocksTorqueOn)
{
  isolated([] {
    Bench b;
    b.arm_motor(3).position = arm_tick(0.25f);
    b.boot();
    b.run(50);
    ASSERT_NEAR(b.ha.last().pos[MAX_WHEELS + 3], 0.25, 0.002);
    b.arm_motor(3).present = false;   // 4번째 관절부터 응답 없음
    b.drive(200, NO_CMD, FLAG_TORQUE_ON);
    State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_SYNCREAD);
    EXPECT_NEAR(sa.pos[MAX_WHEELS + 3], 0.25, 0.002) << "끊긴 칸의 쓰레기 값을 보고하지 않음 (마지막 정상 값)";
    EXPECT_FALSE(b.a.motors[11].torque) << "위치를 모르면 토크를 켜지 않음";
    b.arm_motor(3).present = true;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_TRUE(b.a.motors[11].torque);
    EXPECT_EQ(b.arm_goal(3), arm_tick(0.25f)) << "토크 ON 목표 = 현재 위치";
  });
  // ESTOP 진입: 모든 관절이 응답하면 현재 위치에서 멈추고, 하나라도 모르면 마지막 목표를 유지
  for (const bool lost : {false, true}) {
    isolated([lost] {
      Bench b;
      b.boot();
      b.arm_activate();
      b.ha.arm[0] = 0.3f;
      b.ha.arm[5] = 0.6f;
      b.drive(50, NO_CMD, FLAG_TORQUE_ON);
      b.a.motors_follow_goals = false;
      b.arm_motor(0).position = arm_tick(0.1f);   // 아직 목표로 가는 중
      if (lost) {b.arm_motor(5).present = false;}
      b.drive(50, NO_CMD, FLAG_TORQUE_ON | FLAG_ESTOP);
      EXPECT_TRUE(b.ha.last().error & ERR_HOLD);
      if (lost) {
        EXPECT_EQ(b.arm_goal(0), arm_tick(0.3f)) << "위치를 모르는 관절이 있으면 현재 위치를 믿지 않고 마지막 목표 유지";
        EXPECT_EQ(b.arm_goal(5), arm_tick(0.6f));
      } else {
        EXPECT_EQ(b.arm_goal(0), arm_tick(0.1f)) << "ESTOP = 현재 위치에서 멈춤";
      }
    });
  }
}

// 모터 HW 오류 비트: 정상 모터를 확인할 때마다 지워지지 않고 유지
TEST(FirmwareSil, HwErrorBitIsStable)
{
  isolated([] {
    Bench b;
    b.boot();
    b.a.motors[13].hw_error = 0x20;   // 과부하
    b.run(400);
    b.ha.poll();
    b.ha.states.clear();
    b.run(1000);
    b.ha.poll();
    ASSERT_FALSE(b.ha.states.empty());
    for (const State & s : b.ha.states) {EXPECT_TRUE(s.error & ERR_MOTOR_HW) << "seq " << int(s.seq);}
    b.a.motors[13].hw_error = 0;
    b.run(400);
    EXPECT_FALSE(b.ha.last().error & ERR_MOTOR_HW);
  });
}

// 응답이 끊긴 모터: HW 오류 비트를 지우지 않고, 폴링으로 루프를 막지 않음 (리뷰 R5)
TEST(FirmwareSil, HwErrorKeptWhenMotorStopsResponding)
{
  isolated([] {
    Bench b;
    b.boot();
    b.a.motors[13].hw_error = 0x20;
    b.run(400);
    ASSERT_TRUE(b.ha.last().error & ERR_MOTOR_HW);
    b.a.motors[13].present = false;   // 오류 뒤 버스에서 사라짐
    const int failed_before = b.a.failed_reads;
    b.run(1000);
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_MOTOR_HW) << "응답이 끊겨도 HW 오류 비트를 지우지 않음";
    EXPECT_TRUE(sa.error & ERR_SYNCREAD);
    EXPECT_EQ(b.a.failed_reads, failed_before) << "SyncRead 에 응답하지 않은 모터는 폴링하지 않음 (100 ms 막힘 없음)";
  });
}

// 거부된 프레임 (범위 밖 값) 의 ESTOP 도 바로 정지 — 200 ms 타임아웃을 기다리지 않음 (리뷰 R1)
TEST(FirmwareSil, EstopInRejectedFrameStops)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[0] = 0.5f;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    b.a.motors_follow_goals = false;
    b.a.motors[11].position = arm_tick(0.45f);   // 아직 가는 중
    b.ha.arm[1] = ARM_MAX_RAD[1] + 0.2f;
    b.drive(10, NO_CMD, FLAG_TORQUE_ON | FLAG_ESTOP);
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.45f)) << "현재 위치에서 멈춤";
  });
}

// D5-1: OpenCR 만 재부팅 (팔 모터는 토크 ON 으로 버티는 중) → 끄지 않고 현재 위치로 HOLD
TEST(FirmwareSil, BootKeepsArmTorqueOn)
{
  isolated([] {
    Bench b;
    for (uint8_t id = 11; id <= 16; ++id) {
      b.a.motors[id].torque = true;
      b.a.motors[id].position = arm_tick(0.1f * (id - 10));
      b.a.motors[id].goal_position = b.a.motors[id].position;
    }
    b.boot();
    for (const SilWrite & wr : b.a.writes) {
      EXPECT_FALSE(wr.item == sil_item::TORQUE_ENABLE && wr.value == 0) << "팔 토크 OFF 쓰기 없음 id " << int(wr.id);
      EXPECT_NE(wr.item, sil_item::OPERATING_MODE) << "토크 ON 중 EEPROM 쓰기 없음";
      EXPECT_NE(wr.item, sil_item::RETURN_DELAY_TIME);
    }
    b.run(50);
    for (uint8_t id = 11; id <= 16; ++id) {
      EXPECT_TRUE(b.a.motors[id].torque) << int(id);
      EXPECT_EQ(b.a.motors[id].goal_position, arm_tick(0.1f * (id - 10))) << "목표 = 현재 위치 " << int(id);
      EXPECT_EQ(b.a.motors[id].watchdog, BUS_WATCHDOG_UNIT);
    }
    const State sa = b.ha.last();
    EXPECT_TRUE(sa.error & ERR_HOLD) << "REARM 전까지 HOLD";
    EXPECT_FALSE(sa.error & ERR_FAULT);
    b.ha.arm[0] = 0.3f;
    b.drive(100, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.1f)) << "TORQUE_ON 만으로는 움직이지 않음";
    b.drive(20, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    b.drive(20, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_EQ(b.arm_goal(0), arm_tick(0.3f));
  });
  isolated([] {   // 일부 관절만 토크 ON → 손대지 않고 FAULT
    Bench b;
    b.a.motors[12].torque = true;
    b.a.motors[15].torque = true;
    b.boot();
    b.run(50);
    EXPECT_TRUE(b.ha.last().error & ERR_FAULT);
    EXPECT_TRUE(b.a.motors[12].torque) << "켜져 있던 모터는 그대로 버팀";
    EXPECT_TRUE(b.a.motors[15].torque);
    EXPECT_FALSE(b.a.motors[11].torque);
    b.drive(200, NO_CMD, FLAG_TORQUE_ON | FLAG_REARM);
    EXPECT_FALSE(b.a.motors[11].torque) << "FAULT 이면 토크를 켜지 않음";
  });
}

// 바퀴는 부팅 때 기존처럼 토크를 끈다 — 저장된 Goal Velocity 로 다시 돌 수 있어서
TEST(FirmwareSil, BootTurnsWheelTorqueOff)
{
  isolated([] {
    Bench b;
    for (uint8_t id : WHEEL_IDS) {
      b.w.motors[id].torque = true;
      b.w.motors[id].goal_velocity = 100;
    }
    b.boot();
    for (uint8_t id : WHEEL_IDS) {EXPECT_FALSE(b.w.motors[id].torque) << int(id);}
  });
}

// D5-4: TORQUE_ON 없는 CMD 하나 (flags=0 기본값) 로는 팔 토크가 꺼지지 않음 — HOLD + 연속 요청이어야 함
TEST(FirmwareSil, ArmIgnoresSingleTorqueOffCmd)
{
  isolated([] {
    Bench b;
    b.boot();
    b.arm_activate();
    b.ha.arm[2] = 0.4f;
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    b.a.motors_follow_goals = false;
    b.a.motors[13].position = arm_tick(0.35f);   // 아직 가는 중
    b.drive(10, NO_CMD, 0);
    b.drive(50, NO_CMD, FLAG_TORQUE_ON);
    EXPECT_TRUE(b.a.motors[11].torque) << "CMD 하나로 팔 토크가 꺼지지 않음";
    EXPECT_EQ(b.arm_goal(2), arm_tick(0.35f)) << "토크 OFF 요청 = 현재 위치에서 HOLD";
    EXPECT_TRUE(b.ha.last().error & ERR_HOLD);
    b.drive(ARM_TORQUE_OFF_CMDS * 10 + 20, NO_CMD, 0);
    for (uint8_t id = 11; id <= 16; ++id) {EXPECT_FALSE(b.a.motors[id].torque) << "HOLD + 연속 요청이면 끔 " << int(id);}
  });
}
