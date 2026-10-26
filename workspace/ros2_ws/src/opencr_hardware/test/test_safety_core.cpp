// 펌웨어 안전 판단 시험: opencr_firmware/safety_core.h (opencr_firmware.ino 가 include 하는 그 파일) 를 직접 돌린다.
//   관절 한계 · 타임아웃은 펌웨어 config.h 값을 그대로 쓴다. Arduino · 모터는 없음 (시간 · 위치 유효성은 시험이 넣음).
#include <gtest/gtest.h>

#include <limits>

#include "config.h"
#include "safety_core.h"

using namespace opencr_proto;
using opencr_safety::Reject;
using opencr_safety::SafetyCore;

namespace
{

const float NaN = std::numeric_limits<float>::quiet_NaN();
const float Inf = std::numeric_limits<float>::infinity();

// ino 처럼 CMD 를 넣고 10 ms 주기로 tick 하는 시험 틀
struct Rig
{
  explicit Rig(uint8_t role)
  : core(role, ARM_MIN_RAD, ARM_MAX_RAD, CMD_TIMEOUT_MS, ARM_TORQUE_OFF_CMDS), role(role) {}

  Cmd make(uint8_t flags = FLAG_TORQUE_ON)
  {
    Cmd c{};
    c.role = role;
    c.flags = flags;
    c.seq = seq++;
    return c;
  }
  Reject send(uint8_t flags = FLAG_TORQUE_ON) {return core.on_cmd(make(flags), now);}
  Reject send(const Cmd & c) {return core.on_cmd(c, now);}
  void tick(uint32_t dt_ms = 10, bool hw_estop = false)
  {
    now += dt_ms;
    core.tick(now, hw_estop, torque, positions_valid);
  }
  void torque_on() {torque = true; core.on_torque_on();}
  // 토크 ON → REARM 으로 ACTIVE 까지
  void activate()
  {
    send(FLAG_TORQUE_ON);
    tick();
    torque_on();
    send(FLAG_TORQUE_ON | FLAG_REARM);
    tick();
    send(FLAG_TORQUE_ON);
    tick();
  }

  SafetyCore core;
  uint8_t role;
  uint8_t seq = 0;
  uint32_t now = 1000;
  bool torque = false;
  bool positions_valid = true;
};

}  // namespace

TEST(SafetyCore, FiniteCheck)
{
  EXPECT_TRUE(opencr_safety::finite(0.f));
  EXPECT_TRUE(opencr_safety::finite(std::numeric_limits<float>::max()));
  EXPECT_TRUE(opencr_safety::finite(std::numeric_limits<float>::denorm_min()));
  EXPECT_FALSE(opencr_safety::finite(NaN));
  EXPECT_FALSE(opencr_safety::finite(Inf));
  EXPECT_FALSE(opencr_safety::finite(-Inf));
}

TEST(SafetyCore, StartsTimedOutWithoutCommand)
{
  Rig r(ROLE_ARM);
  r.tick();
  EXPECT_TRUE(r.core.timeout());
  EXPECT_FALSE(r.core.apply_targets());
  EXPECT_TRUE(r.core.status_bits() & ERR_CMD_TIMEOUT);
}

TEST(SafetyCore, WrongRoleRejectedAndDoesNotRefreshTimer)
{
  Rig r(ROLE_WHEEL);
  ASSERT_EQ(r.send(), opencr_safety::ACCEPTED);
  r.tick();
  EXPECT_FALSE(r.core.timeout());
  Cmd other = r.make();
  other.role = ROLE_ARM;
  for (int i = 0; i < 25; ++i) {
    EXPECT_EQ(r.send(other), opencr_safety::BAD_ROLE);
    r.tick();
  }
  EXPECT_TRUE(r.core.timeout()) << "다른 역할 CMD 만 오면 200 ms 뒤 타임아웃";
  EXPECT_TRUE(r.core.status_bits() & ERR_ROLE_MISMATCH);
  ASSERT_EQ(r.send(), opencr_safety::ACCEPTED);
  r.tick();
  EXPECT_FALSE(r.core.status_bits() & ERR_ROLE_MISMATCH);
}

TEST(SafetyCore, WheelRejectsNanInfButNotRange)
{
  Rig r(ROLE_WHEEL);
  for (int k = 0; k < MAX_WHEELS; ++k) {
    for (float bad : {NaN, Inf, -Inf}) {
      Cmd c = r.make();
      c.wheel_vel[k] = bad;
      EXPECT_EQ(r.send(c), opencr_safety::NOT_FINITE) << "wheel " << k;
    }
  }
  Cmd big = r.make();
  big.wheel_vel[0] = 1e6f;
  EXPECT_EQ(r.send(big), opencr_safety::ACCEPTED) << "바퀴 범위 초과는 ino 가 clamp (D6 미결)";
}

TEST(SafetyCore, ArmRejectsNanInfAndOutOfRange)
{
  Rig r(ROLE_ARM);
  for (int k = 0; k < ARM_JOINTS; ++k) {
    for (float bad : {NaN, Inf, -Inf}) {
      Cmd c = r.make();
      c.arm_pos[k] = bad;
      EXPECT_EQ(r.send(c), opencr_safety::NOT_FINITE) << "joint " << k;
    }
    Cmd lo = r.make();
    lo.arm_pos[k] = ARM_MIN_RAD[k];
    EXPECT_EQ(r.send(lo), opencr_safety::ACCEPTED) << "한계값 자체는 허용 " << k;
    Cmd hi = r.make();
    hi.arm_pos[k] = ARM_MAX_RAD[k] + 0.001f;
    EXPECT_EQ(r.send(hi), opencr_safety::OUT_OF_RANGE) << k;
    Cmd below = r.make();
    below.arm_pos[k] = ARM_MIN_RAD[k] - 0.001f;
    EXPECT_EQ(r.send(below), opencr_safety::OUT_OF_RANGE) << k;
  }
  Cmd g = r.make(FLAG_TORQUE_ON | FLAG_GRIPPER_VALID);
  g.gripper_pos = NaN;
  EXPECT_EQ(r.send(g), opencr_safety::NOT_FINITE);
  Cmd g_unused = r.make(FLAG_TORQUE_ON);
  g_unused.gripper_pos = NaN;
  EXPECT_EQ(r.send(g_unused), opencr_safety::ACCEPTED) << "GRIPPER_VALID 가 없으면 그리퍼 값은 쓰지 않음";
}

TEST(SafetyCore, RejectedFramesDoNotRefreshTimeout)
{
  Rig r(ROLE_ARM);
  ASSERT_EQ(r.send(), opencr_safety::ACCEPTED);
  const uint32_t t0 = r.now;
  for (int i = 0; i < 19; ++i) {
    r.tick();
    Cmd c = r.make();
    c.arm_pos[2] = NaN;
    EXPECT_NE(r.send(c), opencr_safety::ACCEPTED);
  }
  r.tick(10);   // t0 + 200
  EXPECT_EQ(r.now - t0, CMD_TIMEOUT_MS);
  EXPECT_FALSE(r.core.timeout()) << "정확히 200 ms 는 아직 아님";
  r.tick(1);
  EXPECT_TRUE(r.core.timeout()) << "거부된 CMD 는 타이머를 갱신하지 않음";
}

TEST(SafetyCore, SeqRejectsDuplicateAndOld)
{
  Rig r(ROLE_WHEEL);
  Cmd c = r.make();
  c.seq = 10;
  ASSERT_EQ(r.send(c), opencr_safety::ACCEPTED);
  EXPECT_EQ(r.send(c), opencr_safety::BAD_SEQ) << "같은 seq = 재전송";
  c.seq = 9;
  EXPECT_EQ(r.send(c), opencr_safety::BAD_SEQ) << "뒤 seq = 오래된 명령";
  c.seq = 12;
  EXPECT_EQ(r.send(c), opencr_safety::ACCEPTED) << "건너뛴 seq 는 허용";
  c.seq = 12 + 127;
  EXPECT_EQ(r.send(c), opencr_safety::ACCEPTED);
  c.seq = static_cast<uint8_t>(c.seq + 128);
  EXPECT_EQ(r.send(c), opencr_safety::BAD_SEQ) << "128 이상 앞섬 = 뒤로 본다";
  c.seq = 250;
  Rig w(ROLE_WHEEL);
  ASSERT_EQ(w.send(c), opencr_safety::ACCEPTED);
  c.seq = 3;
  EXPECT_EQ(w.send(c), opencr_safety::ACCEPTED) << "255 → 0 래핑";
}

TEST(SafetyCore, SeqBaselineClearsAfterTimeout)
{
  Rig r(ROLE_WHEEL);
  Cmd c = r.make();
  c.seq = 100;
  ASSERT_EQ(r.send(c), opencr_safety::ACCEPTED);
  c.seq = 0;
  EXPECT_EQ(r.send(c), opencr_safety::BAD_SEQ);
  r.tick(CMD_TIMEOUT_MS + 1);
  ASSERT_TRUE(r.core.timeout());
  EXPECT_EQ(r.send(c), opencr_safety::ACCEPTED) << "호스트 재시작 뒤 seq 0 부터 받음";
}

TEST(SafetyCore, ArmTorqueOnHoldsUntilRearmEdge)
{
  Rig r(ROLE_ARM);
  r.send();
  r.tick();
  r.torque_on();
  r.tick();
  EXPECT_TRUE(r.core.hold());
  EXPECT_TRUE(r.core.status_bits() & ERR_HOLD);
  EXPECT_FALSE(r.core.apply_targets());
  for (int i = 0; i < 10; ++i) {
    r.send(FLAG_TORQUE_ON);
    r.tick();
  }
  EXPECT_TRUE(r.core.hold()) << "REARM 없이는 ACTIVE 가 되지 않음";
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_FALSE(r.core.hold());
  EXPECT_TRUE(r.core.apply_targets());
  EXPECT_FALSE(r.core.status_bits() & ERR_HOLD);
}

TEST(SafetyCore, ArmCommRecoveryDoesNotResume)
{
  Rig r(ROLE_ARM);
  r.activate();
  ASSERT_TRUE(r.core.apply_targets());
  r.tick(CMD_TIMEOUT_MS + 10);
  EXPECT_TRUE(r.core.timeout());
  EXPECT_TRUE(r.core.hold());
  for (int i = 0; i < 20; ++i) {
    r.send(FLAG_TORQUE_ON);
    r.tick();
  }
  EXPECT_FALSE(r.core.timeout()) << "통신은 복구됨";
  EXPECT_TRUE(r.core.hold()) << "복구만으로는 새 목표를 받지 않음";
  EXPECT_FALSE(r.core.apply_targets());
}

TEST(SafetyCore, ArmRearmHeldHighCountsOnce)
{
  Rig r(ROLE_ARM);
  r.activate();
  // REARM 을 계속 1 로 둔 채 끊겼다 복구
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  r.tick(CMD_TIMEOUT_MS + 10);
  ASSERT_TRUE(r.core.hold());
  for (int i = 0; i < 20; ++i) {
    r.send(FLAG_TORQUE_ON | FLAG_REARM);
    r.tick();
  }
  EXPECT_TRUE(r.core.hold()) << "끊기기 전부터 1 이던 REARM 은 에지가 아님";
  r.send(FLAG_TORQUE_ON);
  r.tick();
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_FALSE(r.core.hold()) << "0 다음 1 이면 에지";
  // ACTIVE 중 REARM 1 유지 → ESTOP → 해제 (REARM 여전히 1)
  r.send(FLAG_TORQUE_ON | FLAG_REARM | FLAG_ESTOP);
  r.tick();
  ASSERT_TRUE(r.core.hold());
  for (int i = 0; i < 5; ++i) {
    r.send(FLAG_TORQUE_ON | FLAG_REARM);
    r.tick();
  }
  EXPECT_TRUE(r.core.hold()) << "REARM 이 계속 1 이면 다시 풀지 않음";
}

TEST(SafetyCore, ArmEstopReleaseNeedsRearm)
{
  Rig r(ROLE_ARM);
  r.activate();
  r.send(FLAG_TORQUE_ON | FLAG_ESTOP);
  r.tick();
  EXPECT_TRUE(r.core.hold());
  EXPECT_TRUE(r.core.entered_stop_hold()) << "ino 가 현재 위치로 목표를 바꾸는 신호";
  r.send(FLAG_TORQUE_ON | FLAG_ESTOP);
  r.tick();
  EXPECT_FALSE(r.core.entered_stop_hold()) << "진입 주기에만";
  r.send(FLAG_TORQUE_ON | FLAG_ESTOP | FLAG_REARM);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "ESTOP 중 REARM 은 무시";
  r.send(FLAG_TORQUE_ON);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "ESTOP 해제만으로는 풀리지 않음";
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_FALSE(r.core.hold());
}

TEST(SafetyCore, ArmHwEstopPinHolds)
{
  Rig r(ROLE_ARM);
  r.activate();
  r.send(FLAG_TORQUE_ON);
  r.tick(10, true);
  EXPECT_TRUE(r.core.hold());
  EXPECT_TRUE(r.core.estop());
}

TEST(SafetyCore, ArmRearmNeedsValidPositionsAndNoFault)
{
  Rig r(ROLE_ARM);
  r.activate();
  r.tick(CMD_TIMEOUT_MS + 10);
  ASSERT_TRUE(r.core.hold());
  r.send(FLAG_TORQUE_ON);
  r.tick();
  r.positions_valid = false;
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "위치를 모르면 REARM 거부";
  r.positions_valid = true;
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "거부된 에지는 버림 — 다시 0→1 필요";
  r.send(FLAG_TORQUE_ON);
  r.tick();
  r.core.set_fault();
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "FAULT 이면 REARM 거부";
  EXPECT_TRUE(r.core.status_bits() & ERR_FAULT);
}

TEST(SafetyCore, RearmWhileTorqueOffIsDiscarded)
{
  Rig r(ROLE_ARM);
  r.send(FLAG_TORQUE_ON);
  r.tick();
  r.send(FLAG_TORQUE_ON | FLAG_REARM);   // 토크를 켜기 전 에지
  r.tick();
  r.torque_on();
  r.send(FLAG_TORQUE_ON | FLAG_REARM);
  r.tick();
  EXPECT_TRUE(r.core.hold()) << "토크 ON 전 REARM 은 남지 않음";
}

TEST(SafetyCore, ArmTorqueOffClearsHold)
{
  Rig r(ROLE_ARM);
  r.activate();
  r.tick(CMD_TIMEOUT_MS + 10);
  ASSERT_TRUE(r.core.hold());
  r.torque = false;
  r.core.on_torque_off();
  r.tick();
  EXPECT_FALSE(r.core.hold()) << "토크 OFF = READY (D5 미결: OFF 요청은 기존처럼 즉시 따름)";
}

// 값 · seq 가 틀려 거부된 프레임이라도 역할이 맞으면 ESTOP 은 정지 요청으로 받는다 (리뷰 R1)
TEST(SafetyCore, EstopInRejectedFrameStillStops)
{
  for (const int bad : {0, 1, 2}) {   // 0 범위 밖 · 1 NaN · 2 오래된 seq
    Rig r(ROLE_ARM);
    r.activate();
    Cmd c = r.make(FLAG_TORQUE_ON | FLAG_ESTOP);
    if (bad == 0) {c.arm_pos[1] = ARM_MAX_RAD[1] + 0.5f;}
    if (bad == 1) {c.arm_pos[1] = NaN;}
    if (bad == 2) {c.seq = static_cast<uint8_t>(r.seq - 5);}
    EXPECT_NE(r.send(c), opencr_safety::ACCEPTED) << bad;
    r.tick();
    EXPECT_TRUE(r.core.estop()) << bad;
    EXPECT_TRUE(r.core.hold()) << bad;
    EXPECT_TRUE(r.core.entered_stop_hold()) << "현재 위치에서 멈춤 " << bad;
  }
  Rig w(ROLE_WHEEL);
  w.send();
  w.tick();
  Cmd c = w.make(FLAG_TORQUE_ON | FLAG_ESTOP);
  c.wheel_vel[0] = NaN;
  w.send(c);
  w.tick();
  EXPECT_FALSE(w.core.apply_targets()) << "바퀴도 0";
  Rig other(ROLE_ARM);
  other.activate();
  Cmd o = other.make(FLAG_TORQUE_ON | FLAG_ESTOP);
  o.role = ROLE_WHEEL;
  other.send(o);
  other.tick();
  EXPECT_FALSE(other.core.estop()) << "다른 역할 프레임의 ESTOP 은 이 보드 것이 아님";
}

// D5-4: 팔은 TORQUE_ON 없는 CMD 하나로 토크를 끄지 않는다 — HOLD + 연속 ARM_TORQUE_OFF_CMDS 번
TEST(SafetyCore, ArmTorqueOffNeedsHoldAndConsecutiveCmds)
{
  Rig r(ROLE_ARM);
  r.activate();
  r.send(0);   // flags=0 기본값 CMD 하나
  r.tick();
  EXPECT_FALSE(r.core.torque_off_allowed());
  EXPECT_TRUE(r.core.hold()) << "작동 중 토크 OFF 요청은 먼저 HOLD (현재 위치)";
  EXPECT_TRUE(r.core.entered_stop_hold());
  for (int i = 1; i < ARM_TORQUE_OFF_CMDS - 1; ++i) {
    r.send(0);
    r.tick();
  }
  EXPECT_FALSE(r.core.torque_off_allowed()) << ARM_TORQUE_OFF_CMDS - 1 << " 번째까지는 불허";
  r.send(0);
  r.tick();
  EXPECT_TRUE(r.core.torque_off_allowed());
  r.send(FLAG_TORQUE_ON);
  r.tick();
  r.send(0);
  r.tick();
  EXPECT_FALSE(r.core.torque_off_allowed()) << "TORQUE_ON CMD 가 끼면 다시 처음부터";
  for (int i = 1; i < ARM_TORQUE_OFF_CMDS; ++i) {r.send(0); r.tick(5);}
  r.tick(CMD_TIMEOUT_MS + 1);
  r.send(0);
  r.tick();
  EXPECT_FALSE(r.core.torque_off_allowed()) << "끊김이 있으면 처음부터";
}

TEST(SafetyCore, WheelTorqueOffImmediate)
{
  Rig r(ROLE_WHEEL);
  r.send();
  r.tick();
  r.torque_on();
  r.send(0);
  r.tick();
  EXPECT_TRUE(r.core.torque_off_allowed()) << "바퀴는 기존처럼 즉시";
}

// D3 미결 — 바퀴는 v4 와 같은 현재 동작을 기록한다 (정책이 정해지면 이 시험을 바꾼다)
TEST(SafetyCore, WheelTimeoutStopsAndCurrentlyAutoResumes)
{
  Rig r(ROLE_WHEEL);
  r.send();
  r.tick();
  r.torque_on();
  EXPECT_FALSE(r.core.hold()) << "바퀴는 HOLD 래치 없음";
  EXPECT_TRUE(r.core.apply_targets());
  r.tick(CMD_TIMEOUT_MS + 10);
  EXPECT_FALSE(r.core.apply_targets()) << "무수신 → 바퀴 0";
  r.send();
  r.tick();
  EXPECT_TRUE(r.core.apply_targets()) << "현재 동작: 복구 후 REARM 없이 재개 (D3 미결)";
  EXPECT_FALSE(r.core.status_bits() & ERR_HOLD);
  r.send(FLAG_TORQUE_ON | FLAG_ESTOP);
  r.tick();
  EXPECT_FALSE(r.core.apply_targets()) << "ESTOP → 바퀴 0";
}
