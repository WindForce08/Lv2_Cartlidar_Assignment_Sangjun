// USB 패킷 규격 시험: 인코딩 ↔ 디코딩, CRC, 잡음 속 재동기, 펌웨어 사본과 동일 여부
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "opencr_hardware/opencr_protocol.hpp"

using namespace opencr_proto;

TEST(Protocol, Sizes)
{
  EXPECT_EQ(CMD_PAYLOAD, 55u);
  EXPECT_EQ(STATE_PAYLOAD, 192u);
}

TEST(Protocol, CrcKnownValue)
{
  // Dynamixel 2.0 매뉴얼 예: FF FF FD 00 01 03 00 01 → CRC 0x4E19 (19 4E)
  const uint8_t ping[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01};
  EXPECT_EQ(crc16(ping, sizeof(ping)), 0x4E19);
}

TEST(Protocol, CmdRoundTrip)
{
  Cmd c{};
  c.role = ROLE_ARM;
  for (int i = 0; i < MAX_WHEELS; ++i) {c.wheel_vel[i] = 0.5f * i - 1.0f;}
  for (int i = 0; i < ARM_JOINTS; ++i) {c.arm_pos[i] = -0.3f * i;}
  c.gripper_pos = 0.012f;
  c.flags = FLAG_TORQUE_ON | FLAG_ESTOP;
  c.seq = 200;
  uint8_t frame[MAX_FRAME];
  const size_t n = encode_cmd(c, frame);
  ASSERT_EQ(n, CMD_PAYLOAD + FRAME_OVERHEAD);

  FrameParser p;
  bool got = false;
  for (size_t i = 0; i < n; ++i) {got = p.push(frame[i]);}
  ASSERT_TRUE(got);
  ASSERT_EQ(p.type(), TYPE_CMD);
  Cmd d{};
  ASSERT_TRUE(decode_cmd(p.payload(), p.length(), d));
  for (int i = 0; i < MAX_WHEELS; ++i) {EXPECT_FLOAT_EQ(d.wheel_vel[i], c.wheel_vel[i]);}
  for (int i = 0; i < ARM_JOINTS; ++i) {EXPECT_FLOAT_EQ(d.arm_pos[i], c.arm_pos[i]);}
  EXPECT_FLOAT_EQ(d.gripper_pos, c.gripper_pos);
  EXPECT_EQ(d.flags, c.flags);
  EXPECT_EQ(d.seq, c.seq);
}

TEST(Protocol, StateResyncAfterNoiseAndBadCrc)
{
  State s{};
  s.role = ROLE_WHEEL;
  for (int i = 0; i < STATE_JOINTS; ++i) {s.pos[i] = 0.1f * i; s.vel[i] = -0.2f * i; s.effort[i] = 0.01f * i;}
  s.quat[0] = 1.0f;
  s.gyro[2] = 0.5f;
  s.accel[2] = 9.8f;
  s.battery_v = 11.6f;
  s.error = ERR_CMD_TIMEOUT;
  s.fw_version = PROTOCOL_VERSION;
  s.seq = 7;
  uint8_t frame[MAX_FRAME];
  const size_t n = encode_state(s, frame);

  std::vector<uint8_t> stream = {0x00, 0xFF, 0x12, 0xFF, 0xFF};   // 잡음
  std::vector<uint8_t> corrupt(frame, frame + n);
  corrupt[20] ^= 0x55;                                          // CRC 틀린 프레임
  stream.insert(stream.end(), corrupt.begin(), corrupt.end());
  stream.insert(stream.end(), frame, frame + n);                // 정상 프레임

  FrameParser p;
  int ok = 0;
  State d{};
  for (uint8_t b : stream) {
    if (p.push(b)) {
      ASSERT_EQ(p.type(), TYPE_STATE);
      ASSERT_TRUE(decode_state(p.payload(), p.length(), d));
      ++ok;
    }
  }
  EXPECT_EQ(ok, 1);
  EXPECT_GE(p.bad_frames(), 1u);
  EXPECT_FLOAT_EQ(d.pos[11], s.pos[11]);
  EXPECT_FLOAT_EQ(d.accel[2], 9.8f);
  EXPECT_FLOAT_EQ(d.battery_v, 11.6f);
  EXPECT_EQ(d.error, ERR_CMD_TIMEOUT);
  EXPECT_EQ(d.fw_version, PROTOCOL_VERSION);
  EXPECT_EQ(d.seq, 7);
}

TEST(Protocol, FirmwareCopyIsIdentical)
{
  auto slurp = [](const char * path) {
      std::ifstream f(path, std::ios::binary);
      std::stringstream ss;
      ss << f.rdbuf();
      return ss.str();
    };
  const std::string host = slurp(HOST_PROTOCOL_PATH);
  const std::string fw = slurp(FIRMWARE_PROTOCOL_PATH);
  ASSERT_FALSE(host.empty());
  ASSERT_FALSE(fw.empty()) << "opencr_firmware/opencr_protocol.h 없음";
  EXPECT_EQ(host, fw) << "펌웨어 사본이 호스트 헤더와 다름 — 한쪽을 고치면 둘 다 복사";
}
