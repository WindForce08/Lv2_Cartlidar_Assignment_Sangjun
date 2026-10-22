// TFmini Plus 프레임 파서 시험
#include <gtest/gtest.h>

#include <vector>

#include "perception/tfmini_parser.hpp"

using namespace perception;

static std::vector<uint8_t> frame(uint16_t cm, uint16_t strength, uint16_t temp_raw)
{
  std::vector<uint8_t> f = {0x59, 0x59, uint8_t(cm & 0xFF), uint8_t(cm >> 8), uint8_t(strength & 0xFF),
    uint8_t(strength >> 8), uint8_t(temp_raw & 0xFF), uint8_t(temp_raw >> 8)};
  uint8_t s = 0;
  for (uint8_t b : f) {s = uint8_t(s + b);}
  f.push_back(s);
  return f;
}

TEST(Tfmini, ParsesValidFrame)
{
  TfminiParser p;
  TfminiSample s;
  int n = 0;
  for (uint8_t b : frame(123, 850, (25 + 256) * 8)) {n += p.push(b, s);}
  EXPECT_EQ(n, 1);
  EXPECT_DOUBLE_EQ(s.distance_m, 1.23);
  EXPECT_EQ(s.strength, 850);
  EXPECT_DOUBLE_EQ(s.temperature_c, 25.0);
  EXPECT_TRUE(tfmini_valid(s, 100, 0.1, 12.0));
}

TEST(Tfmini, ResyncAfterNoiseAndBadChecksum)
{
  std::vector<uint8_t> stream = {0x00, 0x59, 0x13, 0x59};
  auto bad = frame(50, 500, 0);
  bad[8] ^= 0xFF;
  stream.insert(stream.end(), bad.begin(), bad.end());
  auto good = frame(37, 400, 0);
  stream.insert(stream.end(), good.begin(), good.end());
  TfminiParser p;
  TfminiSample s;
  int n = 0;
  for (uint8_t b : stream) {n += p.push(b, s);}
  EXPECT_EQ(n, 1);
  EXPECT_DOUBLE_EQ(s.distance_m, 0.37);
  EXPECT_GE(p.bad_frames(), 1u);
}

TEST(Tfmini, ValidityRules)
{
  TfminiSample s;
  s.distance_m = 0.5;
  s.strength = 99;
  EXPECT_FALSE(tfmini_valid(s, 100, 0.1, 12.0));     // 신호 약함
  s.strength = 65535;
  EXPECT_FALSE(tfmini_valid(s, 100, 0.1, 12.0));     // 과포화
  s.strength = 300;
  s.distance_m = 0.05;
  EXPECT_FALSE(tfmini_valid(s, 100, 0.1, 12.0));     // 최소 거리 미만
  s.distance_m = 12.5;
  EXPECT_FALSE(tfmini_valid(s, 100, 0.1, 12.0));
  s.distance_m = 0.1;
  EXPECT_TRUE(tfmini_valid(s, 100, 0.1, 12.0));
}
