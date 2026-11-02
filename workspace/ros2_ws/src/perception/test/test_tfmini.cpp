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

// mm 출력 모드: 같은 프레임 값을 mm 로 읽는다 (박스 거리 1 mm 단위)
TEST(Tfmini, MillimeterUnit)
{
  TfminiParser p(0.001);
  TfminiSample s;
  int n = 0;
  for (uint8_t b : frame(153, 900, 0)) {n += p.push(b, s);}
  EXPECT_EQ(n, 1);
  EXPECT_NEAR(s.distance_m, 0.153, 1e-9);
}

TEST(Tfmini, OutputFormatCommandChecksum)
{
  for (const uint8_t * c : {TFMINI_CMD_OUTPUT_MM, TFMINI_CMD_OUTPUT_CM}) {
    uint8_t sum = 0;
    for (int i = 0; i < 4; ++i) {sum = uint8_t(sum + c[i]);}
    EXPECT_EQ(sum, c[4]);
  }
}

// 실물 보정: 점 없음 = 오프셋만, 1점 = 차이를 오프셋으로, 2점 이상 = 구간 선형 (끝 밖은 끝 구간으로 연장)
TEST(Tfmini, Calibration)
{
  EXPECT_DOUBLE_EQ(tfmini_calibrate(0.20, {}, {}, 0.01), 0.21);
  EXPECT_DOUBLE_EQ(tfmini_calibrate(0.20, {0.15}, {0.14}), 0.19);
  const std::vector<double> raw = {0.10, 0.20, 0.40}, tru = {0.12, 0.21, 0.40};
  EXPECT_NEAR(tfmini_calibrate(0.15, raw, tru), 0.165, 1e-9);
  EXPECT_NEAR(tfmini_calibrate(0.30, raw, tru), 0.305, 1e-9);
  EXPECT_NEAR(tfmini_calibrate(0.05, raw, tru), 0.075, 1e-9);   // 아래 끝 밖
  EXPECT_NEAR(tfmini_calibrate(0.50, raw, tru), 0.495, 1e-9);   // 위 끝 밖
}
