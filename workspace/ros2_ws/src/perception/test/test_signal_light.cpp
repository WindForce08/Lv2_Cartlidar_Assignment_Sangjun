// 신호등 찾기 · 프레임 상태 시험 (영상 검출은 그린 사각형, 자리 고정은 위치 목록. 렌더 · 실물 영상 시험은 R1 · R2)
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "perception/signal_light.hpp"

using namespace perception;

namespace
{
// 회색 바탕에 사각형 하나 (BGR)
cv::Mat image_with(const cv::Scalar & bgr)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  img(cv::Rect(300, 200, 12, 10)).setTo(bgr);
  return img;
}

const Vec3 LIGHT{2.0, 0.5, 0.12}, PUCK{1.5, -0.3, 0.11};
auto always = [](const Vec3 &) {return true;};

LightObs at(const Vec3 & p, Light c) {return {c, p, 50};}
}  // namespace

TEST(SignalLight, BrightColorsAreBlobsDullAreNot)
{
  const LightBlobConfig c;
  struct Case {cv::Scalar bgr; Light want;};
  for (const Case & k : {Case{{20, 20, 255}, Light::Red}, Case{{20, 230, 250}, Light::Yellow},
      Case{{30, 250, 40}, Light::Green}})
  {
    const auto b = detect_light_blobs(image_with(k.bgr), c);
    ASSERT_EQ(b.size(), 1u);
    EXPECT_EQ(b[0].color, k.want);
    EXPECT_NEAR(b[0].center.x, 305.5, 0.6);
    EXPECT_EQ(b[0].area, 120);
  }
  // 꺼진 등 · 퍽처럼 어두운 색, 하얗게 날아간 부분은 후보 아님
  for (const auto & dull : {cv::Scalar(30, 30, 150), cv::Scalar(30, 120, 30), cv::Scalar(250, 250, 250)}) {
    EXPECT_TRUE(detect_light_blobs(image_with(dull), c).empty());
  }
}

TEST(SignalLight, LocksOnBlinkingSiteNotOnSteadyPuck)
{
  SignalLightLocator loc;
  // 퍽(늘 보임)과 신호등(빨강 켜짐 3프레임 · 꺼짐 3프레임) — 신호등만 고정
  int frame = 0;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 6; ++i, ++frame) {
      std::vector<LightObs> obs{at(PUCK, Light::Red)};
      if (i < 3) {obs.push_back(at(LIGHT, Light::Red));}
      const auto s = loc.update(frame / 30.0, obs, always);
      if (!loc.locked()) {EXPECT_FALSE(s.has_value());}
    }
  }
  ASSERT_TRUE(loc.locked());
  EXPECT_NEAR(loc.position().x, LIGHT.x, 1e-6);
  // 고정 뒤: 신호등 자리 후보 → 그 색, 퍽만 보임 → 꺼짐, 화면 밖 → 모름
  auto s = loc.update(1.0, {at(PUCK, Light::Red), at(LIGHT, Light::Green)}, always);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, Light::Green);
  s = loc.update(1.04, {at(PUCK, Light::Red)}, always);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, Light::Off);
  s = loc.update(1.07, {}, [](const Vec3 &) {return false;});
  EXPECT_FALSE(s.has_value());
}

TEST(SignalLight, OneFrameDropoutIsNotABlink)
{
  // 늘 켜진 물체가 한 프레임씩 검출에서 빠져도(꺼짐 1프레임) 신호등으로 고정하지 않음
  SignalLightLocator loc;
  for (int f = 0; f < 60; ++f) {
    std::vector<LightObs> obs;
    if (f % 4 != 3) {obs.push_back(at(PUCK, Light::Red));}
    loc.update(f / 30.0, obs, always);
  }
  EXPECT_FALSE(loc.locked());
}

TEST(SignalLight, LockedSiteFeedsSequence)
{
  // 고정 → 프레임 상태를 점멸 세기에 넣으면 순서가 확정된다 (켜짐 0.3 s · 꺼짐 0.2 s · 텀 1.5 s)
  const std::string seq = "RGRRGRGGYGYG";
  const double on = 0.3, off = 0.2, term = 1.5, P = 12 * on + 11 * off + term;
  SignalLightLocator loc;
  SignalSequenceTracker tr;
  for (int f = 0; f / 30.0 < 4 * P; ++f) {
    const double t = f / 30.0;
    double u = std::fmod(t, P);
    Light l = Light::Off;
    for (char c : seq) {
      if (u < on) {l = c == 'R' ? Light::Red : c == 'Y' ? Light::Yellow : Light::Green; break;}
      u -= on;
      if (u < off) {break;}
      u -= off;
    }
    std::vector<LightObs> obs{at(PUCK, Light::Green)};
    if (l != Light::Off) {obs.push_back(at({LIGHT.x, LIGHT.y, LIGHT.z + 0.035 * (static_cast<int>(l) - 2)}, l));}
    if (const auto s = loc.update(t, obs, always)) {tr.update(t, *s);}
  }
  ASSERT_TRUE(loc.locked());
  EXPECT_EQ(tr.result().status, SignalResult::CONFIRMED);
  EXPECT_EQ(tr.result().sequence, seq);
}
