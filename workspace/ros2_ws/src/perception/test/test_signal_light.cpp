// 신호등 찾기 · 프레임 상태 시험 (영상 검출은 그린 사각형, 자리 고정은 위치 목록. 렌더 · 실물 영상 시험은 R1 · R2)
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <random>
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

// 칸 중심 (아래부터 초록 · 노랑 · 빨강, 34.9 mm 간격, 가운데 = LIGHT)
Vec3 seg_of(Light l)
{
  const int k = l == Light::Green ? -1 : l == Light::Yellow ? 0 : 1;
  return {LIGHT.x, LIGHT.y, LIGHT.z + 0.0349 * k};
}

// 반복하는 신호등 (test_signal.cpp 의 Blinker 와 같음)
struct Blinker
{
  std::string seq;
  double t_on, t_off, t_term;
  double period() const {return seq.size() * t_on + (seq.size() - 1) * t_off + t_term;}
  Light at(double t) const
  {
    double u = std::fmod(t, period());
    for (char c : seq) {
      if (u < t_on) {return c == 'R' ? Light::Red : c == 'Y' ? Light::Yellow : Light::Green;}
      u -= t_on;
      if (u < t_off) {return Light::Off;}
      u -= t_off;
    }
    return Light::Off;
  }
};

// 신호등 고정 (켜짐 3프레임 · 꺼짐 3프레임 × 3)
void lock_on_light(SignalLightLocator & loc, double & t)
{
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 6; ++i, t += 1.0 / 30) {
      std::vector<LightObs> obs;
      if (i < 3) {obs.push_back(at(seg_of(Light::Red), Light::Red));}
      loc.update(t, obs, always);
    }
  }
}
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

// U5a: 가림 — 켜진 등이 없고 칸 앞이 가려진 프레임은 꺼짐이 아니라 모름
TEST(SignalLight, U5a_OccludedFrameIsUnknownNotOff)
{
  SignalLightLocator loc;
  double t = 0;
  lock_on_light(loc, t);
  ASSERT_TRUE(loc.locked());
  auto hidden = [](const SignalLightLocator::Segments &) {return true;};
  auto clear = [](const SignalLightLocator::Segments &) {return false;};
  // 가림 + 후보 없음 → 모름
  auto s = loc.update(t += 1.0 / 30, {}, always, hidden);
  EXPECT_FALSE(s.has_value());
  EXPECT_TRUE(loc.occluded_last());
  // 가림이어도 켜진 칸이 보이면 그 색
  s = loc.update(t += 1.0 / 30, {at(seg_of(Light::Yellow), Light::Yellow)}, always, hidden);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, Light::Yellow);
  EXPECT_FALSE(loc.occluded_last());
  // 안 가림 + 후보 없음 → 꺼짐 (지금과 같음), 가림 검사를 안 주면 꺼짐 (지금과 같음)
  s = loc.update(t += 1.0 / 30, {}, always, clear);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, Light::Off);
  s = loc.update(t += 1.0 / 30, {}, always);
  ASSERT_TRUE(s);
  EXPECT_EQ(*s, Light::Off);
  // 화면 밖이면 가림과 상관없이 모름
  s = loc.update(t += 1.0 / 30, {}, [](const Vec3 &) {return false;}, hidden);
  EXPECT_FALSE(s.has_value());
}

TEST(SignalLight, U5a_OccludedSteadyObjectDoesNotLock)
{
  // 늘 켜진 물체가 가끔 가려짐(3프레임): 가림을 꺼짐으로 세면 점멸로 보여 고정되지만, 모름이면 고정 안 됨
  for (const bool use_occlusion : {true, false}) {
    SignalLightLocator loc;
    for (int f = 0; f < 90; ++f) {
      const bool hide = f % 10 >= 7;
      std::vector<LightObs> obs;
      if (!hide) {obs.push_back(at(PUCK, Light::Red));}
      if (use_occlusion) {
        loc.update(f / 30.0, obs, always, [&](const SignalLightLocator::Segments &) {return hide;});
      } else {
        loc.update(f / 30.0, obs, always);
      }
    }
    EXPECT_EQ(loc.locked(), !use_occlusion) << (use_occlusion ? "가림 처리" : "가림을 꺼짐으로 셈");
  }
}

TEST(SignalLight, U5a_SegmentsFillUnseenColors)
{
  // 빨강만 본 뒤: 빨강은 본 위치, 노랑 · 초록은 그 아래 34.9 mm 간격 / 세 색을 보면 모두 본 위치
  SignalLightLocator loc;
  double t = 0;
  lock_on_light(loc, t);
  auto g = loc.segments();
  EXPECT_NEAR(g[2].z, seg_of(Light::Red).z, 1e-9);
  EXPECT_NEAR(g[1].z, seg_of(Light::Red).z - 0.0349, 1e-9);
  EXPECT_NEAR(g[0].z, seg_of(Light::Red).z - 2 * 0.0349, 1e-9);
  EXPECT_NEAR(g[0].x, LIGHT.x, 1e-9);
  for (Light l : {Light::Green, Light::Yellow}) {
    Vec3 p = seg_of(l);
    p.z += 0.003;   // 실제 칸이 간격 가정과 3 mm 다름
    loc.update(t += 1.0 / 30, {at(p, l)}, always);
  }
  g = loc.segments();
  EXPECT_NEAR(g[0].z, seg_of(Light::Green).z + 0.003, 1e-9);
  EXPECT_NEAR(g[1].z, seg_of(Light::Yellow).z + 0.003, 1e-9);
  EXPECT_NEAR(g[2].z, seg_of(Light::Red).z, 1e-9);
}

// U5b: 무작위 1000회 = U1 조건(켜짐 · 꺼짐 0.1~1.0 s, 텀 max(3 × 꺼짐, 1.0)~5.0 s, 아무 때나 보기 시작,
//      잘못된 프레임 2 %, 빠진 프레임 5 %) + 가림 구간(주기당 평균 1개, 길이 0.1~1.0 s, 가린 동안 켜진 칸도 안 보임).
//      퍽(늘 켜짐)이 함께 보임. 통과: 가림 처리에서 틀린 확정 0회.
//      기록만: 10주기 · 5주기 안 정답 확정 수, 가림을 꺼짐으로 세던 기존 방식의 결과
TEST(SignalLight, U5b_OcclusionRandom1000)
{
  struct Tally {int ok = 0, ok5 = 0, wrong = 0, none = 0;};
  Tally tally[2];   // [0] 가림 처리, [1] 가림을 꺼짐으로 셈
  std::mt19937 rng(20261024);
  std::uniform_real_distribution<double> on_off(0.1, 1.0), unit(0, 1), len(0.1, 1.0);
  std::uniform_int_distribution<int> color(0, 2), other(1, 3);
  for (int k = 0; k < 1000; ++k) {
    Blinker b{"", on_off(rng), on_off(rng), 0};
    for (int i = 0; i < 12; ++i) {b.seq += "RYG"[color(rng)];}
    const double lo = std::max(3 * b.t_off, 1.0);
    b.t_term = lo + unit(rng) * (5.0 - lo);
    const double P = b.period(), start = unit(rng) * P, limit = 10 * P;
    // 가림 구간: 지수 분포 간격(평균 P)
    std::vector<std::pair<double, double>> win;
    std::exponential_distribution<double> gap(1.0 / P);
    for (double w = gap(rng); w < limit; w += gap(rng)) {const double L = len(rng); win.push_back({w, w + L});}
    // 프레임마다 빠짐 · 잘못된 상태를 미리 정해 두 방식에 같은 입력을 넣음
    const int n_frames = static_cast<int>(limit * 30) + 1;
    std::vector<int> fate(n_frames);   // 0 정상, 1 빠짐, 2.. 잘못된 상태 오프셋 + 1
    for (auto & x : fate) {x = unit(rng) < 0.05 ? 1 : unit(rng) < 0.02 ? 1 + other(rng) : 0;}
    for (int mode = 0; mode < 2; ++mode) {
      SignalLightLocator loc;
      SignalSequenceTracker tr;
      bool done = false;
      for (int n = 0; n < n_frames && !done; ++n) {
        const double tl = n / 30.0, t = start + tl;
        if (fate[n] == 1) {continue;}
        bool hide = false;
        for (const auto & w : win) {if (tl >= w.first && tl < w.second) {hide = true; break;}}
        Light l = b.at(t);
        if (fate[n] > 1) {l = static_cast<Light>((static_cast<int>(l) + fate[n] - 1) % 4);}
        std::vector<LightObs> obs{at(PUCK, Light::Green)};
        if (l != Light::Off && !hide) {obs.push_back(at(seg_of(l), l));}
        const auto s = mode == 0 ?
          loc.update(t, obs, always, [&](const SignalLightLocator::Segments &) {return hide;}) :
          loc.update(t, obs, always);
        if (!s) {continue;}
        const auto & r = tr.update(t, *s);
        if (r.status == SignalResult::CONFIRMED) {
          done = true;
          Tally & c = tally[mode];
          if (r.sequence != b.seq) {
            ++c.wrong;
            if (mode == 0) {ADD_FAILURE() << k << ": 틀린 확정 " << r.sequence << " (정답 " << b.seq << ")";}
          } else {
            ++c.ok;
            if (tl <= 5 * P) {++c.ok5;}
          }
        }
      }
      if (!done) {++tally[mode].none;}
    }
  }
  const char * names[2] = {"occl", "old"};
  for (int m = 0; m < 2; ++m) {
    RecordProperty(std::string(names[m]) + "_ok10", tally[m].ok);
    RecordProperty(std::string(names[m]) + "_ok5", tally[m].ok5);
    RecordProperty(std::string(names[m]) + "_wrong", tally[m].wrong);
    RecordProperty(std::string(names[m]) + "_none", tally[m].none);
    std::cout << names[m] << ": 10주기 안 정답 " << tally[m].ok << " (5주기 안 " << tally[m].ok5 << "), 틀린 확정 "
              << tally[m].wrong << ", 확정 못 함 " << tally[m].none << std::endl;
  }
  EXPECT_EQ(tally[0].wrong, 0);
}
