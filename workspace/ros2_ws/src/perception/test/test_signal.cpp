// 신호등 점멸 순서 세기 시험 (판정 기준안 v1 U1 · U2, 영상 없음)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>

#include "perception/signal_sequence.hpp"

using namespace perception;

namespace
{
constexpr double FPS = 30.0;

Light from_char(char c)
{
  return c == 'R' ? Light::Red : c == 'Y' ? Light::Yellow : c == 'G' ? Light::Green : Light::Off;
}

// 반복하는 신호등: 한 주기 = 12회 점멸(켜짐 t_on, 사이 꺼짐 t_off) + 텀
struct Blinker
{
  std::string seq;
  double t_on, t_off, t_term;
  double period() const {return seq.size() * t_on + (seq.size() - 1) * t_off + t_term;}
  Light at(double t) const
  {
    double u = std::fmod(t, period());
    for (char c : seq) {
      if (u < t_on) {return from_char(c);}
      u -= t_on;
      if (u < t_off) {return Light::Off;}
      u -= t_off;
    }
    return Light::Off;
  }
};

struct Watch
{
  bool confirmed = false;
  std::string sequence;
  double elapsed = 0;     // 보기 시작부터 확정까지 [s]
};

// start: 신호등 시각 기준 보기 시작, limit: 볼 시간. drop · glitch: 프레임 빠짐 · 잘못된 상태 확률
Watch watch(const Blinker & b, double start, double limit, std::mt19937 & rng, double drop = 0, double glitch = 0)
{
  SignalSequenceTracker tr;
  std::uniform_real_distribution<double> u(0, 1);
  std::uniform_int_distribution<int> other(1, 3);
  Watch r;
  for (int n = 0; n / FPS <= limit; ++n) {
    const double t = start + n / FPS;
    if (u(rng) < drop) {continue;}
    Light s = b.at(t);
    if (u(rng) < glitch) {
      s = static_cast<Light>((static_cast<int>(s) + other(rng)) % 4);
    }
    const auto & res = tr.update(t, s);
    if (res.status == SignalResult::CONFIRMED) {
      r.confirmed = true;
      r.sequence = res.sequence;
      r.elapsed = n / FPS;
      return r;
    }
  }
  return r;
}

std::string random_seq(std::mt19937 & rng)
{
  std::uniform_int_distribution<int> c(0, 2);
  std::string s;
  for (int i = 0; i < 12; ++i) {s += "RYG"[c(rng)];}
  return s;
}
}  // namespace

// U1: 무작위 1000회. 켜짐 · 꺼짐 0.1~1.0 s, 텀 max(3 × 꺼짐, 1.0)~5.0 s, 아무 때나 보기 시작,
//     잘못된 프레임 2 %, 빠진 프레임 5 %. 통과: 1000/1000 일치, 틀린 확정 0, 5주기 안에 확정
//     (켜짐 · 꺼짐이 몇 프레임뿐이면 잡음으로 점멸 정보 자체가 사라지는 묶음이 생겨 다음 묶음을 기다린다)
TEST(SignalSequence, U1_Random1000)
{
  std::mt19937 rng(20261023);
  std::uniform_real_distribution<double> on_off(0.1, 1.0), unit(0, 1);
  int ok = 0, wrong = 0, late = 0;
  for (int k = 0; k < 1000; ++k) {
    Blinker b{random_seq(rng), on_off(rng), on_off(rng), 0};
    const double lo = std::max(3 * b.t_off, 1.0);
    b.t_term = lo + unit(rng) * (5.0 - lo);
    const double P = b.period();
    const Watch r = watch(b, unit(rng) * P, 5 * P, rng, 0.05, 0.02);
    if (!r.confirmed) {
      ++late;
      ADD_FAILURE() << k << ": 5주기 안에 확정 못 함 " << b.seq << " on " << b.t_on << " off " << b.t_off
                    << " term " << b.t_term;
    } else if (r.sequence != b.seq) {
      ++wrong;
      ADD_FAILURE() << k << ": 틀린 확정 " << r.sequence << " (정답 " << b.seq << ")";
    } else {
      ++ok;
    }
  }
  RecordProperty("ok", ok);
  RecordProperty("wrong", wrong);
  RecordProperty("late", late);
  EXPECT_EQ(ok, 1000) << "틀린 확정 " << wrong << ", 확정 못 함 " << late;
}

// U2: 경계 상황
TEST(SignalSequence, U2_SameColorRuns)
{
  std::mt19937 rng(1);
  for (const std::string seq : {"RRRRRRRRRRRR", "RRGGYYRRGGYY", "GGGGGGRRRRYY", "YRRRRRRRRRRR"}) {
    Blinker b{seq, 0.3, 0.2, 1.5};
    const Watch r = watch(b, 0.05, 3 * b.period(), rng);
    EXPECT_TRUE(r.confirmed) << seq;
    EXPECT_EQ(r.sequence, seq);
  }
}

TEST(SignalSequence, U2_StartInsideOnAndInsideTerm)
{
  std::mt19937 rng(2);
  Blinker b{"RGRRGRGGYGYG", 0.4, 0.3, 2.0};
  const double P = b.period();
  // 확정 = 두 번째 묶음의 마지막 점멸 뒤 텀 판정(꺼짐 중앙값 × 2)까지
  // 첫 점멸이 켜진 중간에 보기 시작 → 첫 점멸도 색은 보이므로 첫 묶음부터 셈
  Watch r = watch(b, 0.2, 3 * P, rng);
  EXPECT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, b.seq);
  EXPECT_LT(r.elapsed, 2 * P - b.t_term + 2 * b.t_off + 0.1);
  // 텀 중간에 보기 시작 → 첫 묶음부터 점멸 수가 맞아 두 묶음째 끝(텀 판정 시간 포함)에 확정
  r = watch(b, P - b.t_term + 0.1, 3 * P, rng);
  EXPECT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, b.seq);
  EXPECT_LT(r.elapsed, 2 * P + 2 * b.t_off + 0.1);
}

TEST(SignalSequence, U2_DifferentCyclesDoNotConfirm)
{
  // 묶음 A, B, A 순서로 바뀌는 신호등: A · B 만 봤을 때는 확정하지 않고(서로 다름), A 가 두 번 나오면 확정
  const std::string A = "RGRRGRGGYGYG", B = "GGGGRRRRYYYY";
  Blinker a{A, 0.3, 0.2, 1.5}, bb{B, 0.3, 0.2, 1.5};
  const double P = a.period();
  SignalSequenceTracker tr;
  bool saw_mismatch = false;
  for (int n = 0; n / FPS < 4 * P; ++n) {
    const double t = n / FPS;
    const int cycle = static_cast<int>(t / P);
    const Light s = (cycle == 1 ? bb : a).at(t);
    const auto & res = tr.update(t, s);
    if (t < 3 * P - a.t_term) {
      ASSERT_NE(res.status, SignalResult::CONFIRMED) << t;     // 세 번째 묶음(A)이 끝나기 전
    }
    saw_mismatch = saw_mismatch || res.status == SignalResult::MISMATCH;
  }
  EXPECT_TRUE(saw_mismatch);
  EXPECT_EQ(tr.result().status, SignalResult::CONFIRMED);
  EXPECT_EQ(tr.result().sequence, A);
}

TEST(SignalSequence, U2_LostFramesKeepsCompletedCycles)
{
  // 첫 묶음을 다 본 뒤 1 s 동안 신호등을 놓침 → 놓친 묶음은 버리고 그다음 묶음과 맞춰 확정
  Blinker b{"RGRRGRGGYGYG", 0.3, 0.3, 2.0};
  const double P = b.period();
  SignalSequenceTracker tr;
  const double lost_from = P + 1.0;   // 두 번째 묶음 중간
  for (int n = 0; n / FPS < 4 * P; ++n) {
    const double t = (P - b.t_term + 0.1) + n / FPS;     // 텀 중간부터 봄
    if (t > lost_from && t < lost_from + 1.0) {continue;}
    tr.update(t, b.at(t));
    if (tr.result().status == SignalResult::CONFIRMED) {break;}
  }
  EXPECT_EQ(tr.result().status, SignalResult::CONFIRMED);
  EXPECT_EQ(tr.result().sequence, b.seq);
}

TEST(SignalSequence, U2_NothingSeen)
{
  SignalSequenceTracker tr;
  EXPECT_EQ(tr.result().status, SignalResult::SEARCHING);
  for (int n = 0; n < 300; ++n) {tr.update(n / FPS, Light::Off);}
  EXPECT_EQ(tr.result().status, SignalResult::SEARCHING);
  EXPECT_TRUE(tr.result().sequence.empty());
}

TEST(SignalSequence, U2_MeasuredTimesAndCounts)
{
  std::mt19937 rng(3);
  Blinker b{"RGRRGRGGYGYG", 0.5, 0.25, 2.0};
  SignalSequenceTracker tr;
  for (int n = 0; n / FPS < 3 * b.period(); ++n) {
    tr.update(n / FPS, b.at(n / FPS));
  }
  const auto & r = tr.result();
  ASSERT_EQ(r.status, SignalResult::CONFIRMED);
  EXPECT_EQ(r.red, 4);
  EXPECT_EQ(r.yellow, 2);
  EXPECT_EQ(r.green, 6);
  // 잰 길이: 프레임 경계만큼 어긋날 수 있음. 텀은 확정 뒤 다음 점멸이 끝나야 알 수 있다
  EXPECT_NEAR(r.t_on, b.t_on, 1.5 / FPS);
  EXPECT_NEAR(r.t_off, b.t_off, 1.5 / FPS);
  EXPECT_NEAR(r.t_term, b.t_term, 1.5 / FPS);
}

// 텀 길이: 확정 순간에는 아직 모를 수 있지만, 그 뒤 다음 점멸을 보면 채워져야 한다(확정 뒤에도 갱신).
// U1 과 같은 범위 · 잡음으로 200회, 5주기 본 뒤 잰 텀. 통과(사용자 2026-10-24): 99 % 이상이 정답과 3프레임 안,
// 나머지도 0.5 s 안. 100 % 가 아닌 이유: 텀 끝에 같은 색 잘못된 프레임이 2장 겹치면(3프레임 다수결을 넘음) 다음 점멸에
// 합쳐져 텀이 짧게 재지는 경우가 있다(첫 실행 199/200, 0.46 s). 텀은 순서 확정에 쓰지 않는 참고 값이다
TEST(SignalSequence, U2_TermMeasuredAfterConfirm)
{
  std::mt19937 rng(20261024);
  std::uniform_real_distribution<double> on_off(0.1, 1.0), unit(0, 1);
  std::uniform_int_distribution<int> other(1, 3);
  int ok = 0, near = 0;
  for (int k = 0; k < 200; ++k) {
    Blinker b{random_seq(rng), on_off(rng), on_off(rng), 0};
    const double lo = std::max(3 * b.t_off, 1.0);
    b.t_term = lo + unit(rng) * (5.0 - lo);
    const double P = b.period(), start = unit(rng) * P;
    SignalSequenceTracker tr;
    for (int n = 0; n / FPS <= 5 * P; ++n) {
      if (unit(rng) < 0.05) {continue;}
      Light s = b.at(start + n / FPS);
      if (unit(rng) < 0.02) {s = static_cast<Light>((static_cast<int>(s) + other(rng)) % 4);}
      tr.update(start + n / FPS, s);
    }
    const auto & r = tr.result();
    const double err = std::abs(r.t_term - b.t_term);
    if (r.status == SignalResult::CONFIRMED && err <= 3.0 / FPS) {
      ++ok;
    } else if (r.status == SignalResult::CONFIRMED && err <= 0.5) {
      ++near;
      std::cout << k << ": 텀 " << r.t_term << " (정답 " << b.t_term << ") on " << b.t_on << " off " << b.t_off << "\n";
    } else {
      ADD_FAILURE() << k << ": 상태 " << int(r.status) << " 텀 " << r.t_term << " (정답 " << b.t_term
                    << ") on " << b.t_on << " off " << b.t_off;
    }
  }
  RecordProperty("within_3_frames", ok);
  RecordProperty("within_0_5s", near);
  EXPECT_GE(ok, 198) << "3프레임 안 " << ok << "/200";
}
