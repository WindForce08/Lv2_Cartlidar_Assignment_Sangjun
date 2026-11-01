// YOLO 엔진 출력 후처리 시험 (yolo_postprocess.hpp, TensorRT 없이)
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "perception/yolo_postprocess.hpp"

using namespace perception;

namespace
{
constexpr int NC = 4;            // red_cylinder · green_cylinder · red_cuboid · green_cuboid
constexpr int C = 4 + NC;

struct Cand
{
  float cx, cy, w, h;
  int cls;
  float score;
};

// 채널 우선(C × N) 엔진 출력 만들기
std::vector<float> raw_of(const std::vector<Cand> & cs)
{
  const int n = static_cast<int>(cs.size());
  std::vector<float> raw(static_cast<size_t>(C) * n, 0.f);
  for (int i = 0; i < n; ++i) {
    raw[i] = cs[i].cx;
    raw[n + i] = cs[i].cy;
    raw[2 * n + i] = cs[i].w;
    raw[3 * n + i] = cs[i].h;
    raw[static_cast<size_t>(4 + cs[i].cls) * n + i] = cs[i].score;
  }
  return raw;
}

std::vector<Detection> decode(const std::vector<Cand> & cs, int target_class = -1)
{
  return decode_yolo(raw_of(cs), C, static_cast<int>(cs.size()), 1.f, 0, 0, 640, 640, 0.25f, 0.45f, target_class);
}
}  // namespace

// 같은 클래스가 겹치면 점수 높은 것 하나, 다른 클래스는 겹쳐도 둘 다 남음(클래스마다 NMS)
TEST(YoloPostprocess, PerClassNms)
{
  const auto d = decode({{100, 100, 40, 40, 0, 0.9f}, {102, 101, 40, 40, 0, 0.8f}, {101, 100, 40, 40, 2, 0.7f}});
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0].class_id, 0);
  EXPECT_FLOAT_EQ(d[0].score, 0.9f);
  EXPECT_EQ(d[1].class_id, 2);   // 점수 순
}

// 레터박스를 실수로 되돌림: 1280×720 → 640 (scale 0.5, pad_y 140)
TEST(YoloPostprocess, LetterboxBackToImageWithoutRounding)
{
  const auto raw = raw_of({{320.25f, 320.75f, 20.5f, 10.5f, 1, 0.6f}});
  const auto d = decode_yolo(raw, C, 1, 0.5f, 0, 140, 1280, 720, 0.25f, 0.45f);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_NEAR(d[0].box.x, (320.25f - 10.25f) / 0.5f, 1e-3);
  EXPECT_NEAR(d[0].box.y, (320.75f - 5.25f - 140) / 0.5f, 1e-3);
  EXPECT_NEAR(d[0].box.width, 41.f, 1e-3);
  EXPECT_NEAR(d[0].box.height, 21.f, 1e-3);
  EXPECT_EQ(d[0].class_id, 1);
}

// 점수 하한 · target_class · 화면 밖 상자는 잘라 내고, 완전히 밖이면 버림
// 문턱과 같은 점수는 버림(cv::dnn::NMSBoxes 가 문턱보다 큰 것만 남김)
TEST(YoloPostprocess, ThresholdTargetAndClip)
{
  EXPECT_TRUE(decode({{100, 100, 20, 20, 0, 0.2f}}).empty());
  EXPECT_TRUE(decode({{100, 100, 20, 20, 0, 0.25f}}).empty());
  EXPECT_EQ(decode({{100, 100, 20, 20, 0, 0.26f}}).size(), 1u);
  EXPECT_TRUE(decode({{100, 100, 20, 20, 0, 0.9f}}, 3).empty());
  EXPECT_EQ(decode({{100, 100, 20, 20, 3, 0.9f}}, 3).size(), 1u);
  const auto clip = decode({{5, 100, 20, 20, 0, 0.9f}});
  ASSERT_EQ(clip.size(), 1u);
  EXPECT_FLOAT_EQ(clip[0].box.x, 0.f);
  EXPECT_FLOAT_EQ(clip[0].box.width, 15.f);
  EXPECT_TRUE(decode({{700, 100, 20, 20, 0, 0.9f}}).empty());
}

// 유한하지 않은 값 · 1 넘는 점수 · 크기 0 상자는 버림, 잘못된 크기 입력은 빈 결과
TEST(YoloPostprocess, RejectsBadValues)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(decode({{nan, 100, 20, 20, 0, 0.9f}}).empty());
  EXPECT_TRUE(decode({{100, 100, 0, 20, 0, 0.9f}}).empty());
  EXPECT_TRUE(decode({{100, 100, 20, 20, 0, 1.5f}}).empty());
  EXPECT_TRUE(decode({{100, 100, 20, 20, 0, nan}}).empty());
  const auto raw = raw_of({{100, 100, 20, 20, 0, 0.9f}});
  EXPECT_TRUE(decode_yolo(raw, C, 2, 1.f, 0, 0, 640, 640, 0.25f, 0.45f).empty());   // 크기 안 맞음
  EXPECT_TRUE(decode_yolo(raw, 4, 2, 1.f, 0, 0, 640, 640, 0.25f, 0.45f).empty());   // 클래스 없음
  EXPECT_TRUE(decode_yolo(raw, C, 1, 0.f, 0, 0, 640, 640, 0.25f, 0.45f).empty());   // scale 0
}

// 최대 300개
TEST(YoloPostprocess, AtMost300)
{
  std::vector<Cand> cs;
  for (int i = 0; i < 400; ++i) {
    cs.push_back({static_cast<float>(10 + (i % 20) * 30), static_cast<float>(10 + (i / 20) * 30), 10, 10, 0,
        0.3f + 0.001f * i});
  }
  const auto d = decode(cs);
  ASSERT_EQ(d.size(), 300u);
  EXPECT_NEAR(d.front().score, 0.3f + 0.399f, 1e-6);   // 점수 높은 것부터
}
