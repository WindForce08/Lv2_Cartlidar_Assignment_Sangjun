// 색 덩어리 깊이 나누기 시험 (depth_split.hpp, 합성 영상)
#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "perception/depth_split.hpp"

using namespace perception;

namespace
{
const PinholeK K{600.0, 600.0, 320.0, 240.0};

std::vector<Detection> detect(const cv::Mat & img)
{
  DetectorConfig c;
  c.type = "color";
  c.blur_ksize = 3;
  c.morph_kernel = 3;
  std::string info;
  return make_detector(c, info)->detect(img);
}

std::vector<DepthPiece> split(const cv::Mat & img, const cv::Mat & depth)
{
  DetectorConfig c;
  const auto dets = detect(img);
  return split_by_depth(dets, img, depth, 1.0, c.hsv, 3, 3, K, DepthSplitConfig{}, 0.3f);
}
}  // namespace

// 같은 색 퍽 둘이 화면에서 맞닿고 깊이가 5 cm 다름 → 두 조각, 중심 · 깊이는 각 퍽
TEST(DepthSplit, TouchingSameColorAtDifferentDepths)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(2.0f));
  cv::rectangle(img, cv::Rect(300, 200, 30, 60), cv::Scalar(0, 0, 220), cv::FILLED);
  cv::rectangle(img, cv::Rect(330, 200, 30, 60), cv::Scalar(0, 0, 220), cv::FILLED);
  depth(cv::Rect(300, 200, 30, 60)).setTo(0.60f);
  depth(cv::Rect(330, 200, 30, 60)).setTo(0.65f);
  ASSERT_EQ(detect(img).size(), 1u);   // 색 검출기는 한 덩어리로 냄
  auto p = split(img, depth);
  ASSERT_EQ(p.size(), 2u);
  std::sort(p.begin(), p.end(), [](const auto & a, const auto & b) {return a.center.x < b.center.x;});
  EXPECT_NEAR(p[0].center.x, 314.5, 1.5);
  EXPECT_NEAR(p[1].center.x, 344.5, 1.5);
  EXPECT_NEAR(p[0].depth, 0.60, 1e-6);
  EXPECT_NEAR(p[1].depth, 0.65, 1e-6);
  EXPECT_EQ(p[0].det.class_id, 0);
  EXPECT_NEAR(p[0].det.box.width, 30, 2);
}

// 퍽 하나: 면이 비스듬해 깊이가 화소마다 조금씩 변해도(1 mm/px) 한 조각
TEST(DepthSplit, SlopedSingleFaceStaysWhole)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(2.0f));
  cv::rectangle(img, cv::Rect(300, 200, 30, 60), cv::Scalar(0, 200, 0), cv::FILLED);
  for (int y = 200; y < 260; ++y) {depth.row(y).colRange(300, 330).setTo(0.6f + 0.001f * (y - 200));}
  const auto p = split(img, depth);
  ASSERT_EQ(p.size(), 1u);
  EXPECT_EQ(p[0].det.class_id, 1);
  EXPECT_NEAR(p[0].center.y, 229.5, 1.5);
}

// 깊이가 없으면 조각도 없음 (기존 방식처럼 위치를 낼 수 없음)
TEST(DepthSplit, NoDepthNoPiece)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(0.0f));
  cv::rectangle(img, cv::Rect(300, 200, 30, 60), cv::Scalar(0, 0, 220), cv::FILLED);
  EXPECT_TRUE(split(img, depth).empty());
}

// 다른 퍽에 가려 남은 작은 조각(그 깊이 30 mm 면의 30 % 미만)은 버리고, 나누기는 원본 영상을 바꾸지 않음
TEST(DepthSplit, DropsSmallPieceAndKeepsImage)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(2.0f));
  cv::rectangle(img, cv::Rect(300, 200, 34, 60), cv::Scalar(0, 0, 220), cv::FILLED);
  depth(cv::Rect(300, 200, 34, 60)).setTo(0.60f);
  depth(cv::Rect(330, 200, 4, 30)).setTo(0.70f);   // 뒤 퍽 조각 4 × 30 = 120 px < 0.3 × (600 · 0.03 / 0.7)² ≈ 198 px
  const cv::Mat before = img.clone();
  const auto p = split(img, depth);
  ASSERT_EQ(p.size(), 1u);
  EXPECT_NEAR(p[0].depth, 0.60, 1e-6);
  EXPECT_EQ(cv::norm(before, img, cv::NORM_INF), 0.0);
}

// 상자 두 개가 같은 덩어리를 덮어도 조각은 한 번만
TEST(DepthSplit, OverlappingBoxesNoDuplicate)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(0.6f));
  cv::rectangle(img, cv::Rect(300, 200, 30, 60), cv::Scalar(0, 0, 220), cv::FILLED);
  auto dets = detect(img);
  ASSERT_EQ(dets.size(), 1u);
  dets.push_back(dets[0]);
  dets.back().box.x -= 5;
  dets.back().box.width += 10;
  DetectorConfig c;
  EXPECT_EQ(split_by_depth(dets, img, depth, 1.0, c.hsv, 3, 3, K, DepthSplitConfig{}, 0.3f).size(), 1u);
}
