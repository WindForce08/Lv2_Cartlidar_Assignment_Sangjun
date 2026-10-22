// 색 검출기 · 기하 계산 시험 (합성 영상)
#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"
#include "perception/geometry.hpp"

using namespace perception;

TEST(ColorDetector, FindsColoredPucks)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  cv::rectangle(img, cv::Rect(100, 200, 60, 60), cv::Scalar(0, 0, 220), cv::FILLED);    // 빨강 (BGR)
  cv::circle(img, {400, 240}, 35, cv::Scalar(0, 200, 0), cv::FILLED);                   // 초록
  cv::rectangle(img, cv::Rect(500, 50, 50, 50), cv::Scalar(220, 60, 0), cv::FILLED);     // 파랑
  DetectorConfig c;
  c.type = "color";
  std::string info;
  auto d = make_detector(c, info);
  ASSERT_TRUE(d);
  const auto dets = d->detect(img);
  ASSERT_EQ(dets.size(), 3u);
  const int g = best_detection(dets, 1);
  ASSERT_GE(g, 0);
  EXPECT_NEAR(dets[g].box.x + dets[g].box.width / 2, 400, 2);
  EXPECT_NEAR(dets[g].box.y + dets[g].box.height / 2, 240, 2);

  c.target_color = "red";
  ColorDetector red(c);
  const auto r = red.detect(img);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].class_id, 0);
  EXPECT_NEAR(r[0].box.width, 60, 2);
}

TEST(ColorDetector, IgnoresSmallAndGray)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(120, 120, 120));
  cv::rectangle(img, cv::Rect(10, 10, 5, 5), cv::Scalar(0, 0, 255), cv::FILLED);   // 너무 작음
  DetectorConfig c;
  ColorDetector d(c);
  EXPECT_TRUE(d.detect(img).empty());
}

TEST(ColorDetector, RejectsLowFillAndUsesConfiguredHsv)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  // 속 빈 빨간 테두리 (채움률 약 0.26) → 거름
  cv::rectangle(img, cv::Rect(100, 100, 120, 120), cv::Scalar(0, 0, 220), 8);
  DetectorConfig c;
  c.target_color = "red";
  EXPECT_TRUE(ColorDetector(c).detect(img).empty());

  // 파랑 범위를 좁히면 같은 파란 퍽이 안 잡힘 (파라미터가 실제로 쓰임)
  cv::rectangle(img, cv::Rect(400, 200, 60, 60), cv::Scalar(220, 60, 0), cv::FILLED);
  c.target_color = "blue";
  EXPECT_EQ(ColorDetector(c).detect(img).size(), 1u);
  for (auto & h : c.hsv) {if (h.color == "blue") {h.lower[0] = 120; h.upper[0] = 130;}}
  EXPECT_TRUE(ColorDetector(c).detect(img).empty());
}

TEST(Geometry, SizePlausible)
{
  const PinholeK k{600, 600, 320, 240};
  // 1 m 거리에서 퍽 30 × 60 mm → 18 × 36 px
  EXPECT_TRUE(size_plausible(cv::Rect2f(0, 0, 18, 36), 1.0, k, 0.015, 0.09));
  EXPECT_FALSE(size_plausible(cv::Rect2f(0, 0, 120, 120), 1.0, k, 0.015, 0.09));   // 20 cm 벽 조각
  EXPECT_FALSE(size_plausible(cv::Rect2f(0, 0, 5, 5), 1.0, k, 0.015, 0.09));       // 8 mm 점
  EXPECT_TRUE(size_plausible(cv::Rect2f(0, 0, 5, 5), 0.0, k, 0.015, 0.09));        // 깊이 없음 → 통과
}

TEST(Geometry, DeprojectAndFrames)
{
  PinholeK k{600, 600, 320, 240};
  // 화면 중심 → 정면
  auto p = optical_to_link(deproject(k, 320, 240, 1.0));
  EXPECT_NEAR(p.x, 1.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.z, 0.0, 1e-12);
  // 화면 오른쪽 · 아래 → 링크 기준 오른쪽(y-) · 아래(z-)
  p = optical_to_link(deproject(k, 380, 300, 2.0));
  EXPECT_NEAR(p.x, 2.0, 1e-12);
  EXPECT_NEAR(p.y, -0.2, 1e-12);
  EXPECT_NEAR(p.z, -0.2, 1e-12);
}

TEST(Geometry, MedianDepthIgnoresHoles)
{
  cv::Mat d(100, 100, CV_16U, cv::Scalar(0));
  d(cv::Rect(45, 45, 11, 11)).setTo(1000);   // 1.000 m
  d.at<uint16_t>(50, 50) = 0;                // 구멍
  d.at<uint16_t>(49, 49) = 9000;             // 튀는 값 (9 m, 범위 밖)
  EXPECT_NEAR(median_depth(d, {50, 50}, 3, 0.001, 0.15, 4.0), 1.0, 1e-9);
  EXPECT_TRUE(std::isnan(median_depth(d, {5, 5}, 2, 0.001, 0.15, 4.0)));
}

TEST(Geometry, ImageOffset)
{
  const auto o = image_offset(cv::Rect2f(600, 300, 80, 120), {1280, 720});
  EXPECT_NEAR(o[0], (640.0 - 640.0) / 640.0, 1e-12);
  EXPECT_NEAR(o[1], (360.0 - 360.0) / 360.0, 1e-12);
  EXPECT_NEAR(o[2], 80.0 / 1280.0, 1e-12);
  const auto r = image_offset(cv::Rect2f(1180, 620, 100, 100), {1280, 720});
  EXPECT_GT(r[0], 0.8);    // 오른쪽
  EXPECT_GT(r[1], 0.8);    // 아래
}
