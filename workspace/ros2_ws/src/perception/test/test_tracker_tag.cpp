// 대상 추적 · AprilTag 시험
#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/tag_depth.hpp"
#include "perception/tag_detector.hpp"
#include "perception/target_tracker.hpp"

using namespace perception;

static Candidate cand(double x, double y, double conf, double ox = NAN, double oy = NAN)
{
  Candidate c;
  c.base = {x, y, 0.0};
  c.odom = {std::isnan(ox) ? x : ox, std::isnan(oy) ? y : oy, 0.0};
  c.confidence = conf;
  return c;
}

TEST(Tracker, PicksClosestOutsideExcludeAboveConfidence)
{
  TargetTracker t;
  auto r = t.update({cand(1.5, 0.0, 0.9), cand(0.8, 0.3, 0.9), cand(0.2, 0.0, 0.99), cand(0.6, 0.0, 0.3)}, 0.0);
  ASSERT_TRUE(r);   // 0.2 m 는 exclude(차체), 0.6 m 는 신뢰도 낮음 → 0.8 m
  EXPECT_DOUBLE_EQ(r->base.x, 0.8);
}

TEST(Tracker, StaysOnSameObjectWithinGateAndReacquiresAfterTimeout)
{
  TargetTracker t;
  ASSERT_TRUE(t.update({cand(1.0, 0.0, 0.9)}, 0.0));
  // 더 가까운 다른 물체가 나타나도 직전 위치 0.3 m 안의 것을 따라감
  auto r = t.update({cand(0.7, 0.5, 0.9), cand(0.95, 0.05, 0.9)}, 0.1);
  ASSERT_TRUE(r);
  EXPECT_DOUBLE_EQ(r->base.x, 0.95);
  // 대상이 사라지면 gate 밖 물체는 고르지 않음
  EXPECT_FALSE(t.update({cand(0.7, 0.5, 0.9)}, 0.5));
  // 1 s 넘게 안 보이면 다시 고름
  r = t.update({cand(0.7, 0.5, 0.9)}, 1.2);
  ASSERT_TRUE(r);
  EXPECT_DOUBLE_EQ(r->base.x, 0.7);
}

TEST(Tracker, GateUsesOdomSoRobotMotionDoesNotBreakTracking)
{
  TargetTracker t;
  ASSERT_TRUE(t.update({cand(1.0, 0.0, 0.9, 3.0, 2.0)}, 0.0));
  // 로봇이 0.5 m 전진: base 기준은 0.5 m 로 바뀌지만 odom 위치는 같음
  auto r = t.update({cand(0.5, 0.0, 0.9, 3.0, 2.0), cand(0.9, 0.2, 0.9, 3.4, 2.2)}, 0.2);
  ASSERT_TRUE(r);
  EXPECT_DOUBLE_EQ(r->base.x, 0.5);
  t.reset();
  EXPECT_FALSE(t.locked());
}

TEST(TagDetector, DetectsAprilTag36h11WithPose)
{
  const int id = 7, px = 200;
  cv::Mat marker;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, px, marker, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, px, marker, 1);
#endif
  cv::Mat img(720, 1280, CV_8UC1, cv::Scalar(255));
  marker.copyTo(img(cv::Rect(540, 260, px, px)));   // 화면 중앙
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  const double f = 900.0;
  cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, 640, 0, f, 360, 0, 0, 1);
  TagDetector d("tag36h11", 0.10);
  ASSERT_TRUE(d.ok());
  const auto tags = d.detect(bgr, K, cv::Mat());
  ASSERT_EQ(tags.size(), 1u);
  EXPECT_EQ(tags[0].id, id);
  ASSERT_TRUE(tags[0].pose_valid);
  // 태그 바깥 테두리 포함 200 px · 안쪽 마커 크기 비율 → 거리 대략 f·size/px 근처 (0.3~0.6 m)
  EXPECT_GT(tags[0].tvec[2], 0.3);
  EXPECT_LT(tags[0].tvec[2], 0.6);
  EXPECT_NEAR(tags[0].tvec[0], 0.0, 0.02);
  EXPECT_NEAR(tags[0].tvec[1], 0.0, 0.02);
}

// 작은 태그(약 22 px): 2배로 키워 다시 찾는 경우를 포함해 번호와 꼭짓점(원래 크기 화소)이 맞아야 한다
TEST(TagDetector, SmallTagFoundWithCornersInOriginalPixels)
{
  const int id = 13, px = 22;
  cv::Mat big, marker;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 220, big, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, 220, big, 1);
#endif
  cv::resize(big, marker, cv::Size(px, px), 0, 0, cv::INTER_AREA);
  cv::Mat img(480, 640, CV_8UC1, cv::Scalar(200));
  img(cv::Rect(300, 220, px + 8, px + 8)).setTo(255);   // 흰 여백
  marker.copyTo(img(cv::Rect(304, 224, px, px)));
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  TagDetector d("tag36h11", 0.048);
  const auto tags = d.detect(bgr, cv::Mat(), cv::Mat());
  ASSERT_EQ(tags.size(), 1u);
  EXPECT_EQ(tags[0].id, id);
  // 바깥 검은 테두리 = 마커 화소 [304, 326) 가로 · [224, 246) 세로 → 꼭짓점은 그 테두리 근처
  for (const auto & c : tags[0].corners) {
    EXPECT_TRUE(c.x > 302 && c.x < 328 && c.y > 222 && c.y < 248) << c;
  }
}

// 정렬 깊이로 태그 거리 맞추기: 사각형 안 깊이 1.0 m, 바깥 3.0 m. PnP 거리 0.95 → 1.0, x · y 도 같은 비율
TEST(TagDepth, ScalesRangeToDepthInsideQuad)
{
  cv::Mat depth(480, 640, CV_32F, cv::Scalar(3.0f));
  depth(cv::Rect(300, 200, 31, 31)).setTo(1.0f);
  const std::vector<cv::Point2f> q = {{300, 200}, {330, 200}, {330, 230}, {300, 230}};
  cv::Vec3d t(0.05, 0.02, 0.95);
  ASSERT_TRUE(scale_tag_to_depth(t, q, depth, 1.0, 0.15, 4.0));
  EXPECT_NEAR(t[2], 1.0, 1e-9);
  EXPECT_NEAR(t[0], 0.05 / 0.95, 1e-9);
  EXPECT_NEAR(t[1], 0.02 / 0.95, 1e-9);
  // 깊이가 비면(0) 그대로
  cv::Vec3d t2(0.05, 0.02, 0.95);
  cv::Mat empty(480, 640, CV_32F, cv::Scalar(0.0f));
  EXPECT_FALSE(scale_tag_to_depth(t2, q, empty, 1.0, 0.15, 4.0));
  EXPECT_NEAR(t2[2], 0.95, 1e-12);
}
