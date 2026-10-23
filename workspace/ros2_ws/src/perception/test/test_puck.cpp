// 태그 → 퍽 추정 · Aruco 마커 시험 (카메라를 정면으로 보는 합성 영상)
#include <gtest/gtest.h>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/puck_estimator.hpp"

using namespace perception;

namespace
{
constexpr double F = 900.0, CX = 640.0, CY = 360.0, Z = 0.25;   // 카메라에서 면까지 [m]
constexpr double PPM = F / Z;                                   // 면 위 1 m 의 화소 수
const cv::Scalar RED(30, 30, 200), GREEN(40, 160, 40), BLUE(200, 60, 20);

cv::Mat K() {return (cv::Mat_<double>(3, 3) << F, 0, CX, 0, F, CY, 0, 0, 1);}

cv::Mat marker_image(int dict, int id, int px)
{
  cv::Mat m;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(dict), id, px, m, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(dict), id, px, m, 1);
#endif
  return m;
}

cv::Rect centered(double w_m, double h_m)
{
  const int w = static_cast<int>(std::lround(w_m * PPM)), h = static_cast<int>(std::lround(h_m * PPM));
  return {static_cast<int>(CX) - w / 2, static_cast<int>(CY) - h / 2, w, h};
}

// 회색 바탕 위 색 면(w × h m) 가운데에 흰 여백 1칸 포함 tag36h11 (검은 테두리 바깥 0.018 m)
cv::Mat face(double w_m, double h_m, const cv::Scalar & color, int id)
{
  cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(128, 128, 128));
  img(centered(w_m, h_m)).setTo(color);
  img(centered(0.018 * 10 / 8, 0.018 * 10 / 8)).setTo(cv::Scalar(255, 255, 255));
  const cv::Rect r = centered(0.018, 0.018);
  cv::Mat tag;
  cv::cvtColor(marker_image(cv::aruco::DICT_APRILTAG_36h11, id, r.width), tag, cv::COLOR_GRAY2BGR);
  tag.copyTo(img(r));
  return img;
}

PuckEstimate run(const cv::Mat & img, int id, TagDetection & t)
{
  TagDetector d("tag36h11", 0.018);
  const auto tags = d.detect(img, K(), cv::Mat());
  EXPECT_EQ(tags.size(), 1u);
  if (tags.empty()) {return {};}
  t = tags[0];
  EXPECT_EQ(t.id, id);
  EXPECT_TRUE(t.pose_valid);
  EXPECT_NEAR(t.tvec[2], Z, 0.005);
  return estimate_puck(img, K(), t, default_hsv(), PuckGeometry{});
}
}  // namespace

TEST(Puck, ClassifyHsv)
{
  EXPECT_EQ(classify_hsv(cv::Vec3b(0, 200, 200), default_hsv()), 0);
  EXPECT_EQ(classify_hsv(cv::Vec3b(175, 200, 200), default_hsv()), 0);
  EXPECT_EQ(classify_hsv(cv::Vec3b(60, 200, 200), default_hsv()), 1);
  EXPECT_EQ(classify_hsv(cv::Vec3b(110, 220, 200), default_hsv()), 2);
  EXPECT_EQ(classify_hsv(cv::Vec3b(0, 0, 128), default_hsv()), -1);   // 회색
}

TEST(Puck, SideFaceLongAxisVertical)
{
  TagDetection t;
  const auto e = run(face(0.03, 0.06, RED, 3), 3, t);
  EXPECT_EQ(e.color_cls, 0);
  EXPECT_GT(e.confidence, 0.5);
  ASSERT_TRUE(e.orientation_valid);
  EXPECT_FALSE(e.end_face);
  EXPECT_NEAR(e.center[2], t.tvec[2] + 0.015, 0.001);   // 옆면: 법선 반대로 반폭
  EXPECT_GT(std::abs(e.axes(1, 2)), 0.95);              // 긴 축 = 화면 세로(광학 y)
  EXPECT_GT(-e.axes(2, 0), 0.95);                       // x = 보이는 면 바깥 법선 = 카메라 쪽
}

TEST(Puck, SideFaceLongAxisHorizontal)
{
  TagDetection t;
  const auto e = run(face(0.06, 0.03, GREEN, 5), 5, t);
  EXPECT_EQ(e.color_cls, 1);
  ASSERT_TRUE(e.orientation_valid);
  EXPECT_FALSE(e.end_face);
  EXPECT_GT(std::abs(e.axes(0, 2)), 0.95);              // 긴 축 = 화면 가로(광학 x)
}

TEST(Puck, EndFace)
{
  TagDetection t;
  const auto e = run(face(0.03, 0.03, BLUE, 11), 11, t);
  EXPECT_EQ(e.color_cls, 2);
  ASSERT_TRUE(e.orientation_valid);
  EXPECT_TRUE(e.end_face);
  EXPECT_NEAR(e.center[2], t.tvec[2] + 0.03, 0.001);    // 끝면: 법선 반대로 반길이
  EXPECT_GT(std::abs(e.axes(2, 2)), 0.95);              // 긴 축 = 시선 방향
}

TEST(Puck, WhiteFaceIsCylinderCap)
{
  TagDetection t;   // 면 전체가 흰 스티커(원기둥 끝면 가정) → 끝면 · 원기둥, 색은 바깥 회색이라 모름
  const auto e = run(face(0.03, 0.03, cv::Scalar(240, 240, 240), 0), 0, t);
  EXPECT_TRUE(e.cylinder_cap);
  EXPECT_EQ(e.color_cls, -1);
  ASSERT_TRUE(e.orientation_valid);
  EXPECT_TRUE(e.end_face);
  EXPECT_NEAR(e.center[2], t.tvec[2] + 0.03, 0.001);
}

TEST(Puck, WhiteDiscOnGrayIsCylinderCap)
{
  TagDetection t;   // 지름 30 mm 흰 원판(원기둥 끝면)을 위에서 봄. 정사각 띠 모서리는 원판 밖 회색
  cv::Mat img = face(0.0, 0.0, cv::Scalar(128, 128, 128), 4);
  cv::Mat disc(img.size(), CV_8UC3, cv::Scalar(128, 128, 128));
  cv::circle(disc, cv::Point(static_cast<int>(CX), static_cast<int>(CY)), static_cast<int>(0.015 * PPM), cv::Scalar(235, 235, 235), -1);
  const cv::Rect m = centered(0.018 * 10 / 8, 0.018 * 10 / 8);
  img(m).copyTo(disc(m));
  const auto e = run(disc, 4, t);
  EXPECT_TRUE(e.cylinder_cap);
  EXPECT_TRUE(e.end_face);
  EXPECT_NEAR(e.center[2], t.tvec[2] + 0.03, 0.001);
}

TEST(Puck, NoColorNoCapMeansNoOrientation)
{
  TagDetection t;   // 태그 둘레가 회색 바탕 → 색 모름 · 스티커도 아님 → 자세 모름, 옆면 깊이
  const auto e = run(face(0.0, 0.0, cv::Scalar(128, 128, 128), 0), 0, t);
  EXPECT_FALSE(e.cylinder_cap);
  EXPECT_EQ(e.color_cls, -1);
  EXPECT_FALSE(e.orientation_valid);
  EXPECT_NEAR(e.center[2], t.tvec[2] + 0.015, 0.001);
}

TEST(Marker, DetectsAruco4x4)
{
  cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Rect r = centered(0.10 * Z, 0.10 * Z);   // 0.10 m 마커를 1 m 에서 본 크기
  cv::Mat m;
  cv::cvtColor(marker_image(cv::aruco::DICT_4X4_50, 10, r.width), m, cv::COLOR_GRAY2BGR);
  m.copyTo(img(r));
  TagDetector d("DICT_4X4_50", 0.10);
  ASSERT_TRUE(d.ok());
  const auto tags = d.detect(img, K(), cv::Mat());
  ASSERT_EQ(tags.size(), 1u);
  EXPECT_EQ(tags[0].id, 10);
  ASSERT_TRUE(tags[0].pose_valid);
  EXPECT_NEAR(tags[0].tvec[2], 1.0, 0.02);
  EXPECT_EQ(tags[0].corners.size(), 4u);
}
