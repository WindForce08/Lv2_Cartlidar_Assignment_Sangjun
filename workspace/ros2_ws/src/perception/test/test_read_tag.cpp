// READ_TAG 퍽 태그 · 장소 태그 고르기 시험 (read_tag.hpp). 정면 합성 영상
#include <gtest/gtest.h>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/read_tag.hpp"

using namespace perception;

namespace
{
constexpr double F = 900.0, Z = 0.30;   // 카메라에서 태그 면까지 [m]
const std::vector<int> PUCK_IDS{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, PLACE_IDS{12, 13, 14};

cv::Mat K() {return (cv::Mat_<double>(3, 3) << F, 0, 640, 0, F, 360, 0, 0, 1);}

// 한 변 size_m 인 태그를 화면 (cx, cy) 에 Z 거리로 (흰 여백 포함) 그린다
void draw_tag(cv::Mat & img, int id, double size_m, int cx, int cy)
{
  const int px = static_cast<int>(std::lround(F * size_m / Z));
  cv::Mat m;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, px, m, 1);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, px, m, 1);
#endif
  m.copyTo(img(cv::Rect(cx - px / 2, cy - px / 2, px, px)));
}

TagDetection det(int id, double area)
{
  TagDetection t;
  t.id = id;
  t.area = area;
  return t;
}
}  // namespace

TEST(ReadTag, LargestOfBothLists)
{
  const std::vector<TagDetection> pucks{det(7, 900), det(3, 2500)}, places{det(12, 1600)};
  ASSERT_NE(largest_tag(pucks, places), nullptr);
  EXPECT_EQ(largest_tag(pucks, places)->id, 3);
  EXPECT_EQ(largest_tag({det(7, 900)}, places)->id, 12);
  EXPECT_EQ(largest_tag({}, {}), nullptr);
}

TEST(ReadTag, KeepIdsEmptyKeepsAll)
{
  std::vector<TagDetection> v{det(7, 1), det(12, 1)};
  keep_tag_ids(v, {});
  EXPECT_EQ(v.size(), 2u);
  keep_tag_ids(v, PUCK_IDS);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].id, 7);
}

// 48 mm 장소 태그 12 와 16 mm 퍽 태그 7 을 같은 0.30 m 에 둔다:
// 퍽 검출기(16 mm)는 12 를 버리고, 장소 검출기(48 mm)는 12 를 0.30 m 로 낸다 → READ_TAG 결과는 12, 거리 0.30 m
TEST(ReadTag, PlaceTagReadWithItsOwnSize)
{
  cv::Mat img(720, 1280, CV_8UC1, cv::Scalar(255));
  draw_tag(img, 12, 0.048, 440, 360);
  draw_tag(img, 7, 0.016, 840, 360);
  cv::Mat bgr;
  cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);

  TagDetector puck_det("tag36h11", 0.016), place_det("tag36h11", 0.048);
  auto pucks = puck_det.detect(bgr, K(), cv::Mat());
  auto places = place_det.detect(bgr, K(), cv::Mat());
  // 거르기 전: 퍽 크기로 잰 태그 12 는 거리가 1/3 (0.10 m) 로 틀린다
  const auto wrong = std::find_if(pucks.begin(), pucks.end(), [](const TagDetection & t) {return t.id == 12;});
  ASSERT_NE(wrong, pucks.end());
  ASSERT_TRUE(wrong->pose_valid);
  EXPECT_NEAR(wrong->tvec[2], Z / 3, 0.01);
  keep_tag_ids(pucks, PUCK_IDS);
  keep_tag_ids(places, PLACE_IDS);
  ASSERT_EQ(pucks.size(), 1u);
  EXPECT_EQ(pucks[0].id, 7);
  ASSERT_EQ(places.size(), 1u);

  const TagDetection * best = largest_tag(pucks, places);
  ASSERT_NE(best, nullptr);
  EXPECT_EQ(best->id, 12);
  ASSERT_TRUE(best->pose_valid);
  EXPECT_NEAR(best->tvec[2], Z, 0.01);
  EXPECT_NEAR(pucks[0].tvec[2], Z, 0.01);   // 퍽 태그는 퍽 크기로 그대로
}
