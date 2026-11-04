// 장소 상자 윗면 찾기 시험 (place_box.hpp): 합성 영상 (바닥 · 윗면만, 바늘구멍 카메라 위에서 내려다봄)
#include <gtest/gtest.h>

#include <cmath>

#include "perception/place_box.hpp"

using namespace perception;

namespace
{
constexpr double kTop = 0.07;
// 카메라: 평면 기준 (0.255, 0, 0.28) 에서 아래를 봄. 광학 x = −Y (오른쪽), y = −X (아래), z = −Z (앞)
const cv::Matx33d kR(0, -1, 0, -1, 0, 0, 0, 0, -1);
const cv::Vec3d kT(0.255, 0.0, 0.28);
const cv::Matx33d kK(420, 0, 640, 0, 420, 360, 0, 0, 1);

// 화소마다 시선을 윗면 높이 → 상자 안이면 상자 색, 아니면 바닥 높이 → 바닥 색. pucks: 윗면 위 빨간 원 (반지름 15 mm, 위에서 본 모양만)
cv::Mat render(double cx, double cy, double yaw, const std::vector<cv::Point2d> & pucks)
{
  cv::Mat img(720, 1280, CV_8UC3);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int v = 0; v < img.rows; ++v) {
    for (int u = 0; u < img.cols; ++u) {
      const cv::Vec3d d = kR * cv::Vec3d((u + 0.5 - kK(0, 2)) / kK(0, 0), (v + 0.5 - kK(1, 2)) / kK(1, 1), 1.0);
      const double st = (kTop - kT[2]) / d[2];
      const double X = kT[0] + st * d[0] - cx, Y = kT[1] + st * d[1] - cy;
      const double lx = c * X + s * Y, ly = -s * X + c * Y;
      cv::Vec3b col(115, 115, 115);   // 바닥 (BGR)
      if (std::abs(lx) <= 0.1025 && std::abs(ly) <= 0.1025) {
        col = cv::Vec3b(122, 173, 204);   // 상자 (0.8 · 0.68 · 0.48 RGB)
        for (const auto & p : pucks) {
          if (std::hypot(X + cx - p.x, Y + cy - p.y) <= 0.015) {col = cv::Vec3b(26, 26, 217);}
        }
      }
      img.at<cv::Vec3b>(v, u) = col;
    }
  }
  return img;
}
}  // namespace

TEST(PlaceBox, FindsRotatedShiftedBox)
{
  for (const auto & [cx, cy, yaw] : {std::tuple{0.26, 0.0, 0.0}, std::tuple{0.28, 0.04, 0.45}, std::tuple{0.24, -0.05, -0.5}}) {
    const auto r = find_place_box(render(cx, cy, yaw, {}), kK, kR, kT, cv::Point2d(0.2553, 0.0), 0.0);
    ASSERT_TRUE(r.ok);
    EXPECT_NEAR(r.x, cx, 0.0015);
    EXPECT_NEAR(r.y, cy, 0.0015);
    EXPECT_NEAR(r.yaw, yaw, 0.2 * CV_PI / 180);
  }
}

// 칸 위 빨간 퍽 (테두리 가까이 포함) 이 있어도 같은 자리. 퍽 색 화소는 표본에서 빠진다
TEST(PlaceBox, PuckColoredPixelsIgnored)
{
  const double cx = 0.27, cy = 0.02, yaw = 0.3;
  PlaceBoxResult box{true, cx, cy, yaw, 0};
  std::vector<cv::Point2d> pucks;
  for (const auto & p : place_cells(box)) {pucks.push_back(p);}
  const auto r = find_place_box(render(cx, cy, yaw, pucks), kK, kR, kT, cv::Point2d(0.2553, 0.0), 0.0);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.x, cx, 0.0015);
  EXPECT_NEAR(r.y, cy, 0.0015);
  EXPECT_NEAR(r.yaw, yaw, 0.2 * CV_PI / 180);
}

// 칸 순서 = 행 우선 (dx 바깥, dy 안쪽), 간격 0.06, 요 0 이면 축 그대로
TEST(PlaceBox, CellsRowMajor)
{
  const auto c = place_cells(PlaceBoxResult{true, 1.0, 2.0, 0.0, 0});
  EXPECT_NEAR(c[0].x, 0.94, 1e-12);
  EXPECT_NEAR(c[0].y, 1.94, 1e-12);
  EXPECT_NEAR(c[4].x, 1.0, 1e-12);
  EXPECT_NEAR(c[8].y, 2.06, 1e-12);
  const auto r = place_cells(PlaceBoxResult{true, 0.0, 0.0, CV_PI / 2, 0});
  EXPECT_NEAR(r[0].x, 0.06, 1e-12);   // dx −1 · dy −1 → 90° 돌리면 (+0.06, −0.06)
  EXPECT_NEAR(r[0].y, -0.06, 1e-12);
}
