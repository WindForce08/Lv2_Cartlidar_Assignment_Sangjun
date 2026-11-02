// 후방 장애물 거리 시험: 뒤 D435 자세(URDF 261027_1711)에서 바닥 · 벽 · 상자를 광선으로 그린 합성 깊이로 본다
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>

#include "perception/obstacle_stop.hpp"
#include "perception/rear_obstacle.hpp"

using namespace perception;

namespace
{

// base_footprint ← 뒤 컬러 광학 프레임. 링크 (−0.2204, 0, 0.4836 + 0.00675), 아래로 0.6144 rad, 뒤를 봄(yaw π)
struct RearCam
{
  double px = -0.22041, py = 0.0, pz = 0.48362 + 0.00675, pitch = 0.61441;
  PinholeK K{605.85, 605.85, 320.0, 240.0};
  int w = 640, h = 480;

  cv::Matx34d T() const
  {
    const double c = std::cos(pitch), s = std::sin(pitch);
    // 열 = 광학 축(x 오른쪽 · y 아래 · z 앞)을 base 로 나타낸 것
    return {0.0, s, -c, px,
            1.0, 0.0, 0.0, py,
            0.0, -c, -s, pz};
  }
};

// 축에 나란한 상자 [x0,x1]×[y0,y1]×[z0,z1] 와 광선의 첫 교점 거리 (없으면 inf)
double hit_box(const cv::Vec3d & o, const cv::Vec3d & d, const cv::Vec3d & lo, const cv::Vec3d & hi)
{
  double t0 = 0.0, t1 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-12) {
      if (o[i] < lo[i] || o[i] > hi[i]) {return std::numeric_limits<double>::infinity();}
      continue;
    }
    double a = (lo[i] - o[i]) / d[i], b = (hi[i] - o[i]) / d[i];
    if (a > b) {std::swap(a, b);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
  }
  return t0 <= t1 ? t0 : std::numeric_limits<double>::infinity();
}

// 바닥(z = 0) + 상자들을 그린 깊이(광학 z, m). boxes 는 {lo, hi} 쌍
cv::Mat render(const RearCam & cam, const std::vector<std::pair<cv::Vec3d, cv::Vec3d>> & boxes)
{
  const cv::Matx34d T = cam.T();
  const cv::Vec3d o(T(0, 3), T(1, 3), T(2, 3));
  cv::Mat depth(cam.h, cam.w, CV_32F, cv::Scalar(0.0f));
  for (int v = 0; v < cam.h; ++v) {
    for (int u = 0; u < cam.w; ++u) {
      const cv::Vec3d r((u - cam.K.ppx) / cam.K.fx, (v - cam.K.ppy) / cam.K.fy, 1.0);   // 광학 z 성분 = 1 → t = 깊이
      const cv::Vec3d d(T(0, 0) * r[0] + T(0, 1) * r[1] + T(0, 2) * r[2],
        T(1, 0) * r[0] + T(1, 1) * r[1] + T(1, 2) * r[2],
        T(2, 0) * r[0] + T(2, 1) * r[1] + T(2, 2) * r[2]);
      double t = d[2] < 0 ? -o[2] / d[2] : std::numeric_limits<double>::infinity();
      for (const auto & b : boxes) {t = std::min(t, hit_box(o, d, b.first, b.second));}
      if (std::isfinite(t) && t < 10.0) {depth.at<float>(v, u) = static_cast<float>(t);}
    }
  }
  return depth;
}

RearConfig cfg() {return RearConfig{};}

// 뒤 끝(x_edge)에서 dist 떨어진 앞면을 가진 상자 (x 방향 두께 thick, 가운데 y, 크기 w × h)
std::pair<cv::Vec3d, cv::Vec3d> box_behind(double dist, double y, double w, double h, double thick = 0.03)
{
  const double xf = RearConfig{}.x_edge - dist;
  return {cv::Vec3d(xf - thick, y - w / 2, 0.0), cv::Vec3d(xf, y + w / 2, h)};
}

}  // namespace

TEST(RearObstacle, FloorOnlyIsEmpty)
{
  RearCam cam;
  const auto r = nearest_rear(render(cam, {}), 0.001, cam.K, cam.T(), cfg());
  EXPECT_FALSE(r.distance.has_value());
  EXPECT_LT(r.points, cfg().min_points);
}

TEST(RearObstacle, WallDistance)
{
  RearCam cam;
  for (double dist : {0.30, 0.40, 0.60, 0.90}) {
    const auto r = nearest_rear(render(cam, {box_behind(dist, 0.0, 1.0, 0.38)}), 0.001, cam.K, cam.T(), cfg());
    ASSERT_TRUE(r.distance.has_value()) << dist;
    EXPECT_NEAR(*r.distance, dist, 0.01) << dist;
  }
}

TEST(RearObstacle, LowPuckSeenOnlyOutsideBlindZone)
{
  RearCam cam;   // 누운 퍽 크기 0.06 × 0.03 × 0.03 m: 바닥 높이 사각(뒤 끝에서 약 0.3 m) 밖에서만 보인다
  const auto far = nearest_rear(render(cam, {box_behind(0.38, 0.0, 0.06, 0.03)}), 0.001, cam.K, cam.T(), cfg());
  ASSERT_TRUE(far.distance.has_value());
  EXPECT_NEAR(*far.distance, 0.38, 0.03);
  const auto near = nearest_rear(render(cam, {box_behind(0.20, 0.0, 0.06, 0.03)}), 0.001, cam.K, cam.T(), cfg());
  EXPECT_FALSE(near.distance.has_value());
}

TEST(RearObstacle, OutsideCorridorIgnored)
{
  RearCam cam;
  const auto r = nearest_rear(render(cam, {box_behind(0.35, 0.40, 0.10, 0.30)}), 0.001, cam.K, cam.T(), cfg());
  EXPECT_FALSE(r.distance.has_value());   // |y| 0.35~0.45 > 반폭 0.2176
}

TEST(RearObstacle, U16MatchesF32AndSkipsInvalid)
{
  RearCam cam;
  cv::Mat f = render(cam, {box_behind(0.45, 0.0, 0.20, 0.20)});
  cv::Mat u;
  f.convertTo(u, CV_16U, 1000.0);
  f.at<float>(10, 10) = std::numeric_limits<float>::quiet_NaN();
  const auto a = nearest_rear(f, 0.001, cam.K, cam.T(), cfg());
  const auto b = nearest_rear(u, 0.001, cam.K, cam.T(), cfg());
  ASSERT_TRUE(a.distance && b.distance);
  EXPECT_NEAR(*a.distance, *b.distance, 0.002);
  EXPECT_FALSE(nearest_rear(cv::Mat(), 0.001, cam.K, cam.T(), cfg()).distance.has_value());
}

TEST(RearObstacle, FewNoisePixelsIgnored)
{
  RearCam cam;
  cv::Mat f = render(cam, {});
  // 바닥 대신 통로 안 높이(카메라 아래 0.3 m 쯤)로 보이는 잡음 화소 5개
  for (int i = 0; i < 5; ++i) {f.at<float>(400, 300 + 2 * i) = 0.35f;}
  EXPECT_FALSE(nearest_rear(f, 0.001, cam.K, cam.T(), cfg()).distance.has_value());
}

TEST(RearObstacle, StopLogicWithCameraSettings)
{
  StopConfig c;
  c.stop_m = 0.40;
  c.release_m = 0.50;
  c.release_count = 3;
  c.near_m = 0.0;   // 점 없음 = 비었음
  c.stale_s = 0.5;
  ObstacleStop s(c);
  EXPECT_TRUE(s.update(0.0, std::nullopt));         // 켤 때는 정지 상태에서 시작 → 비었다는 측정 3번에 풂
  EXPECT_TRUE(s.update(0.03, std::nullopt));
  EXPECT_FALSE(s.update(0.06, std::nullopt));
  EXPECT_FALSE(s.update(0.09, 0.45));               // 정지 ~ 해제 사이: 서 있지 않으면 그대로
  EXPECT_TRUE(s.update(0.1, 0.35));                 // 정지
  EXPECT_TRUE(s.update(0.2, std::nullopt));         // 사각으로 사라짐 → 비었다는 측정 1
  EXPECT_TRUE(s.update(0.3, std::nullopt));         // 2
  EXPECT_FALSE(s.update(0.4, std::nullopt));        // 3 → 풂
  EXPECT_TRUE(s.check_stale(1.0));                  // 깊이 끊김 → 정지
}
