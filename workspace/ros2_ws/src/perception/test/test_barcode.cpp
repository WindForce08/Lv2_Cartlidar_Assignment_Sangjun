// 바코드 찾기 · 읽기 시험 (numpy 재현 값 · 그린 평면 장면 · 라벨 그림 zbar). 렌더 장면에서 기준 Python 과 같은지는
// scripts/barcode_compare.py (R3 렌더 5 벌)
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/barcode_locator.hpp"
#include "perception/barcode_reader.hpp"
#include "perception/numpy_compat.hpp"

using namespace perception;

namespace
{
const std::string TEX = std::string(PERCEPTION_DATA_DIR) + "/barcode_P2-01.png";
const std::string TEX2 = std::string(PERCEPTION_TEST_DATA_DIR) + "/barcode_P2-02.png";
const BarcodeK K{606.0, 606.0, 320.0, 240.0};

// 카메라 광학 좌표 → base_footprint. 카메라 높이 0.30 m, 앞으로 보며 pitch 만큼 아래로 숙임
cv::Matx44d cam_pose(double pitch_deg)
{
  const double p = pitch_deg * M_PI / 180.0;
  const cv::Vec3d xc(0, -1, 0), zc(std::cos(p), 0, -std::sin(p)), yc = zc.cross(xc);
  return cv::Matx44d(xc[0], yc[0], zc[0], 0.0, xc[1], yc[1], zc[1], 0.0, xc[2], yc[2], zc[2], 0.30, 0, 0, 0, 1);
}

struct Scene
{
  cv::Mat bgr, depth;
};

// 화면 전체가 평면 하나(base_footprint 의 점 q · 법선 nb)인 장면. 라벨(35 × 20 mm)은 base 의 점 lc 에
// 가로축 lu 로 붙임(label = false 면 빈 면). 4 × 4 부분 화소 평균으로 그림, 깊이는 평면 그대로(잡음 없음)
Scene render_plane(const cv::Matx44d & T, const cv::Vec3d & q, const cv::Vec3d & nb, const cv::Vec3d & lc,
  const cv::Vec3d & lu, bool label)
{
  const cv::Mat tex = cv::imread(TEX, cv::IMREAD_GRAYSCALE);
  const cv::Matx33d R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  const cv::Vec3d t(T(0, 3), T(1, 3), T(2, 3));
  // 카메라 좌표로
  const cv::Vec3d qc = R.t() * (q - t), nc = R.t() * nb, lcc = R.t() * (lc - t), luc = R.t() * lu;
  const cv::Vec3d lvc = nc.cross(luc);   // 라벨 세로축 (글자 줄 쪽 방향은 상관없음)
  Scene s;
  s.bgr.create(480, 640, CV_8UC3);
  s.depth.create(480, 640, CV_32F);
  const int ss = 4;
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 640; ++x) {
      double acc = 0;
      for (int j = 0; j < ss; ++j) {
        for (int i = 0; i < ss; ++i) {
          const cv::Vec3d ray((x - 0.5 + (i + 0.5) / ss - K.cx) / K.fx, (y - 0.5 + (j + 0.5) / ss - K.cy) / K.fy, 1.0);
          const double lam = qc.dot(nc) / ray.dot(nc);
          const cv::Vec3d X = lam * ray - lcc;
          const double a = X.dot(luc) * 1000 + 17.5, b = X.dot(lvc) * 1000 + 10.0;   // 라벨 그림 mm
          double v = 150.0;
          if (label && a >= 0 && a < 35 && b >= 0 && b < 20) {
            v = tex.at<uchar>(std::min(479, static_cast<int>(b / 20 * 480)), std::min(839, static_cast<int>(a / 35 * 840)));
          }
          acc += v;
        }
      }
      const auto g = static_cast<uchar>(std::lround(acc / (ss * ss)));
      s.bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(g, g, g);
      const cv::Vec3d ray((x - K.cx) / K.fx, (y - K.cy) / K.fy, 1.0);
      s.depth.at<float>(y, x) = static_cast<float>(qc.dot(nc) / ray.dot(nc));
    }
  }
  cv::GaussianBlur(s.bgr, s.bgr, cv::Size(0, 0), 0.7);   // 카메라 흐림 대신
  return s;
}

cv::Vec3d to_base(const cv::Matx44d & T, const cv::Vec3d & c)
{
  return {T(0, 0) * c[0] + T(0, 1) * c[1] + T(0, 2) * c[2] + T(0, 3), T(1, 0) * c[0] + T(1, 1) * c[1] + T(1, 2) * c[2] + T(1, 3),
    T(2, 0) * c[0] + T(2, 1) * c[1] + T(2, 2) * c[2] + T(2, 3)};
}
}  // namespace

// numpy 1.26 에서 잰 값 (np.random.default_rng(seed).choice(n, 3, replace=False) 네 번)
TEST(NumpyCompat, RngMatchesNumpy)
{
  NumpyRng a(0);
  EXPECT_EQ(a.next_uint64(), 11749869230777074271ULL);
  NumpyRng b(0);
  const std::vector<std::vector<int64_t>> want = {{126, 102, 168}, {3, 14, 8}, {128, 100, 181}, {144, 108, 125}};
  for (const auto & w : want) {EXPECT_EQ(b.choice(200, 3), w);}
  NumpyRng c(123456);
  EXPECT_EQ(c.choice(43456, 3), (std::vector<int64_t>{35448, 4277, 27659}));
  NumpyRng d(5000000000ULL);   // 32 bit 넘는 씨앗
  EXPECT_EQ(d.choice(20000, 3), (std::vector<int64_t>{11441, 18783, 14758}));
}

TEST(NumpyCompat, Float32TrigAndReductions)
{
  // np.cos(np.radians(np.float32(67.5))) · np.sin(np.radians(np.float32(195))) — 표준 cosf 와 1 ulp 다른 값
  EXPECT_EQ(npc::np_cosf(npc::np_radiansf(67.5f)), 0x1.87de2cp-2f);
  EXPECT_EQ(npc::np_sinf(npc::np_radiansf(195.0f)), -0x1.0907dep-2f);
  std::vector<float> v(20001);
  for (size_t i = 0; i < v.size(); ++i) {v[i] = static_cast<float>(i) * 0.1f + 0.3f;}
  EXPECT_EQ(npc::np_sum_f32(v), 0x1.31485ap+24f);
  EXPECT_EQ(npc::np_mean_f32(v), 0x1.f4266ap+9f);
  EXPECT_EQ(npc::np_std_f32(v), 0x1.20b08ap+9f);
  const std::vector<float> w = {3, 1, 4, 1, 5, 9, 2, 6};
  EXPECT_DOUBLE_EQ(npc::np_percentile(w, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(npc::np_percentile(w, 90.0), 6.8999999999999995);
  EXPECT_FLOAT_EQ(npc::np_median(w), 3.5f);
  EXPECT_DOUBLE_EQ(npc::np_median(std::vector<double>{3, 1, 4}), 3.0);
}

TEST(BarcodeLocator, TemplatesBuilt)
{
  const BarcodeLocator loc(BarcodeLocatorConfig{}, TEX);
  ASSERT_TRUE(loc.ok());
  ASSERT_EQ(loc.templates().size(), 48u);   // 7.5° 마다 한 바퀴
  const auto & t = loc.templates()[0];
  EXPECT_EQ(t.tp.rows, 27);                 // 23 × 13 칸(1.5 mm) 라벨의 대각 27
  EXPECT_GT(cv::countNonZero(t.bright), 50);
  EXPECT_GT(cv::countNonZero(t.dark), 50);
  EXPECT_FALSE(BarcodeLocator(BarcodeLocatorConfig{}, std::string("/없는/그림.png")).ok());
}

TEST(BarcodeLocator, FindsLabelOnWall)
{
  const BarcodeLocator loc(BarcodeLocatorConfig{}, TEX);
  const cv::Matx44d T = cam_pose(0.0);
  const cv::Vec3d lc(0.60, -0.05, 0.28);   // 카메라 앞 0.6 m 벽, 높이 0.28 m
  const Scene s = render_plane(T, {0.60, 0, 0}, {-1, 0, 0}, lc, {0, -1, 0}, true);
  const auto dets = loc.locate(s.bgr, s.depth, K, T);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].surface, BarcodeDetection::WALL);
  EXPECT_LT(cv::norm(to_base(T, dets[0].center) - lc), 0.005);
  EXPECT_GE(dets[0].score, 0.65);
}

TEST(BarcodeLocator, FindsLabelOnTop)
{
  const BarcodeLocator loc(BarcodeLocatorConfig{}, TEX);
  const cv::Matx44d T = cam_pose(45.0);
  const cv::Vec3d lc(0.22, 0.02, 0.10);    // 높이 0.10 m 선반 윗면, 카메라에서 약 0.3 m
  const Scene s = render_plane(T, {0, 0, 0.10}, {0, 0, 1}, lc, {0.6, 0.8, 0}, true);
  const auto dets = loc.locate(s.bgr, s.depth, K, T);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].surface, BarcodeDetection::TOP);
  EXPECT_LT(cv::norm(to_base(T, dets[0].center) - lc), 0.005);
}

TEST(BarcodeLocator, NothingOnBlankPlanesOrLowWall)
{
  const BarcodeLocator loc(BarcodeLocatorConfig{}, TEX);
  const cv::Matx44d T = cam_pose(0.0);
  const Scene s = render_plane(T, {0.60, 0, 0}, {-1, 0, 0}, {0.60, -0.05, 0.28}, {0, -1, 0}, false);
  EXPECT_TRUE(loc.locate(s.bgr, s.depth, K, T).empty());
  // 수직면이 wall_min_top 보다 낮으면(상자 옆면) 벽으로 보지 않는다: 문턱을 면 높이보다 높게 하면 라벨이 있어도 없음
  BarcodeLocatorConfig c;
  c.wall_min_top = 1.0;
  const BarcodeLocator high(c, TEX);
  const Scene s2 = render_plane(T, {0.60, 0, 0}, {-1, 0, 0}, {0.60, -0.05, 0.28}, {0, -1, 0}, true);
  EXPECT_TRUE(high.locate(s2.bgr, s2.depth, K, T).empty());
  EXPECT_EQ(loc.locate(s2.bgr, s2.depth, K, T).size(), 1u);
  // 크기가 맞지 않는 입력은 빈 결과
  EXPECT_TRUE(loc.locate(s2.bgr, cv::Mat(10, 10, CV_32F, cv::Scalar(1.0f)), K, T).empty());
}

TEST(BarcodeLocator, ContrastLimit)
{
  // 진짜 라벨의 무늬 명암비(fit_a · 255 / 흰 테두리)는 1 근처: 기본 con_max 1.08 은 통과, 0.5 로 낮추면 버림
  const cv::Matx44d T = cam_pose(0.0);
  const Scene s = render_plane(T, {0.60, 0, 0}, {-1, 0, 0}, {0.60, -0.05, 0.28}, {0, -1, 0}, true);
  const auto dets = BarcodeLocator(BarcodeLocatorConfig{}, TEX).locate(s.bgr, s.depth, K, T);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_GT(dets[0].con, 0.5);
  EXPECT_LE(dets[0].con, 1.08);
  BarcodeLocatorConfig c;
  c.con_max = 0.5;
  EXPECT_TRUE(BarcodeLocator(c, TEX).locate(s.bgr, s.depth, K, T).empty());
}

TEST(BarcodeLocator, SurfacesSplitWallAndTop)
{
  const BarcodeLocator loc(BarcodeLocatorConfig{}, TEX);
  const cv::Matx44d T = cam_pose(0.0);
  const Scene s = render_plane(T, {0.60, 0, 0}, {-1, 0, 0}, {0.60, -0.05, 0.28}, {0, -1, 0}, false);
  const auto pls = loc.surfaces(s.depth, K, T);
  ASSERT_EQ(pls.size(), 1u);
  EXPECT_EQ(pls[0].kind, BarcodeDetection::WALL);
  EXPECT_NEAR(pls[0].n[2], -1.0f, 1e-4);   // 카메라 쪽을 봄
  // 평면 rms 는 깊이가 정확해도 0 이 아니다: 중심을 float32 로 차례로 더해(Py: Q.mean(0)) 화소 수십만 개면 중심이
  // 법선 방향으로 1 mm 안팎 어긋나고, 그 제곱이 가장 작은 고유값에 들어간다(기준 Python 과 같은 동작)
  EXPECT_LT(pls[0].rms, 0.002);
  EXPECT_GT(pls[0].ys.size(), 200000u);
}

TEST(BarcodeReader, ReadsLabelPicture)
{
  BarcodeReader r;
  const cv::Mat img = cv::imread(TEX, cv::IMREAD_COLOR);
  ASSERT_FALSE(img.empty());
  const auto res = r.read(img);
  EXPECT_TRUE(res.ok);
  EXPECT_EQ(res.text, "P2-01");
  cv::Mat g;
  cv::cvtColor(img, g, cv::COLOR_BGR2GRAY);
  EXPECT_EQ(r.read(g).text, "P2-01");   // 회색조도 같음
  BarcodeReader r2(2.0);                 // 확대 배율 (barcode.read_scale)
  EXPECT_EQ(r2.read(g).text, "P2-01");
}

TEST(BarcodeReader, TwoDifferentStringsIsUnread)
{
  BarcodeReader r;
  const cv::Mat a = cv::imread(TEX, cv::IMREAD_GRAYSCALE), b = cv::imread(TEX2, cv::IMREAD_GRAYSCALE);
  ASSERT_FALSE(b.empty());
  cv::Mat both;
  cv::hconcat(a, b, both);
  const auto res = r.read(both);
  EXPECT_FALSE(res.ok);
  EXPECT_TRUE(res.text.empty());
  EXPECT_GE(res.symbols.size(), 2u);
  // 같은 문자열 두 개는 읽음
  cv::Mat same;
  cv::hconcat(a, a, same);
  EXPECT_TRUE(r.read(same).ok);
  // 빈 영상 · 바코드 없는 영상
  EXPECT_FALSE(r.read(cv::Mat()).ok);
  const auto none = r.read(cv::Mat(480, 640, CV_8U, cv::Scalar(128)));
  EXPECT_FALSE(none.ok);
  EXPECT_TRUE(none.symbols.empty());
}

TEST(BarcodeReader, QualityFilterAndGamma)
{
  const cv::Mat a = cv::imread(TEX, cv::IMREAD_GRAYSCALE);
  BarcodeReader strict(1.0, 2.0, 1000000);   // quality 하한이 아주 크면 기호를 모두 뺌
  const auto r = strict.read(a);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(r.symbols.empty());
  EXPECT_GE(r.dropped, 1);
  BarcodeReader plain(1.0, 1.0, 1);           // 감마 · quality 거르기 없음 (44차 전 규칙)
  EXPECT_EQ(plain.read(a).text, "P2-01");
  EXPECT_EQ(plain.read(a).dropped, 0);
}

TEST(BarcodeReader, Decide)
{
  EXPECT_TRUE(decide_barcode({"P2-03", "P2-03"}).ok);
  EXPECT_EQ(decide_barcode({"P2-03", "P2-03"}).text, "P2-03");
  EXPECT_FALSE(decide_barcode({"PF-&3", "P2-03"}).ok);   // 판정 1차 틀린 문자열 사례
  EXPECT_FALSE(decide_barcode({}).ok);
}
