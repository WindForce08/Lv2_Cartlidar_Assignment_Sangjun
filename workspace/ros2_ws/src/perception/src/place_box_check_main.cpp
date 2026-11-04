// place_box_check: 영상 한 장에서 장소 상자 윗면 찾기 (place_box.hpp) — 렌더 자료로 Python 시제품과 견주는 도구
//   place_box_check <영상> fx fy cx cy <T 16 개: 평면 기준 ← 카메라 광학, 행 우선> <사전 x> <사전 y> [사전 요]
//   출력 한 줄: x y yaw score (평면 기준, m · rad)
#include <cstdio>
#include <cstdlib>

#include <opencv2/imgcodecs.hpp>

#include "perception/place_box.hpp"

int main(int argc, char ** argv)
{
  if (argc < 24) {
    std::fprintf(stderr, "사용: place_box_check <영상> fx fy cx cy T00..T33 prior_x prior_y [prior_yaw]\n");
    return 2;
  }
  const cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
  if (img.empty()) {std::fprintf(stderr, "영상을 못 읽음: %s\n", argv[1]); return 1;}
  auto f = [&](int i) {return std::atof(argv[i]);};
  const cv::Matx33d K(f(2), 0, f(4), 0, f(3), f(5), 0, 0, 1);
  double T[16];
  for (int i = 0; i < 16; ++i) {T[i] = f(6 + i);}
  const cv::Matx33d R(T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]);
  const cv::Vec3d t(T[3], T[7], T[11]);
  const auto r = perception::find_place_box(img, K, R, t, cv::Point2d(f(22), f(23)), argc > 24 ? f(24) : 0.0);
  std::printf("%.6f %.6f %.6f %.3f %d\n", r.x, r.y, r.yaw, r.score, r.ok ? 1 : 0);
  return r.ok ? 0 : 1;
}
