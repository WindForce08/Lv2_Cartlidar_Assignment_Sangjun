// camera_info 형식 검사 (perception_node, ROS 의존 없음 → 단위 시험)
//   K 가 3×3 pinhole 꼴인지(k[8] = 1, k[3] · k[6] · k[7] = 0, fx · fy > 0, 주점이 화면 안)와 영상 크기가 맞는지 본다.
//   근거: Isaac Sim 6.1 + Lyrical 기본 CameraInfoHelper 는 K/R/P 가 한 칸씩 밀려 나온 사례가 있다(waffle-lyrical-slam 검증 기록).
//   밀린 K 를 그대로 쓰면 퍽 위치 · 신호등 크기 · 바코드 위치가 소리 없이 틀리므로, 맞지 않으면 그 카메라 결과를 내지 않는다.
//   allow_scale: 팔 카메라처럼 camera_info 해상도와 받은 영상 해상도가 다르면 비율로 K 를 맞추는 경로(가로세로 비는 같아야 함)
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace perception
{

inline std::string check_camera_info(const std::array<double, 9> & k, uint32_t info_w, uint32_t info_h,
  uint32_t image_w, uint32_t image_h, bool allow_scale)
{
  for (double v : k) {
    if (!std::isfinite(v)) {return "K 에 유한하지 않은 값";}
  }
  if (std::abs(k[8] - 1.0) > 1e-6 || k[3] != 0.0 || k[6] != 0.0 || k[7] != 0.0) {
    return "K 가 3×3 pinhole 꼴이 아님(칸 밀림 의심)";
  }
  if (k[0] <= 0.0 || k[4] <= 0.0) {return "초점거리(fx · fy)가 0 이하";}
  if (info_w == 0 || info_h == 0) {return "camera_info 해상도 없음";}
  if (!(k[2] > 0.0 && k[2] < info_w && k[5] > 0.0 && k[5] < info_h)) {return "주점이 camera_info 화면 밖";}
  if (info_w != image_w || info_h != image_h) {
    if (!allow_scale) {return "camera_info 해상도가 영상과 다름";}
    if (std::abs(static_cast<double>(image_w) / info_w - static_cast<double>(image_h) / info_h) > 1e-3) {
      return "camera_info 와 영상의 가로세로 비가 다름";
    }
  }
  return "";
}

}  // namespace perception
