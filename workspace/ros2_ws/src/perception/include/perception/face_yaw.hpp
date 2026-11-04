// 서 있는 직육면체 퍽의 면 요 (ROS 의존 없음 → 단위 시험). /perception/pucks 를 받는 쪽이 같은 식을 쓴다
//   정의: pose.orientation 의 x 축(끝면 태그 x = 면 모서리와 나란함, puck_estimator.hpp)을 그 frame 의 xy 평면에 투영한 각을
//         90° 주기로 접은 값 [−45°, 45°). /perception/pucks 는 base_link 라 base_link 기준 면 요다
//   대상: shape = SHAPE_CUBOID, standing, orientation_valid. x 축이 거의 수직(누운 퍽 등)이면 값이 없다
//   45° 집기: 손가락 닫는 축이 J1 둘레 접선이므로 면이 J1 → 퍽 방향과 맞아야 한다 → face_yaw_error(면 요, J1 → 퍽 방향각)
#pragma once

#include <cmath>
#include <optional>

namespace perception
{

// 각을 90° 주기로 접어 [−π/4, π/4)
inline double fold_quarter(double a)
{
  const double q = M_PI / 2;
  double r = std::fmod(a + M_PI / 4, q);
  if (r < 0) {r += q;}
  return r - M_PI / 4;
}

// 쿼터니언(x, y, z, w) → 면 요 [rad]. x 축의 수평 성분이 min_horizontal 보다 작으면 값 없음
inline std::optional<double> face_yaw(double qx, double qy, double qz, double qw, double min_horizontal = 0.5)
{
  const double ax = 1 - 2 * (qy * qy + qz * qz);
  const double ay = 2 * (qx * qy + qz * qw);
  if (std::hypot(ax, ay) < min_horizontal) {return std::nullopt;}
  return fold_quarter(std::atan2(ay, ax));
}

// 면 요와 기준 방향(예: J1 → 퍽 방향각)의 차이 [rad], 90° 주기로 접어 [−π/4, π/4)
inline double face_yaw_error(double face, double reference)
{
  return fold_quarter(face - reference);
}

}  // namespace perception
