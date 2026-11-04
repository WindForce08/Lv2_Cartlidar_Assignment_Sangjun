// 서 있는 직육면체 퍽 면 요 · 서 있는 윗면 높이 단서 시험 (face_yaw.hpp · puck_estimator.hpp)
#include <gtest/gtest.h>

#include <cmath>

#include "perception/face_yaw.hpp"
#include "perception/puck_estimator.hpp"

using namespace perception;

namespace
{
constexpr double kDeg = M_PI / 180;
// z 둘레 yaw, 그다음 x 둘레 roll (서 있는 퍽이 조금 기운 경우) 쿼터니언 (x, y, z, w)
void quat(double yaw, double roll, double q[4])
{
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  q[0] = cy * sr; q[1] = sy * sr; q[2] = sy * cr; q[3] = cy * cr;   // q = q_yaw · q_roll
}
}  // namespace

// 0° · 17° · 44° · 46° 돌린 퍽: 46° 는 같은 정사각 면이라 −44°
TEST(FaceYaw, FoldsToQuarterTurn)
{
  const double in[] = {0, 17, 44, 46, 90, 135, -17, -46, 180};
  const double want[] = {0, 17, 44, -44, 0, -45, -17, 44, 0};
  for (size_t i = 0; i < sizeof(in) / sizeof(in[0]); ++i) {
    double q[4];
    quat(in[i] * kDeg, 0, q);
    const auto y = face_yaw(q[0], q[1], q[2], q[3]);
    ASSERT_TRUE(y) << in[i];
    EXPECT_NEAR(*y, want[i] * kDeg, 1e-9) << in[i];
    EXPECT_GE(*y, -M_PI / 4);
    EXPECT_LT(*y, M_PI / 4);
  }
}

// 조금 기울어도(태그 자세 잡음 3°) 면 요는 거의 그대로, 누운 퍽(x 축이 수직)은 값 없음
TEST(FaceYaw, TiltAndLying)
{
  double q[4];
  quat(17 * kDeg, 3 * kDeg, q);
  EXPECT_NEAR(*face_yaw(q[0], q[1], q[2], q[3]), 17 * kDeg, 1e-9);   // x 둘레 roll 은 x 축을 바꾸지 않음
  // y 둘레 90°: x 축이 아래(−z) → 수평 성분 0
  const double s = std::sqrt(0.5);
  EXPECT_FALSE(face_yaw(0, s, 0, s));
}

// J1 → 퍽 방향과 비교: 17.06° 돈 퍽에 J1 방향 17.06° 면 0, 면이 2° 더 돌면 2°
TEST(FaceYaw, ErrorAgainstReference)
{
  EXPECT_NEAR(face_yaw_error(17.06 * kDeg, 17.06 * kDeg), 0, 1e-12);
  EXPECT_NEAR(face_yaw_error(19.06 * kDeg, 17.06 * kDeg), 2 * kDeg, 1e-12);
  EXPECT_NEAR(face_yaw_error(-44 * kDeg, 44 * kDeg), 2 * kDeg, 1e-12);   // 90° 주기
}

// 높이 단서 (puck_estimator.hpp 2-1): 받침 0 · 0.07, 서 있는 윗면 0.06 · 0.13, 누운 윗면 0.03 · 0.10
TEST(FaceHint, StandingTopByHeight)
{
  const std::vector<double> levels{0.0, 0.07};
  EXPECT_EQ(face_hint_from_height(0.130, 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kEnd);    // 상자 위 서 있음
  EXPECT_EQ(face_hint_from_height(0.122, 0.95, levels, 0.015, 0.03, 0.012), FaceHint::kEnd);   // 8 mm 낮게 잼
  EXPECT_EQ(face_hint_from_height(0.100, 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kNone);   // 상자 위 누움
  EXPECT_EQ(face_hint_from_height(0.060, 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kEnd);    // 바닥 서 있음
  EXPECT_EQ(face_hint_from_height(0.030, 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kNone);   // 바닥 누움
  EXPECT_EQ(face_hint_from_height(0.130, 0.5, levels, 0.015, 0.03, 0.012), FaceHint::kNone);   // 법선이 60° 기욺 (옆면 태그)
  EXPECT_EQ(face_hint_from_height(0.180, 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kNone);   // 목록에 없는 받침
  EXPECT_EQ(face_hint_from_height(std::nan(""), 1.0, levels, 0.015, 0.03, 0.012), FaceHint::kNone);
}
