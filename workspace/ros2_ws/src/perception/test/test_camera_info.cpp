// camera_info 형식 검사 시험 (camera_info_check.hpp). D435 640 × 480, fx 606
#include <gtest/gtest.h>

#include "perception/camera_info_check.hpp"

using namespace perception;

namespace
{
const std::array<double, 9> K{606.0, 0.0, 320.0, 0.0, 606.0, 240.0, 0.0, 0.0, 1.0};
}  // namespace

TEST(CameraInfoCheck, ValidPinholeAccepted)
{
  EXPECT_EQ(check_camera_info(K, 640, 480, 640, 480, false), "");
}

// Isaac Sim 6.1 + Lyrical 에서 본 것처럼 한 칸 밀린 K 는 거부
TEST(CameraInfoCheck, ShiftedMatrixRejected)
{
  const std::array<double, 9> shifted{0.0, 606.0, 0.0, 320.0, 0.0, 606.0, 240.0, 0.0, 0.0};
  EXPECT_NE(check_camera_info(shifted, 640, 480, 640, 480, false), "");
  const std::array<double, 9> shifted_back{0.0, 320.0, 0.0, 606.0, 240.0, 0.0, 0.0, 1.0, 0.0};
  EXPECT_NE(check_camera_info(shifted_back, 640, 480, 640, 480, false), "");
}

TEST(CameraInfoCheck, BadValuesRejected)
{
  auto k = K;
  k[0] = 0.0;
  EXPECT_NE(check_camera_info(k, 640, 480, 640, 480, false), "");   // fx 0
  k = K;
  k[2] = 700.0;
  EXPECT_NE(check_camera_info(k, 640, 480, 640, 480, false), "");   // 주점 화면 밖
  k = K;
  k[4] = std::nan("");
  EXPECT_NE(check_camera_info(k, 640, 480, 640, 480, false), "");
  EXPECT_NE(check_camera_info(K, 0, 0, 640, 480, false), "");        // 해상도 없음
}

// D435 경로는 해상도가 영상과 같아야 함. 팔 경로는 같은 가로세로 비면 비율 보정 허용
TEST(CameraInfoCheck, ResolutionMismatch)
{
  EXPECT_NE(check_camera_info(K, 640, 480, 1280, 720, false), "");
  EXPECT_EQ(check_camera_info(K, 640, 480, 1280, 960, true), "");
  EXPECT_NE(check_camera_info(K, 640, 480, 1280, 720, true), "");    // 4:3 → 16:9
}
