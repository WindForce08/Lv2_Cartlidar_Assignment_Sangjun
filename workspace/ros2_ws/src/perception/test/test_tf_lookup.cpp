// 촬영 시각 TF 조회 시험 (tf_lookup.hpp). odom ← base_link 를 0.00 s · 0.05 s 에 넣고 찾는다
#include <gtest/gtest.h>

#include "perception/tf_lookup.hpp"

using namespace perception;

namespace
{
tf2::TimePoint at(double s) {return tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(s * 1e9)));}

geometry_msgs::msg::TransformStamped tf_at(double s, double x)
{
  geometry_msgs::msg::TransformStamped m;
  m.header.frame_id = "odom";
  m.child_frame_id = "base_link";
  m.header.stamp.sec = static_cast<int32_t>(s);
  m.header.stamp.nanosec = static_cast<uint32_t>((s - static_cast<int32_t>(s)) * 1e9);
  m.transform.translation.x = x;
  m.transform.rotation.w = 1.0;
  return m;
}

void fill(tf2::BufferCore & b)
{
  b.setTransform(tf_at(10.00, 0.0), "test");
  b.setTransform(tf_at(10.05, 0.1), "test");
}
}  // namespace

// 촬영 시각 값이 있으면 그 값(보간)
TEST(TfLookup, ExactTimeInterpolates)
{
  tf2::BufferCore b;
  fill(b);
  const auto m = lookup_near(b, "odom", "base_link", at(10.025), 0.1);
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->transform.translation.x, 0.05, 1e-9);
}

// 최근 값보다 늦은 영상: 차가 허용 안이면 최근 값, 넘으면 없음 (예전에는 차와 상관없이 최근 값)
TEST(TfLookup, LatestOnlyWithinLimit)
{
  tf2::BufferCore b;
  fill(b);
  const auto near = lookup_near(b, "odom", "base_link", at(10.10), 0.1);
  ASSERT_TRUE(near);
  EXPECT_NEAR(near->transform.translation.x, 0.1, 1e-9);
  EXPECT_FALSE(lookup_near(b, "odom", "base_link", at(10.20), 0.1));
  EXPECT_FALSE(lookup_near(b, "odom", "base_link", at(10.10), -1.0));   // 촬영 시각 값만
}

// 변환이 아예 없으면 예외 (부르는 쪽이 '없음'으로 처리)
TEST(TfLookup, MissingFrameThrows)
{
  tf2::BufferCore b;
  fill(b);
  EXPECT_THROW(lookup_near(b, "odom", "arm_camera_optical_frame", at(10.0), 0.1), tf2::TransformException);
}
