// 촬영 시각 TF 조회 (ROS 노드 의존 없음 → tf2::BufferCore 로 단위 시험)
#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core.hpp"

namespace perception
{

// 움직이는 변환 target ← source 를 촬영 시각 tp 로 찾는다. 그 시각 값이 없으면(아직 안 옴 · 버퍼 밖) 가장 최근 값을 쓰되,
// 그 값의 시각이 tp 와 max_dt_s 넘게 다르면 쓰지 않는다(nullopt). max_dt_s < 0 이면 촬영 시각 값만.
// 예전에는 시각 차를 보지 않고 최근 값을 썼다 → 팔 · 차체가 움직이는 중 TF 가 늦거나 멈추면 다른 자세로 위치를 냄.
// 변환이 아예 없으면 tf2::TransformException 을 그대로 던진다
inline std::optional<geometry_msgs::msg::TransformStamped> lookup_near(
  const tf2::BufferCore & buf, const std::string & target, const std::string & source, const tf2::TimePoint & tp,
  double max_dt_s)
{
  if (buf.canTransform(target, source, tp)) {return buf.lookupTransform(target, source, tp);}
  if (max_dt_s < 0) {return std::nullopt;}
  auto m = buf.lookupTransform(target, source, tf2::TimePointZero);
  const auto got = tf2::TimePoint(std::chrono::seconds(m.header.stamp.sec) + std::chrono::nanoseconds(m.header.stamp.nanosec));
  const double dt = std::abs(std::chrono::duration<double>(got - tp).count());
  if (dt > max_dt_s) {return std::nullopt;}
  return m;
}

}  // namespace perception
