// TF 조회 도우미: 고정 변환 기억 + 시계 되감기 대응
//   tf2_ros::Buffer 는 시계가 뒤로 가면(시뮬레이션 리셋) "Clearing TF buffer" 하면서 고정(static) 변환까지 지운다.
//   /tf_static 은 처음 연결될 때 한 번만 오므로 그대로 두면 리셋 뒤 카메라·라이다 장착 변환을 다시 찾지 못한다
//   (2026-10-22 모의 시험 M3·M4 에서 "Static cache is empty" 로 확인).
//   그래서 (1) 한 번 찾은 고정 변환은 따로 기억하고 (2) 시계가 뒤로 가면 버퍼와 리스너를 새로 만들어
//   /tf_static 을 다시 받는다(TransientLocal 이라 새 구독자에게 다시 온다).
//   사용 스레드: fixed() · buffer() 는 한 콜백 그룹(한 스레드씩)에서만 부른다. 시계 콜백은 표시만 한다.
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception
{

class TfHelper
{
public:
  explicit TfHelper(rclcpp::Node & node)
  : node_(node)
  {
    make();
    rcl_jump_threshold_t th{};
    th.on_clock_change = true;
    th.min_forward.nanoseconds = 0;     // 앞으로 뛰는 것은 보지 않음
    th.min_backward.nanoseconds = -1;   // 조금이라도 뒤로 가면
    jump_ = node_.get_clock()->create_jump_callback(
      []() {}, [this](const rcl_time_jump_t &) {rebuild_ = true;}, th);
  }

  // 장착처럼 시간에 따라 바뀌지 않는 변환 target ← source. 한 번 찾으면 기억한다
  std::optional<tf2::Transform> fixed(const std::string & target, const std::string & source)
  {
    const auto key = std::make_pair(target, source);
    if (auto it = cache_.find(key); it != cache_.end()) {return it->second;}
    try {
      tf2::Transform t;
      tf2::fromMsg(buffer().lookupTransform(target, source, tf2::TimePointZero).transform, t);
      cache_[key] = t;
      return t;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "TF %s→%s 없음: %s",
        source.c_str(), target.c_str(), e.what());
      return std::nullopt;
    }
  }

  // 움직이는 변환(odom 등) 조회용 버퍼. 시계가 뒤로 갔으면 먼저 새로 만든다
  tf2_ros::Buffer & buffer()
  {
    if (rebuild_.exchange(false)) {
      listener_.reset();
      make();
      RCLCPP_WARN(node_.get_logger(), "시계가 뒤로 감: TF 리스너를 새로 만들어 /tf_static 을 다시 받음 (기억한 고정 변환 %zu개 유지)",
        cache_.size());
    }
    return *buffer_;
  }

private:
  void make()
  {
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

  rclcpp::Node & node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::JumpHandler::SharedPtr jump_;
  std::atomic<bool> rebuild_{false};
  std::map<std::pair<std::string, std::string>, tf2::Transform> cache_;
};

}  // namespace perception
