// ScanToRangeNode (Gazebo 모의 전용)
//   Gazebo 는 TFmini 를 광선 센서로 그려 /tfmini/scan (LaserScan) 으로 낸다 (urdf_and_usd gz_bridge.yaml).
//   obstacle_stop · drive FINE 정렬은 실물 tfmini_node 와 같은 /tfmini/range (Range) 를 받으므로 이 노드가 바꿔 준다.
//   range = 유효한 광선 중 가장 가까운 값 (TFmini 빔 3.6° 안의 가장 가까운 면), 없으면 +inf. stamp = 스캔 stamp
//   실물에서는 띄우지 않는다 (tfmini_node 와 같은 토픽을 낸다)
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace perception
{

class ScanToRangeNode : public rclcpp::Node
{
public:
  explicit ScanToRangeNode(const rclcpp::NodeOptions & options)
  : Node("scan_to_range", options)
  {
    frame_id_ = declare_parameter("frame_id", std::string("tfmini_link"));
    min_range_ = declare_parameter("min_range", 0.1);
    max_range_ = declare_parameter("max_range", 12.0);
    fov_ = declare_parameter("field_of_view", 0.063);
    pub_ = create_publisher<sensor_msgs::msg::Range>(declare_parameter("output_topic", std::string("/tfmini/range")),
        rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter("input_topic", std::string("/tfmini/scan")),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr s) {on_scan(*s);});
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & s)
  {
    float best = std::numeric_limits<float>::infinity();
    for (float r : s.ranges) {
      if (std::isfinite(r) && r >= min_range_ && r <= max_range_ && r < best) {best = r;}
    }
    sensor_msgs::msg::Range m;
    m.header.stamp = s.header.stamp;
    m.header.frame_id = frame_id_;
    m.radiation_type = sensor_msgs::msg::Range::INFRARED;
    m.field_of_view = static_cast<float>(fov_);
    m.min_range = static_cast<float>(min_range_);
    m.max_range = static_cast<float>(max_range_);
    m.range = best;
    pub_->publish(m);
  }

  std::string frame_id_;
  double min_range_, max_range_, fov_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::ScanToRangeNode)
