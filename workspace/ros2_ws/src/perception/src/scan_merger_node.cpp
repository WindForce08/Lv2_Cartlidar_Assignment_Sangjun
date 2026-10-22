// ScanMergerNode (perception_container)
//   inputs(라이다 토픽 목록)의 LaserScan 을 output_frame(base_link) 한 평면 스캔으로 합쳐 /scan 으로 발행한다.
//   라이다마다 따로 나오는 토픽(/scan_front · /scan_rear)은 그대로 두므로 둘 다 쓸 수 있다.
//   - 각 점을 TF 로 output_frame 에 옮긴 뒤 각도 칸에 넣는다. 라이다는 3층 판 아래에 거꾸로(roll π) 달려 있고
//     URDF scan 프레임에 그 방향이 들어 있어서, 뒤집힌 각도 순서가 TF 로 저절로 바로잡힌다
//   - 라이다는 차체에 고정이라 TF 는 처음 한 번 읽어 둔다(정적)
//   - 발행 시점: 목록의 첫 라이다(trigger)가 들어올 때. 나머지는 trigger 와 stamp 차이가 max_dt_s 이내인 최신 스캔만 합친다.
//     trigger 가 stale_s 동안 안 오면 들어오는 다른 라이다로 대신 발행한다(한 대가 죽어도 /scan 이 끊기지 않게)
//   - 두 스캔의 측정 시각 차이(최대 max_dt_s) 동안의 차체 이동은 보정하지 않는다
//   - stamp = trigger 스캔 stamp
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "perception/scan_merge.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception
{
using LaserScan = sensor_msgs::msg::LaserScan;

class ScanMergerNode : public rclcpp::Node
{
public:
  explicit ScanMergerNode(const rclcpp::NodeOptions & options)
  : Node("scan_merger", options)
  {
    inputs_ = declare_parameter("inputs", std::vector<std::string>{"/scan_front", "/scan_rear"});
    output_frame_ = declare_parameter("output_frame", std::string("base_link"));
    cfg_.bins = static_cast<int>(declare_parameter("bins", 360));
    cfg_.angle_min = declare_parameter("angle_min", -M_PI);
    cfg_.range_min = declare_parameter("range_min", 0.0);
    cfg_.range_max = declare_parameter("range_max", 12.0);
    const auto box = declare_parameter("self_box", std::vector<double>{});   // [x_min, x_max, y_min, y_max] (output_frame)
    if (box.size() == 4) {cfg_.self_box = {box[0], box[1], box[2], box[3]};}
    max_dt_s_ = declare_parameter("max_dt_s", 0.15);
    stale_s_ = declare_parameter("stale_s", 0.5);
    if (inputs_.empty() || cfg_.bins < 1) {throw std::runtime_error("inputs 가 비었거나 bins < 1");}

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<LaserScan>(declare_parameter("output_topic", std::string("/scan")), rclcpp::SensorDataQoS());
    latest_.resize(inputs_.size());
    last_rx_.resize(inputs_.size());
    for (size_t i = 0; i < inputs_.size(); ++i) {
      subs_.push_back(create_subscription<LaserScan>(inputs_[i], rclcpp::SensorDataQoS(),
          [this, i](LaserScan::ConstSharedPtr m) {on_scan(i, m);}));
    }
    RCLCPP_INFO(get_logger(), "라이다 %zu대 → %s (%s, %d칸)", inputs_.size(), pub_->get_topic_name(),
      output_frame_.c_str(), cfg_.bins);
  }

private:
  static double sec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

  void on_scan(size_t i, const LaserScan::ConstSharedPtr & m)
  {
    latest_[i] = m;
    const double now_s = now().seconds();
    last_rx_[i] = now_s;
    // trigger: 첫 라이다. 첫 라이다가 stale_s 동안 없으면, 살아 있는 라이다 중 번호가 가장 작은 것
    size_t trigger = 0;
    while (trigger < inputs_.size() && (!last_rx_[trigger] || now_s - *last_rx_[trigger] > stale_s_)) {++trigger;}
    if (trigger != i) {return;}
    if (trigger != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 가 %.1f s 넘게 없음: %s 기준으로 발행",
        inputs_[0].c_str(), stale_s_, inputs_[i].c_str());
    }
    publish(m);
  }

  std::optional<tf2::Transform> transform_of(const std::string & frame)
  {
    if (auto it = tf_cache_.find(frame); it != tf_cache_.end()) {return it->second;}
    try {
      tf2::Transform t;
      tf2::fromMsg(tf_buffer_->lookupTransform(output_frame_, frame, tf2::TimePointZero).transform, t);
      tf_cache_[frame] = t;
      return t;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s→%s 없음: %s", frame.c_str(), output_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  void add_scan(ScanBinner & b, const LaserScan & s)
  {
    const auto t = transform_of(s.header.frame_id);
    if (!t) {return;}
    for (size_t k = 0; k < s.ranges.size(); ++k) {
      const float r = s.ranges[k];
      if (!std::isfinite(r) || r < s.range_min || r > s.range_max) {continue;}
      const double a = s.angle_min + static_cast<double>(k) * s.angle_increment;
      const tf2::Vector3 p = *t * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
      b.add(p.x(), p.y());   // 평면에 투영 (높이 차이는 버린다)
    }
  }

  void publish(const LaserScan::ConstSharedPtr & trigger)
  {
    ScanBinner b(cfg_);
    add_scan(b, *trigger);
    int used = 1;
    for (const auto & s : latest_) {
      if (!s || s == trigger) {continue;}
      if (std::abs(sec(s->header.stamp) - sec(trigger->header.stamp)) > max_dt_s_) {continue;}
      add_scan(b, *s);
      ++used;
    }
    if (used < static_cast<int>(inputs_.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "라이다 %d/%zu대만 합침 (나머지는 없거나 %.2f s 넘게 차이)",
        used, inputs_.size(), max_dt_s_);
    }
    LaserScan out;
    out.header.stamp = trigger->header.stamp;
    out.header.frame_id = output_frame_;
    out.angle_increment = static_cast<float>(b.increment());
    out.angle_min = static_cast<float>(cfg_.angle_min);
    out.angle_max = static_cast<float>(cfg_.angle_min + (cfg_.bins - 1) * b.increment());
    out.time_increment = 0.0f;
    out.scan_time = trigger->scan_time;
    out.range_min = static_cast<float>(cfg_.range_min);
    out.range_max = static_cast<float>(cfg_.range_max);
    out.ranges = b.ranges();
    pub_->publish(out);
  }

  std::vector<std::string> inputs_;
  std::string output_frame_;
  MergeConfig cfg_;
  double max_dt_s_, stale_s_;
  std::vector<LaserScan::ConstSharedPtr> latest_;
  std::vector<std::optional<double>> last_rx_;
  std::map<std::string, tf2::Transform> tf_cache_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<LaserScan>::SharedPtr> subs_;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::ScanMergerNode)
