// ObstacleStopNode (perception_container)
//   앞쪽 바닥 거리 센서(TFmini Plus)의 Range 를 받아 비상 정지 여부를 발행한다. 판정 규칙은 obstacle_stop.hpp
//   출력  stop_topic   std_msgs/Bool  true = 정지   (Reliable + TransientLocal, 바뀔 때 바로 + publish_rate 주기)
//         reason_topic std_msgs/String obstacle · too_close · stale · clear · no_return
//   이 노드는 "멈춰야 하는지"만 알린다. 실제로 바퀴를 세우는 것(예: twist_mux lock)은 제어 쪽에서 이 토픽을 받아 한다
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "perception/obstacle_stop.hpp"

namespace perception
{

class ObstacleStopNode : public rclcpp::Node
{
public:
  explicit ObstacleStopNode(const rclcpp::NodeOptions & options)
  : Node("obstacle_stop", options)
  {
    StopConfig c;
    c.stop_m = declare_parameter("stop_m", c.stop_m);
    c.release_m = declare_parameter("release_m", c.release_m);
    c.release_count = static_cast<int>(declare_parameter("release_count", c.release_count));
    c.near_m = declare_parameter("near_m", c.near_m);
    c.stale_s = declare_parameter("stale_s", c.stale_s);
    if (!(c.release_m > c.stop_m)) {throw std::runtime_error("release_m 은 stop_m 보다 커야 함");}
    logic_ = std::make_unique<ObstacleStop>(c);

    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(declare_parameter("stop_topic", std::string("/perception/obstacle_stop")), out_qos);
    reason_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter("reason_topic", std::string("/perception/obstacle_stop/reason")), out_qos);
    sub_ = create_subscription<sensor_msgs::msg::Range>(declare_parameter("range_topic", std::string("/tfmini/range")),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Range::ConstSharedPtr m) {on_range(*m);});
    const double rate = declare_parameter("publish_rate", 10.0);
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
        logic_->check_stale(now().seconds());
        publish(true);
      });
    RCLCPP_INFO(get_logger(), "정지 %.2f m 미만, 해제 %.2f m 이상 %d회, 입력 끊김 %.2f s", c.stop_m, c.release_m,
      c.release_count, c.stale_s);
  }

private:
  void on_range(const sensor_msgs::msg::Range & m)
  {
    std::optional<double> r;
    if (std::isfinite(m.range) && m.range >= m.min_range && m.range <= m.max_range) {r = m.range;}
    logic_->update(now().seconds(), r);
    publish(false);
  }

  // always=false 면 바뀌었을 때만
  void publish(bool always)
  {
    const bool s = logic_->stopped();
    const std::string & why = logic_->reason();
    if (!always && last_ && *last_ == s && last_reason_ == why) {return;}
    if (!last_ || *last_ != s) {
      RCLCPP_INFO(get_logger(), "비상 정지 %s (%s)", s ? "켬" : "끔", why.c_str());
    }
    last_ = s;
    last_reason_ = why;
    std_msgs::msg::Bool b;
    b.data = s;
    stop_pub_->publish(b);
    std_msgs::msg::String w;
    w.data = why;
    reason_pub_->publish(w);
  }

  std::unique_ptr<ObstacleStop> logic_;
  std::optional<bool> last_;
  std::string last_reason_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::ObstacleStopNode)
