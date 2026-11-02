// ObstacleStopNode (bringup control_container — tfmini_node 와 같은 프로세스. perception.launch 단독 실행 때는 perception_container)
//   앞쪽 바닥 거리 센서(TFmini Plus, 박스 앞면 거리)의 Range 를 받아 비상 정지 여부를 발행한다. 판정 규칙은 obstacle_stop.hpp
//   출력  stop_topic   std_msgs/Bool  true = 정지   (Reliable + TransientLocal, 바뀔 때 바로 + publish_rate 주기)
//         reason_topic std_msgs/String obstacle · too_close · stale · clear · no_return
//   이 노드는 "멈춰야 하는지"만 알린다. 실제로 바퀴를 세우는 것은 twist_mux lock (priority 60: drive · nav 를 막고 조이스틱은 통과)
//   박스 접근: 파라미터 approach_stop_m (0 = 끔) 을 drive_node 가 FINE 정렬 시작 때 "목표 − 여유"로 넣고 끝날 때 0 으로 되돌린다.
//     drive 가 되돌리지 못하고 죽어도 approach_max_s 뒤에 스스로 평소 정지 거리로 돌아간다
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
    c.approach_hysteresis = declare_parameter("approach_hysteresis", c.approach_hysteresis);
    approach_max_s_ = declare_parameter("approach_max_s", 60.0);
    c.stale_s = declare_parameter("stale_s", c.stale_s);
    if (!(c.release_m > c.stop_m)) {throw std::runtime_error("release_m 은 stop_m 보다 커야 함");}
    logic_ = std::make_unique<ObstacleStop>(c);
    stop_m_ = c.stop_m;
    min_approach_m_ = declare_parameter("approach_min_m", 0.10);   // TFmini 최소 측정 거리 — 이보다 짧은 정지 거리는 잴 수 없음
    // 박스 접근 정지 거리 (0 = 끔). 같은 콜백 그룹(기본, 상호 배타)이라 측정 · 타이머 콜백과 겹치지 않는다
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto & p : ps) {
          if (p.get_name() != "approach_stop_m") {continue;}
          const double v = p.as_double();
          if (v != 0.0 && !(v >= min_approach_m_ && v < stop_m_)) {
            res.successful = false;
            res.reason = "approach_stop_m 은 0(끔) 또는 approach_min_m 이상 · stop_m 미만";
            return res;
          }
          apply_approach(v);
        }
        return res;
      });
    declare_parameter("approach_stop_m", 0.0);

    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(declare_parameter("stop_topic", std::string("/perception/obstacle_stop")), out_qos);
    reason_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter("reason_topic", std::string("/perception/obstacle_stop/reason")), out_qos);
    sub_ = create_subscription<sensor_msgs::msg::Range>(declare_parameter("range_topic", std::string("/tfmini/range")),
        rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Range::ConstSharedPtr m) {on_range(*m);});
    const double rate = declare_parameter("publish_rate", 10.0);
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
        if (logic_->approach() && (now() - approach_since_).seconds() > approach_max_s_) {
          RCLCPP_WARN(get_logger(), "박스 접근 정지 거리 %.0f s 지남 — 평소 정지 거리로 되돌림", approach_max_s_);
          set_parameter(rclcpp::Parameter("approach_stop_m", 0.0));
        }
        logic_->check_stale(now().seconds());
        publish(true);
      });
    RCLCPP_INFO(get_logger(), "정지 %.2f m 미만, 해제 %.2f m 이상 %d회, 입력 끊김 %.2f s", c.stop_m, c.release_m,
      c.release_count, c.stale_s);
  }

private:
  void apply_approach(double v)
  {
    if (v == 0.0) {
      if (logic_->approach()) {RCLCPP_INFO(get_logger(), "박스 접근 끝: 정지 거리 %.3f m 로 되돌림", stop_m_);}
      logic_->set_approach(std::nullopt);
      return;
    }
    logic_->set_approach(v);
    approach_since_ = now();
    RCLCPP_INFO(get_logger(), "박스 접근: 정지 거리 %.3f m (해제 %.3f m)", logic_->stop_m(), logic_->release_m());
  }

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
  double stop_m_ = 0.25, min_approach_m_ = 0.10, approach_max_s_ = 60.0;
  rclcpp::Time approach_since_{0, 0, RCL_ROS_TIME};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::optional<bool> last_;
  std::string last_reason_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::ObstacleStopNode)
