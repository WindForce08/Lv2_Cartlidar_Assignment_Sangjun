// RearObstacleNode (perception_container)
//   뒤 D435 의 컬러에 맞춘 깊이로 차체 뒤 통로의 장애물을 찾아 "뒤로 가면 안 되는지"를 발행한다.
//   거리 계산은 rear_obstacle.hpp, 정지 · 해제 규칙은 obstacle_stop.hpp (TFmini 비상 정지와 같은 규칙)
//   출력  stop_topic     std_msgs/Bool    true = 뒤로 가면 안 됨   (Reliable + TransientLocal, 바뀔 때 바로 + publish_rate 주기)
//         reason_topic   std_msgs/String  obstacle · stale · clear · no_return
//         distance_topic std_msgs/Float32 뒤 끝에서 가장 가까운 장애물 [m], 없으면 NaN (깊이 한 장마다)
//   이 노드는 알리기만 한다. 후진을 막는 것은 제어 쪽에서 이 토픽을 받아 한다 (앞으로 갈 때는 쓰지 않음)
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "perception/obstacle_stop.hpp"
#include "perception/rear_obstacle.hpp"
#include "perception/tf_helper.hpp"

namespace perception
{

class RearObstacleNode : public rclcpp::Node
{
public:
  explicit RearObstacleNode(const rclcpp::NodeOptions & options)
  : Node("rear_obstacle", options)
  {
    RearConfig r;
    r.x_edge = declare_parameter("x_edge", r.x_edge);
    r.y_half = declare_parameter("y_half", r.y_half);
    r.z_min = declare_parameter("z_min", r.z_min);
    r.z_max = declare_parameter("z_max", r.z_max);
    r.range_m = declare_parameter("range_m", r.range_m);
    r.depth_min_m = declare_parameter("depth_min_m", r.depth_min_m);
    r.depth_max_m = declare_parameter("depth_max_m", r.depth_max_m);
    r.stride = static_cast<int>(declare_parameter("stride", r.stride));
    r.min_points = static_cast<int>(declare_parameter("min_points", r.min_points));
    cfg_ = r;
    depth_scale_ = declare_parameter("depth_scale", 0.001);   // CV_16U 깊이 한 단위 [m] (realsense2_camera 기본 mm)
    frame_ = declare_parameter("target_frame", std::string("base_footprint"));

    StopConfig c;
    c.stop_m = declare_parameter("stop_m", 0.40);
    c.release_m = declare_parameter("release_m", 0.50);
    c.release_count = static_cast<int>(declare_parameter("release_count", c.release_count));
    c.near_m = declare_parameter("near_m", 0.0);   // 0 = "없음"을 너무 가까움으로 보지 않음 (카메라는 없음 = 비었음)
    c.stale_s = declare_parameter("stale_s", 0.5);
    if (!(c.release_m > c.stop_m)) {throw std::runtime_error("release_m 은 stop_m 보다 커야 함");}
    if (!(r.range_m >= c.release_m)) {throw std::runtime_error("range_m 은 release_m 이상이어야 함 (해제 거리를 잴 수 있게)");}
    logic_ = std::make_unique<ObstacleStop>(c);
    tf_ = std::make_unique<TfHelper>(*this);

    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter("stop_topic", std::string("/perception/rear_obstacle")), out_qos);
    reason_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter("reason_topic", std::string("/perception/rear_obstacle/reason")), out_qos);
    dist_pub_ = create_publisher<std_msgs::msg::Float32>(
      declare_parameter("distance_topic", std::string("/perception/rear_obstacle/distance")), 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter("info_topic", std::string("/top_camera_rear/color/camera_info")), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        K_ = PinholeK{m->k[0], m->k[4], m->k[2], m->k[5]};
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter("depth_topic", std::string("/top_camera_rear/aligned_depth_to_color/image_raw")),
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {on_depth(m);});
    const double rate = declare_parameter("publish_rate", 10.0);
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
        logic_->check_stale(now().seconds());
        publish(true);
      });
    RCLCPP_INFO(get_logger(), "통로 x < %.3f, |y| ≤ %.3f, 높이 %.2f~%.2f m · 정지 %.2f m 미만, 해제 %.2f m 이상 %d회, 입력 끊김 %.2f s",
      r.x_edge, r.y_half, r.z_min, r.z_max, c.stop_m, c.release_m, c.release_count, c.stale_s);
  }

private:
  void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr & m)
  {
    if (!K_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 를 아직 못 받음");
      return;
    }
    const auto t = tf_->fixed(frame_, m->header.frame_id);
    if (!t) {return;}   // TF 가 없으면 측정 없음 → stale_s 뒤 정지 (TfHelper 가 경고)
    cv_bridge::CvImageConstPtr cv;
    try {
      cv = cv_bridge::toCvShare(m);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "깊이 영상 변환 실패: %s", e.what());
      return;
    }
    const tf2::Matrix3x3 & R = t->getBasis();
    const tf2::Vector3 & p = t->getOrigin();
    const cv::Matx34d T(R[0][0], R[0][1], R[0][2], p.x(), R[1][0], R[1][1], R[1][2], p.y(), R[2][0], R[2][1], R[2][2], p.z());
    const RearResult res = nearest_rear(cv->image, depth_scale_, *K_, T, cfg_);
    logic_->update(now().seconds(), res.distance);
    std_msgs::msg::Float32 d;
    d.data = res.distance ? static_cast<float>(*res.distance) : std::numeric_limits<float>::quiet_NaN();
    dist_pub_->publish(d);
    publish(false);
  }

  // always=false 면 바뀌었을 때만
  void publish(bool always)
  {
    const bool s = logic_->stopped();
    const std::string & why = logic_->reason();
    if (!always && last_ && *last_ == s && last_reason_ == why) {return;}
    if (!last_ || *last_ != s) {
      RCLCPP_INFO(get_logger(), "후방 장애물 %s (%s)", s ? "있음" : "없음", why.c_str());
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

  RearConfig cfg_;
  double depth_scale_;
  std::string frame_;
  std::optional<PinholeK> K_;
  std::unique_ptr<ObstacleStop> logic_;
  std::unique_ptr<TfHelper> tf_;
  std::optional<bool> last_;
  std::string last_reason_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::RearObstacleNode)
