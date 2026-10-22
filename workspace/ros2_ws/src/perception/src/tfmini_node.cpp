// TFminiNode (Perception 팀 작성 · ⑧ control_container 에서 실행)
//   /dev/tfmini 115200 bps 를 읽기 스레드에서 파싱 → ~50 Hz 로 sensor_msgs/Range /arm_range 발행
//   신호세기 < 100 · 0.1~12 m 밖 · 0.2 s 이상 새 프레임 없음 → range = +inf (무효)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "perception/tfmini_parser.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace perception
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;

class TFminiNode : public rclcpp::Node
{
public:
  explicit TFminiNode(const rclcpp::NodeOptions & options)
  : Node("tfmini_node", options)
  {
    port_ = declare_parameter("port", std::string("/dev/tfmini"));
    baud_ = declare_parameter("baud", 115200);
    frame_id_ = declare_parameter("frame_id", std::string("tfmini_link"));
    min_strength_ = declare_parameter("min_strength", 100);
    min_range_ = declare_parameter("min_range", 0.1);
    max_range_ = declare_parameter("max_range", 12.0);
    fov_ = declare_parameter("field_of_view", 0.063);   // TFmini Plus 3.6°
    stale_s_ = declare_parameter("stale_timeout_s", 0.2);
    const double rate = declare_parameter("publish_rate", 50.0);

    pub_ = create_publisher<sensor_msgs::msg::Range>(names::ARM_RANGE, rclcpp::SensorDataQoS());
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {publish();}, cbg_);
    reader_ = std::thread([this]() {read_loop();});   // 무거운 작업(시리얼)은 별도 스레드
  }

  ~TFminiNode() override
  {
    running_ = false;
    if (reader_.joinable()) {reader_.join();}
    close_port();
  }

private:
  static speed_t to_speed(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 57600: return B57600;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B115200;
    }
  }

  bool open_port()
  {
    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {return false;}
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, to_speed(baud_));
    cfsetospeed(&tio, to_speed(baud_));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;   // 0.1 s 읽기 타임아웃 → 종료 신호를 빨리 봄
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIFLUSH);
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
  }

  void read_loop()
  {
    TfminiParser parser;
    uint8_t buf[256];
    while (running_) {
      if (fd_ < 0) {
        if (!open_port()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 열기 실패 — 1 s 뒤 다시 시도", port_.c_str());
          std::this_thread::sleep_for(1s);
          continue;
        }
        RCLCPP_INFO(get_logger(), "%s 열림 (%d bps)", port_.c_str(), baud_);
      }
      const ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n < 0) {
        RCLCPP_ERROR(get_logger(), "%s 읽기 오류 — 다시 연결", port_.c_str());
        close_port();
        continue;
      }
      for (ssize_t i = 0; i < n; ++i) {
        TfminiSample s;
        if (parser.push(buf[i], s)) {
          std::lock_guard<std::mutex> lk(mtx_);
          latest_ = s;
          latest_time_ = std::chrono::steady_clock::now();
          have_ = true;
        }
      }
    }
  }

  void publish()
  {
    sensor_msgs::msg::Range m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.radiation_type = sensor_msgs::msg::Range::INFRARED;
    m.field_of_view = static_cast<float>(fov_);
    m.min_range = static_cast<float>(min_range_);
    m.max_range = static_cast<float>(max_range_);
    m.range = std::numeric_limits<float>::infinity();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_time_).count();
      if (have_ && age <= stale_s_ &&
        tfmini_valid(latest_, static_cast<uint16_t>(min_strength_), min_range_, max_range_))
      {
        m.range = static_cast<float>(latest_.distance_m);
      }
    }
    pub_->publish(m);
  }

  std::string port_, frame_id_;
  int baud_, min_strength_;
  double min_range_, max_range_, fov_, stale_s_;
  int fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread reader_;
  std::mutex mtx_;
  TfminiSample latest_;
  std::chrono::steady_clock::time_point latest_time_{};
  bool have_ = false;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::TFminiNode)
