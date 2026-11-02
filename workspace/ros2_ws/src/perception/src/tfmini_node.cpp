// TFminiNode (bringup control_container)
//   TFmini Plus(1층 정면 바닥)는 퍽이 놓인 박스 앞면까지의 거리를 잰다 (drive FINE 정렬 · obstacle_stop). 팔 거리(/arm_range)가 아님.
//   TFmini Plus 는 UART 로만 값을 내고 Lyrical 에 ROS 드라이버가 없어 시리얼을 직접 읽는다.
//   /dev/tfmini 115200 bps 를 읽기 스레드에서 파싱 → range_topic(기본 /tfmini/range) 으로 sensor_msgs/Range 발행
//   - 새 측정이 있을 때만 발행한다 (publish_rate 로 묶음, 같은 측정을 두 번 내지 않음). stamp = 프레임을 받은 시각
//   - 신호세기 < min_strength · 측정 범위 밖 → range = +inf (무효)
//   - stale_timeout_s 동안 새 프레임이 없으면 내지 않는다. 받는 쪽 obstacle_stop 이 stale_s 뒤에 끊김으로 정지한다
//     (끊긴 동안 지금 시각의 +inf 를 내면 obstacle_stop 이 새 측정으로 받아 끊김 정지를 못 하고, 직전 거리가 멀었으면 '비었음'으로 풂)
//   - output_mm: 켤 때 출력 형식을 mm 로 바꾸는 명령을 보낸다 (기본 cm 는 1 cm 단위라 박스 정밀 정지에 거침). 저장하지 않으므로 매번 보냄
//   - 보정: calib_raw_m · calib_true_m (박스 재질 앞 줄자 실측, tfmini_parser.hpp tfmini_calibrate) + offset_m.
//     사양 정확도는 ±5 cm (0.1~5 m) 라 실측 보정 없이는 박스 거리로 쓰기 어렵다 ★
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "perception/tfmini_parser.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace perception
{
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
    output_mm_ = declare_parameter("output_mm", true);
    calib_raw_ = declare_parameter("calib_raw_m", std::vector<double>{});
    calib_true_ = declare_parameter("calib_true_m", std::vector<double>{});
    offset_m_ = declare_parameter("offset_m", 0.0);
    if (calib_raw_.size() != calib_true_.size() || !std::is_sorted(calib_raw_.begin(), calib_raw_.end())) {
      throw std::runtime_error("calib_raw_m · calib_true_m 은 같은 길이, calib_raw_m 오름차순이어야 함");
    }
    RCLCPP_INFO(get_logger(), "출력 %s, 보정점 %zu개, 오프셋 %.3f m", output_mm_ ? "mm" : "cm", calib_raw_.size(), offset_m_);

    pub_ = create_publisher<sensor_msgs::msg::Range>(declare_parameter("range_topic", std::string("/tfmini/range")),
        rclcpp::SensorDataQoS());
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
    fd_ = ::open(port_.c_str(), (output_mm_ ? O_RDWR : O_RDONLY) | O_NOCTTY);
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
    if (output_mm_) {
      // 출력 형식 mm. 센서가 같은 5바이트로 답하지만 헤더(0x5A)가 측정 프레임(0x59)과 달라 파서가 버린다
      if (::write(fd_, TFMINI_CMD_OUTPUT_MM, sizeof(TFMINI_CMD_OUTPUT_MM)) != static_cast<ssize_t>(sizeof(TFMINI_CMD_OUTPUT_MM))) {
        RCLCPP_ERROR(get_logger(), "%s 출력 형식(mm) 명령 쓰기 실패 — 다시 연결", port_.c_str());
        close_port();
        return false;
      }
      tcdrain(fd_);
    }
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
  }

  void read_loop()
  {
    TfminiParser parser(output_mm_ ? 0.001 : 0.01);
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
          s.distance_m = tfmini_calibrate(s.distance_m, calib_raw_, calib_true_, offset_m_);
          std::lock_guard<std::mutex> lk(mtx_);
          latest_ = s;
          latest_time_ = std::chrono::steady_clock::now();
          latest_stamp_ = now();
          have_ = true;
          fresh_ = true;
        }
      }
    }
  }

  void publish()
  {
    sensor_msgs::msg::Range m;
    m.header.frame_id = frame_id_;
    m.radiation_type = sensor_msgs::msg::Range::INFRARED;
    m.field_of_view = static_cast<float>(fov_);
    m.min_range = static_cast<float>(min_range_);
    m.max_range = static_cast<float>(max_range_);
    m.range = std::numeric_limits<float>::infinity();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_time_).count();
      if (have_ && age <= stale_s_) {
        if (!fresh_) {return;}   // 지난번에 낸 측정 → 다시 내지 않음
        fresh_ = false;
        m.header.stamp = latest_stamp_;
        if (tfmini_valid(latest_, static_cast<uint16_t>(min_strength_), min_range_, max_range_)) {
          m.range = static_cast<float>(latest_.distance_m);
        }
      } else {
        return;   // 끊김: 내지 않는다 (위 설명)
      }
    }
    pub_->publish(m);
  }

  std::string port_, frame_id_;
  int baud_, min_strength_;
  double min_range_, max_range_, fov_, stale_s_;
  bool output_mm_ = true;
  std::vector<double> calib_raw_, calib_true_;
  double offset_m_ = 0.0;
  int fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread reader_;
  std::mutex mtx_;
  TfminiSample latest_;
  std::chrono::steady_clock::time_point latest_time_{};
  rclcpp::Time latest_stamp_;
  bool have_ = false;
  bool fresh_ = false;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::TFminiNode)
