// OpenCRHardware — ros2_control SystemInterface 플러그인 (Firmware 팀)
// controller_manager(① ros2_control_node) 안에서 로드되어 100 Hz 로 read()/write() 가 불린다.
//   command: 바퀴 velocity × N(≤6) · 팔 position × 6 · 그리퍼 position × 0/1
//   state  : 관절 position · velocity · effort · IMU 10
//   USB CDC 후보 포트 2개 — CMD 0x01 송신 / STATE 0x81 수신, STATE.role로 보드 판별
//   E-stop 은 이 플러그인 한 곳에서 판단 (잠금식, estop_latch.hpp):
//     /joy 구독 (heartbeat · E-stop · 리셋 버튼) → /e_stop 발행 (TransientLocal, 10 Hz)
//     /diagnostics 발행 (opencr/link · opencr/motors · opencr/battery · opencr/estop, 1 Hz + 오류 즉시)
//   토픽은 하드웨어 컴포넌트 노드(get_node(), controller_manager executor 가 돌림)에서 처리하고,
//   read()/write() 실시간 루프와는 atomic 값으로만 주고받는다 (실시간 루프에서 할당 · 발행 없음).
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "opencr_hardware/estop_latch.hpp"
#include "opencr_hardware/opencr_protocol.hpp"
#include "opencr_hardware/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace opencr_hardware
{

class OpenCRHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Role { WHEEL, ARM, GRIPPER };
  struct JointMap
  {
    std::string name;
    Role role;
    int slot;            // WHEEL 0~5 · ARM 0~5 · GRIPPER 0
    double lo = -std::numeric_limits<double>::infinity();   // 팔 명령 한계 (URDF command_interface min/max)
    double hi = std::numeric_limits<double>::infinity();
  };

  bool poll_serial();
  bool wait_first_state(double timeout_s);
  bool roles_ready() const;
  void refresh_aggregate_state();
  void apply_state_to_interfaces();
  void set_state_if(const std::string & name, double v);
  bool send_cmd(bool torque_on);
  bool send_cmd_for_role(uint8_t role, bool torque_on);
  void update_estop();             // read() 안에서 100 Hz
  void setup_ros_io();             // 하드웨어 컴포넌트 노드에 /joy · /e_stop · /diagnostics
  void publish_estop();
  void publish_diagnostics(bool force);
  static int64_t steady_ns();

  std::vector<JointMap> joints_;
  std::string imu_name_;
  // 파라미터 (URDF <hardware><param>)
  // 후보 포트는 역할을 미리 가정하지 않는다. 각 포트의 STATE.role로 WHEEL/ARM을 판별한다.
  std::string port_ = "/dev/ttyACM0,/dev/ttyACM1";
  double comm_timeout_s_ = 0.5;
  double first_state_timeout_s_ = 2.0;
  bool joy_required_ = true;
  double joy_timeout_s_ = 0.5;
  int estop_button_ = 6;
  int reset_button_ = 7;
  double battery_warn_v_ = 11.1;
  double battery_error_v_ = 10.5;
  bool listen_only_ = false;   // true: CMD 를 한 번도 보내지 않음 — 토크 · 모션 없이 STATE 수신만 확인
  bool auto_rearm_ = false;    // true: 잠금이 풀리면 리셋 버튼 없이 REARM (조이스틱 없는 벤치 전용, 실기 운용 금지)

  struct BoardLink
  {
    explicit BoardLink(std::string path_in) : path(std::move(path_in)) {}
    std::string path;
    SerialPort serial;
    opencr_proto::FrameParser parser;
    opencr_proto::State state{};
    bool have_state = false;
    rclcpp::Time last_state_time{0, 0, RCL_STEADY_TIME};
  };
  std::vector<std::unique_ptr<BoardLink>> links_;
  opencr_proto::State latest_{};
  bool have_state_ = false;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  uint8_t seq_ = 0;
  uint8_t last_error_ = 0;

  // E-stop (실시간 루프 ↔ 노드 콜백, atomic 만)
  EstopLatch latch_;
  RearmPulse rearm_;              // v5: 운용자 리셋 → 팔 보드에 REARM 펄스
  bool rearm_flag_ = false;       // 이번 주기 팔 CMD 에 FLAG_REARM
  bool prev_arm_hold_ = false;    // STATE ERR_HOLD 상승 → 잠금 trip
  bool prev_reset_btn_ = false;
  std::atomic<bool> estop_{true};
  std::atomic<int64_t> joy_rx_ns_{0};
  std::atomic<bool> joy_estop_btn_{false};
  std::atomic<bool> joy_reset_btn_{false};
  std::atomic<bool> link_ok_{false};
  std::atomic<uint8_t> fw_error_{0};
  std::atomic<uint8_t> fw_version_{0};
  std::atomic<float> battery_v_{0.f};
  std::atomic<uint32_t> bad_frames_{0};
  std::atomic<int> estop_reason_code_{1};   // 0 없음 · 1 시작 · 2 링크 · 3 물리 · 4 버튼 · 5 joy · 6 팔 HOLD

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr estop_timer_, diag_timer_;
  uint8_t last_diag_level_ = 0;
};

}  // namespace opencr_hardware
