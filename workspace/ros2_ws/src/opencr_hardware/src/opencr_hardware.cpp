#include "opencr_hardware/opencr_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_interfaces/names.hpp"

namespace opencr_hardware
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
namespace hi = hardware_interface;
using hi::return_type;

namespace
{
std::string param_or(const std::unordered_map<std::string, std::string> & m, const std::string & key,
  const std::string & def)
{
  auto it = m.find(key);
  return it == m.end() ? def : it->second;
}
const char * IMU_SUFFIX[10] = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
}  // namespace

hi::CallbackReturn OpenCRHardware::on_init(const hi::HardwareComponentInterfaceParams & params)
{
  if (hi::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  port_ = param_or(hw, "ports", param_or(hw, "port", port_));
  comm_timeout_s_ = std::stod(param_or(hw, "comm_timeout_s", "0.5"));
  first_state_timeout_s_ = std::stod(param_or(hw, "first_state_timeout_s", "2.0"));
  joy_required_ = param_or(hw, "joy_required", "true") == "true";
  joy_timeout_s_ = std::stod(param_or(hw, "joy_timeout_s", "0.5"));
  estop_button_ = std::stoi(param_or(hw, "estop_button", "6"));
  reset_button_ = std::stoi(param_or(hw, "reset_button", "7"));
  battery_warn_v_ = std::stod(param_or(hw, "battery_warn_v", "11.1"));
  battery_error_v_ = std::stod(param_or(hw, "battery_error_v", "10.5"));

  // 관절 ↔ 패킷 슬롯: <param name="role">wheel|arm|gripper</param> <param name="slot">N</param>
  std::vector<bool> wheel_used(opencr_proto::MAX_WHEELS, false), arm_used(opencr_proto::ARM_JOINTS, false);
  for (const auto & j : info_.joints) {
    const std::string role = param_or(j.parameters, "role", "");
    const int slot = std::stoi(param_or(j.parameters, "slot", "-1"));
    JointMap m{j.name, Role::WHEEL, slot};
    if (role == "wheel") {
      m.role = Role::WHEEL;
      if (slot < 0 || slot >= opencr_proto::MAX_WHEELS || wheel_used[slot]) {
        RCLCPP_FATAL(get_logger(), "바퀴 관절 %s: slot %d 잘못됨 (0~5, 중복 불가)", j.name.c_str(), slot);
        return CallbackReturn::ERROR;
      }
      wheel_used[slot] = true;
      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != hi::HW_IF_VELOCITY) {
        RCLCPP_FATAL(get_logger(), "바퀴 관절 %s 는 velocity 명령 하나만 가져야 함", j.name.c_str());
        return CallbackReturn::ERROR;
      }
    } else if (role == "arm") {
      m.role = Role::ARM;
      if (slot < 0 || slot >= opencr_proto::ARM_JOINTS || arm_used[slot]) {
        RCLCPP_FATAL(get_logger(), "팔 관절 %s: slot %d 잘못됨 (0~5, 중복 불가)", j.name.c_str(), slot);
        return CallbackReturn::ERROR;
      }
      arm_used[slot] = true;
      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != hi::HW_IF_POSITION) {
        RCLCPP_FATAL(get_logger(), "팔 관절 %s 는 position 명령 하나만 가져야 함", j.name.c_str());
        return CallbackReturn::ERROR;
      }
    } else if (role == "gripper") {
      m.role = Role::GRIPPER;
      m.slot = 0;
    } else {
      RCLCPP_FATAL(get_logger(), "관절 %s: role 파라미터가 wheel/arm/gripper 중 하나여야 함", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    joints_.push_back(m);
  }
  if (!info_.sensors.empty()) {
    imu_name_ = info_.sensors.front().name;
  }
  RCLCPP_INFO(get_logger(), "OpenCRHardware: 관절 %zu개, IMU '%s', 후보 포트 %s", joints_.size(),
    imu_name_.c_str(), port_.c_str());
  return CallbackReturn::SUCCESS;
}

hi::CallbackReturn OpenCRHardware::on_configure(const rclcpp_lifecycle::State &)
{
  links_.clear();
  std::istringstream input(port_);
  std::string path;
  std::string err;
  while (std::getline(input, path, ',')) {
    const auto first = path.find_first_not_of(" \t");
    const auto last = path.find_last_not_of(" \t");
    if (first == std::string::npos) {continue;}
    path = path.substr(first, last - first + 1);
    auto link = std::make_unique<BoardLink>(path);
    if (!link->serial.open(path, B115200, err)) {
      RCLCPP_ERROR(get_logger(), "후보 포트 열기 실패: %s", err.c_str());
      links_.clear();
      return CallbackReturn::ERROR;
    }
    links_.push_back(std::move(link));
  }
  if (links_.size() != 2) {
    RCLCPP_ERROR(get_logger(), "역할 판별을 위해 후보 포트를 정확히 두 개 지정해야 합니다: %s", port_.c_str());
    links_.clear();
    return CallbackReturn::ERROR;
  }
  setup_ros_io();
  return CallbackReturn::SUCCESS;
}

int64_t OpenCRHardware::steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// /joy · /e_stop · /diagnostics — 하드웨어 컴포넌트 노드 (controller_manager executor 가 콜백 · 타이머를 돌림)
void OpenCRHardware::setup_ros_io()
{
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(get_logger(), "하드웨어 컴포넌트 노드 없음 — /joy · /e_stop · /diagnostics 비활성 "
      "(joy_required=%s)", joy_required_ ? "true → 계속 잠김" : "false");
    return;
  }
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(names::JOY, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {
        auto btn = [&](int i) {return i >= 0 && i < static_cast<int>(m->buttons.size()) && m->buttons[i] != 0;};
        joy_estop_btn_.store(btn(estop_button_));
        joy_reset_btn_.store(btn(reset_button_));
        joy_rx_ns_.store(steady_ns());
      });
  estop_pub_ = node->create_publisher<std_msgs::msg::Bool>(names::E_STOP, qos::e_stop());
  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(names::DIAGNOSTICS, 10);
  estop_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        publish_estop();
        publish_diagnostics(false);   // 오류로 바뀌면 즉시
      });
  diag_timer_ = node->create_wall_timer(std::chrono::seconds(1), [this]() {publish_diagnostics(true);});
}

void OpenCRHardware::publish_estop()
{
  std_msgs::msg::Bool m;
  m.data = estop_.load();
  estop_pub_->publish(m);
}

void OpenCRHardware::publish_diagnostics(bool force)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = get_node()->now();
  auto add = [&](const std::string & name, uint8_t level, const std::string & msg,
    std::vector<std::pair<std::string, std::string>> kv = {}) {
      DiagnosticStatus s;
      s.name = "opencr/" + name;   // 이름 규약 = 계약 (mission 이 opencr/ 로 거름)
      s.hardware_id = "opencr";
      s.level = level;
      s.message = msg;
      for (auto & p : kv) {KeyValue k; k.key = p.first; k.value = p.second; s.values.push_back(k);}
      arr.status.push_back(s);
    };
  const uint8_t err = fw_error_.load();
  const bool link = link_ok_.load();
  const float bat = battery_v_.load();
  add("link", link ? DiagnosticStatus::OK : DiagnosticStatus::ERROR, link ? "정상" : "STATE 무수신",
    {{"bad_frames", std::to_string(bad_frames_.load())}, {"ports", port_}});
  const bool motor_bad = err & (opencr_proto::ERR_MOTOR_HW | opencr_proto::ERR_SYNCREAD |
      opencr_proto::ERR_MOTOR_CONFIG | opencr_proto::ERR_ROLE_MISMATCH);
  add("motors", motor_bad ? DiagnosticStatus::ERROR : DiagnosticStatus::OK,
    (err & opencr_proto::ERR_MOTOR_CONFIG) ? "부팅 점검 실패 (응답 · 모델 · Homing Offset)" :
    (err & opencr_proto::ERR_MOTOR_HW) ? "모터 Hardware Error" :
    (err & opencr_proto::ERR_ROLE_MISMATCH) ? "CMD 역할 불일치" :
    (err & opencr_proto::ERR_SYNCREAD) ? "SyncRead 응답 부족" : "정상",
    {{"error_bits", std::to_string(err)}, {"fw_version", std::to_string(fw_version_.load())}});
  uint8_t bl = DiagnosticStatus::OK;
  if (link && bat < battery_error_v_) {bl = DiagnosticStatus::ERROR;} else if (link && bat < battery_warn_v_) {bl = DiagnosticStatus::WARN;}
  char vb[16];
  std::snprintf(vb, sizeof(vb), "%.2f", bat);
  add("battery", bl, bl == DiagnosticStatus::OK ? "정상" : (bl == DiagnosticStatus::WARN ? "전압 낮음" : "전압 위험"), {{"voltage", vb}});
  static const char * reasons[] = {"", "시작 — 리셋 버튼 필요", "OpenCR 링크 끊김", "물리 E-stop 스위치", "조이스틱 E-stop 버튼", "조이스틱 무수신"};
  const int rc = estop_reason_code_.load();
  add("estop", estop_.load() ? DiagnosticStatus::WARN : DiagnosticStatus::OK, estop_.load() ? std::string("잠김: ") + reasons[rc] : "해제");

  uint8_t worst = 0;
  for (const auto & s : arr.status) {worst = std::max(worst, s.level);}
  const bool became_error = worst >= DiagnosticStatus::ERROR && last_diag_level_ < DiagnosticStatus::ERROR;
  last_diag_level_ = worst;
  if (force || became_error) {diag_pub_->publish(arr);}
}

hi::CallbackReturn OpenCRHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  joy_sub_.reset();
  estop_timer_.reset();
  diag_timer_.reset();
  links_.clear();
  return CallbackReturn::SUCCESS;
}

bool OpenCRHardware::poll_serial()
{
  uint8_t buf[512];
  bool io_ok = true;
  for (auto & link : links_) {
    for (int k = 0; k < 16; ++k) {   // 각 포트에서 한 주기에 최대 8 KB
      const ssize_t n = link->serial.read_some(buf, sizeof(buf));
      if (n < 0) {io_ok = false; break;}
      if (n == 0) {break;}
      for (ssize_t i = 0; i < n; ++i) {
        if (link->parser.push(buf[i]) && link->parser.type() == opencr_proto::TYPE_STATE) {
          opencr_proto::State s;
          if (opencr_proto::decode_state(link->parser.payload(), link->parser.length(), s)) {
            link->state = s;
            link->have_state = true;
            link->last_state_time = steady_.now();
          }
        }
      }
    }
  }
  refresh_aggregate_state();
  uint32_t bad = 0;
  for (const auto & link : links_) {bad += link->parser.bad_frames();}
  bad_frames_.store(bad);
  return io_ok;
}

bool OpenCRHardware::roles_ready() const
{
  int wheels = 0, arms = 0;
  for (const auto & link : links_) {
    if (!link->have_state) {continue;}
    if (link->state.fw_version != opencr_proto::PROTOCOL_VERSION) {return false;}
    if (link->state.role == opencr_proto::ROLE_WHEEL) {++wheels;}
    else if (link->state.role == opencr_proto::ROLE_ARM) {++arms;}
  }
  return wheels == 1 && arms == 1;
}

void OpenCRHardware::refresh_aggregate_state()
{
  opencr_proto::State merged{};
  bool wheel = false, arm = false;
  for (const auto & link : links_) {
    if (!link->have_state) {continue;}
    const auto & s = link->state;
    if (s.role == opencr_proto::ROLE_WHEEL) {
      wheel = true;
      for (int i = 0; i < opencr_proto::MAX_WHEELS; ++i) {
        merged.pos[i] = s.pos[i]; merged.vel[i] = s.vel[i]; merged.effort[i] = s.effort[i];
      }
      std::copy(std::begin(s.quat), std::end(s.quat), std::begin(merged.quat));
      std::copy(std::begin(s.gyro), std::end(s.gyro), std::begin(merged.gyro));
      std::copy(std::begin(s.accel), std::end(s.accel), std::begin(merged.accel));
      merged.battery_v = s.battery_v;
    } else if (s.role == opencr_proto::ROLE_ARM) {
      arm = true;
      for (int i = 0; i < opencr_proto::ARM_JOINTS; ++i) {
        const int k = opencr_proto::MAX_WHEELS + i;
        merged.pos[k] = s.pos[k]; merged.vel[k] = s.vel[k]; merged.effort[k] = s.effort[k];
      }
    }
    merged.error |= s.error;
    if (s.fw_version != opencr_proto::PROTOCOL_VERSION) {merged.fw_version = s.fw_version;}
  }
  have_state_ = wheel && arm;
  if (have_state_) {merged.fw_version = opencr_proto::PROTOCOL_VERSION;}
  latest_ = merged;
}

bool OpenCRHardware::wait_first_state(double timeout_s)
{
  const auto t0 = steady_.now();
  while ((steady_.now() - t0).seconds() < timeout_s) {
    if (!poll_serial()) {return false;}
    if (roles_ready()) {
      have_state_ = true;
      return send_cmd(false);   // 역할이 확인된 두 보드에 토크 OFF 명령
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

hi::CallbackReturn OpenCRHardware::on_activate(const rclcpp_lifecycle::State &)
{
  bool any_open = false;
  for (const auto & link : links_) {any_open = any_open || link->serial.is_open();}
  if (!any_open && on_configure(rclcpp_lifecycle::State()) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!wait_first_state(first_state_timeout_s_)) {
    RCLCPP_ERROR(get_logger(), "%.1f s 안에 WHEEL·ARM 역할 STATE를 모두 받지 못함", first_state_timeout_s_);
    return CallbackReturn::ERROR;
  }
  // 펌웨어 버전 확인 — 다르면 패킷 해석이 어긋나므로 명령을 보내지 않음
  if (latest_.fw_version != opencr_proto::PROTOCOL_VERSION) {
    RCLCPP_ERROR(get_logger(), "두 보드 모두 프로토콜 v%u이어야 합니다 — 역할별 펌웨어를 다시 올리세요",
      opencr_proto::PROTOCOL_VERSION);
    return CallbackReturn::ERROR;
  }
  if (latest_.error & opencr_proto::ERR_MOTOR_CONFIG) {
    RCLCPP_ERROR(get_logger(), "OpenCR 부팅 점검 실패 (모터 응답 · 모델 · Homing Offset) — Dynamixel Wizard 로 확인");
    return CallbackReturn::ERROR;
  }
  apply_state_to_interfaces();
  // 팔 명령 초기값 = 현재 위치 (토크 ON 순간 튀지 않게), 바퀴 = 0
  for (const auto & j : joints_) {
    const std::string cmd_if = j.name + "/" + (j.role == Role::WHEEL ? hi::HW_IF_VELOCITY : hi::HW_IF_POSITION);
    if (j.role == Role::WHEEL) {
      set_command(cmd_if, 0.0);
    } else if (j.role == Role::ARM) {
      set_command(cmd_if, static_cast<double>(latest_.pos[opencr_proto::MAX_WHEELS + j.slot]));
    } else {
      set_command(cmd_if, 0.0);   // 그리퍼 피드백 없음 — 0(열림 기준)에서 시작 ★
    }
  }
  RCLCPP_INFO(get_logger(), "OpenCRHardware 활성화 (팔 명령 = 현재 위치)");
  return CallbackReturn::SUCCESS;
}

hi::CallbackReturn OpenCRHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 바퀴 0, 팔은 마지막 명령 위치 유지 (토크를 끄면 팔이 처지므로 끄지 않음)
  for (const auto & j : joints_) {
    if (j.role == Role::WHEEL) {set_command(j.name + "/" + hi::HW_IF_VELOCITY, 0.0);}
  }
  send_cmd(true);
  return CallbackReturn::SUCCESS;
}

void OpenCRHardware::set_state_if(const std::string & name, double v)
{
  if (has_state(name)) {set_state(name, v);}   // URDF 에 없는 상태 인터페이스는 건너뜀
}

void OpenCRHardware::apply_state_to_interfaces()
{
  for (const auto & j : joints_) {
    if (j.role == Role::GRIPPER) {
      // 그리퍼 개폐 피드백 없음 (액추에이터 미정) → 명령값을 상태로 되돌려 줌
      const double c = get_command<double>(j.name + "/" + hi::HW_IF_POSITION);
      set_state_if(j.name + "/" + hi::HW_IF_POSITION, std::isfinite(c) ? c : 0.0);
      continue;
    }
    const int k = (j.role == Role::WHEEL ? 0 : opencr_proto::MAX_WHEELS) + j.slot;
    set_state_if(j.name + "/" + hi::HW_IF_POSITION, latest_.pos[k]);
    set_state_if(j.name + "/" + hi::HW_IF_VELOCITY, latest_.vel[k]);
    set_state_if(j.name + "/" + hi::HW_IF_EFFORT, latest_.effort[k]);
  }
  if (!imu_name_.empty()) {
    const float v[10] = {latest_.quat[1], latest_.quat[2], latest_.quat[3], latest_.quat[0],
      latest_.gyro[0], latest_.gyro[1], latest_.gyro[2],
      latest_.accel[0], latest_.accel[1], latest_.accel[2]};
    for (int i = 0; i < 10; ++i) {
      set_state_if(imu_name_ + "/" + IMU_SUFFIX[i], v[i]);
    }
  }
}

void OpenCRHardware::update_estop()
{
  EstopInputs in;
  in.joy_required = joy_required_;
  in.joy_fresh = (steady_ns() - joy_rx_ns_.load()) * 1e-9 <= joy_timeout_s_;
  in.estop_button = joy_estop_btn_.load();
  in.reset_button = joy_reset_btn_.load();
  in.physical_pressed = latest_.error & opencr_proto::ERR_HW_ESTOP;
  in.link_ok = link_ok_.load();
  const bool was = estop_.load();
  const bool now_latched = latch_.update(in);
  estop_.store(now_latched);
  if (now_latched != was) {
    const std::string & r = latch_.reason();
    estop_reason_code_.store(!now_latched ? 0 : r.find("시작") != std::string::npos ? 1 :
      r.find("링크") != std::string::npos ? 2 : r.find("물리") != std::string::npos ? 3 :
      r.find("버튼") != std::string::npos ? 4 : 5);
    RCLCPP_WARN(get_logger(), "E-stop %s %s", now_latched ? "잠김:" : "해제", r.c_str());
  }
}

return_type OpenCRHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const bool io_ok = poll_serial();
  const auto now = steady_.now();
  bool wheel_fresh = false, arm_fresh = false;
  for (const auto & link : links_) {
    if (!link->have_state || (now - link->last_state_time).seconds() > comm_timeout_s_) {continue;}
    if (link->state.role == opencr_proto::ROLE_WHEEL) {wheel_fresh = true;}
    else if (link->state.role == opencr_proto::ROLE_ARM) {arm_fresh = true;}
  }
  const bool fresh = wheel_fresh && arm_fresh;
  link_ok_.store(io_ok && fresh);
  update_estop();
  if (!io_ok || !fresh) {
    // 한쪽 링크만 살아 있으면 그 보드에 정지 명령을 보내고, 끊긴 보드에는 펌웨어 타임아웃이 적용된다.
    send_cmd(true);
    RCLCPP_ERROR(get_logger(), "WHEEL·ARM OpenCR 중 하나 이상 링크 오류 (시리얼 분리 또는 상태 타임아웃)");
    return return_type::ERROR;
  }
  fw_error_.store(latest_.error);
  fw_version_.store(latest_.fw_version);
  battery_v_.store(latest_.battery_v);
  if (latest_.error != last_error_) {
    if (latest_.error) {
      RCLCPP_WARN(get_logger(), "OpenCR 오류 비트 0x%02X (SyncRead 누락 %d · 명령 타임아웃 %d · 물리 E-stop %d · 모터 HW %d · 역할 불일치 %d)",
        latest_.error, !!(latest_.error & opencr_proto::ERR_SYNCREAD),
        !!(latest_.error & opencr_proto::ERR_CMD_TIMEOUT), !!(latest_.error & opencr_proto::ERR_HW_ESTOP),
        !!(latest_.error & opencr_proto::ERR_MOTOR_HW), !!(latest_.error & opencr_proto::ERR_ROLE_MISMATCH));
    }
    last_error_ = latest_.error;
  }
  apply_state_to_interfaces();
  return return_type::OK;
}

bool OpenCRHardware::send_cmd(bool torque_on)
{
  bool wheel_sent = false, arm_sent = false, ok = true;
  for (const auto & link : links_) {
    if (!link->have_state) {continue;}
    const uint8_t role = link->state.role;
    if (role != opencr_proto::ROLE_WHEEL && role != opencr_proto::ROLE_ARM) {continue;}
    const bool sent = send_cmd_for_role(role, torque_on);
    ok = sent && ok;
    if (role == opencr_proto::ROLE_WHEEL) {wheel_sent = true;} else {arm_sent = true;}
  }
  return ok && wheel_sent && arm_sent;
}

bool OpenCRHardware::send_cmd_for_role(uint8_t role, bool torque_on)
{
  opencr_proto::Cmd c{};
  c.role = role;
  for (const auto & j : joints_) {
    if (role == opencr_proto::ROLE_WHEEL && j.role == Role::WHEEL) {
      const double v = get_command<double>(j.name + "/" + hi::HW_IF_VELOCITY);
      c.wheel_vel[j.slot] = std::isfinite(v) ? static_cast<float>(v) : 0.0f;
    } else if (role == opencr_proto::ROLE_ARM && j.role == Role::ARM) {
      const double p = get_command<double>(j.name + "/" + hi::HW_IF_POSITION);
      c.arm_pos[j.slot] = static_cast<float>(std::isfinite(p) ? p : latest_.pos[opencr_proto::MAX_WHEELS + j.slot]);
    } else if (role == opencr_proto::ROLE_ARM && j.role == Role::GRIPPER) {
      const double g = get_command<double>(j.name + "/" + hi::HW_IF_POSITION);
      if (std::isfinite(g)) {c.gripper_pos = static_cast<float>(g); c.flags |= opencr_proto::FLAG_GRIPPER_VALID;}
    }
  }
  if (torque_on) {c.flags |= opencr_proto::FLAG_TORQUE_ON;}
  if (estop_.load() || !link_ok_.load()) {
    c.flags |= opencr_proto::FLAG_ESTOP;
    for (float & w : c.wheel_vel) {w = 0.0f;}
  }
  c.seq = seq_++;
  uint8_t frame[opencr_proto::MAX_FRAME];
  const size_t n = opencr_proto::encode_cmd(c, frame);
  for (const auto & link : links_) {
    if (link->have_state && link->state.role == role) {return link->serial.write_all(frame, n);}
  }
  return false;
}

return_type OpenCRHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!send_cmd(true)) {
    RCLCPP_ERROR(get_logger(), "CMD 송신 실패");
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace opencr_hardware

PLUGINLIB_EXPORT_CLASS(opencr_hardware::OpenCRHardware, hardware_interface::SystemInterface)
