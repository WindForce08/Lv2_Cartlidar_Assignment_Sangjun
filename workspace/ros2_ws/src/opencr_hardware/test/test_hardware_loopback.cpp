// OpenCRHardware 루프백 시험: 가상 시리얼(pty) 반대편에서 펌웨어를 흉내 낸다.
//   ResourceManager 가 URDF 로 플러그인을 pluginlib 로 불러 configure · activate → read/write 100 Hz.
//   확인: 활성화 때 팔 명령 = 현재 위치, 명령 → CMD 패킷, STATE → 상태 인터페이스, IMU, 통신 끊김 → ERROR
#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "hardware_interface/resource_manager.hpp"
#include "opencr_hardware/opencr_protocol.hpp"
#include "opencr_hardware/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace opencr_proto;

class FakeOpenCR
{
public:
  explicit FakeOpenCR(uint8_t role, uint8_t version = PROTOCOL_VERSION)
  : role_(role), version_(version)
  {
    master_ = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(master_);
    unlockpt(master_);
    slave_path_ = ptsname(master_);
    termios t{};
    tcgetattr(master_, &t);
    cfmakeraw(&t);
    tcsetattr(master_, TCSANOW, &t);
    fcntl(master_, F_SETFL, O_NONBLOCK);
    for (int i = 0; i < ARM_JOINTS; ++i) {arm_now_[i] = 0.3f + 0.1f * i;}   // 켜졌을 때 팔 위치
    th_ = std::thread([this]() {loop();});
  }
  ~FakeOpenCR()
  {
    run_ = false;
    th_.join();
    close(master_);
  }
  std::string path() const {return slave_path_;}
  void set_silent(bool s) {silent_ = s;}
  void set_error_bits(uint8_t e) {error_bits_ = e;}   // STATE error 에 항상 OR (예: 모터 없는 보드)
  Cmd last_cmd()
  {
    std::lock_guard<std::mutex> lk(m_);
    return last_;
  }
  int cmd_count() const {return cmds_.load();}
  int cmd_count_torque_on() const {return torque_cmds_.load();}

private:
  void loop()
  {
    FrameParser p;
    uint8_t buf[512];
    auto next = std::chrono::steady_clock::now();
    while (run_) {
      const ssize_t n = read(master_, buf, sizeof(buf));
      for (ssize_t i = 0; i < n; ++i) {
        if (p.push(buf[i]) && p.type() == TYPE_CMD) {
          Cmd c;
          if (decode_cmd(p.payload(), p.length(), c)) {
            std::lock_guard<std::mutex> lk(m_);
            last_ = c;
            ++cmds_;
            if (c.role != role_) {continue;}
            last_own_cmd_ = std::chrono::steady_clock::now();
            if (c.flags & FLAG_TORQUE_ON) {++torque_cmds_;}
            if (c.flags & FLAG_TORQUE_ON) {
              for (int k = 0; k < ARM_JOINTS; ++k) {arm_now_[k] = c.arm_pos[k];}   // 이상적인 위치 추종
              for (int k = 0; k < MAX_WHEELS; ++k) {wheel_vel_[k] = (c.flags & FLAG_ESTOP) ? 0.f : c.wheel_vel[k];}
            }
          }
        }
      }
      if (std::chrono::steady_clock::now() >= next) {
        next += 10ms;
        if (!silent_) {
          State s{};
          s.role = role_;
          {
            std::lock_guard<std::mutex> lk(m_);
            for (int k = 0; k < MAX_WHEELS; ++k) {s.vel[k] = wheel_vel_[k];}
            for (int k = 0; k < ARM_JOINTS; ++k) {s.pos[MAX_WHEELS + k] = arm_now_[k];}
          }
          s.quat[0] = 1.f;
          s.accel[2] = 9.8f;
          // 펌웨어 계약: 자기 역할 CMD 가 200 ms 없으면 ERR_CMD_TIMEOUT (호스트는 활성화 전에 이를 기다림)
          if (std::chrono::steady_clock::now() - last_own_cmd_ > 200ms) {s.error = ERR_CMD_TIMEOUT;}
          s.error |= error_bits_;
          s.fw_version = version_;
          s.seq = seq_++;
          uint8_t f[MAX_FRAME];
          const size_t len = encode_state(s, f);
          if (write(master_, f, len) < 0) {}
        }
      }
      std::this_thread::sleep_for(1ms);
    }
  }

  int master_ = -1;
  std::string slave_path_;
  std::thread th_;
  std::atomic<bool> run_{true}, silent_{false};
  std::atomic<uint8_t> error_bits_{0};
  std::mutex m_;
  Cmd last_{};
  std::atomic<int> cmds_{0};
  std::atomic<int> torque_cmds_{0};
  float arm_now_[ARM_JOINTS]{};
  float wheel_vel_[MAX_WHEELS]{};
  std::chrono::steady_clock::time_point last_own_cmd_{};
  uint8_t seq_ = 0;
  uint8_t role_;
  uint8_t version_;
};

// joint1_limits: joint1 command_interface 안에 넣을 <param name="min"/max> (빈 값이면 한계 없음)
static std::string make_urdf(const std::string & ports, const std::string & extra_params = "",
  const std::string & joint1_limits = "")
{
  std::string joints;
  const char * wheels[6] = {"wheel_left_front_joint", "wheel_left_mid_joint", "wheel_left_rear_joint",
    "wheel_right_front_joint", "wheel_right_mid_joint", "wheel_right_rear_joint"};
  for (int i = 0; i < 6; ++i) {
    joints += std::string("<joint name=\"") + wheels[i] + "\"><param name=\"role\">wheel</param><param name=\"slot\">" +
      std::to_string(i) + "</param><command_interface name=\"velocity\"/><state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/><state_interface name=\"effort\"/></joint>";
  }
  const char * arm[6] = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"};
  for (int i = 0; i < 6; ++i) {
    const std::string cmd_if = (i == 0 && !joint1_limits.empty()) ?
      "<command_interface name=\"position\">" + joint1_limits + "</command_interface>" : "<command_interface name=\"position\"/>";
    joints += std::string("<joint name=\"") + arm[i] + "\"><param name=\"role\">arm</param><param name=\"slot\">" +
      std::to_string(i) + "</param>" + cmd_if + "<state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/><state_interface name=\"effort\"/></joint>";
  }
  std::string imu = "<sensor name=\"imu\">";
  for (const char * s : {"orientation.x", "orientation.y", "orientation.z", "orientation.w", "angular_velocity.x",
      "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x", "linear_acceleration.y",
      "linear_acceleration.z"})
  {
    imu += std::string("<state_interface name=\"") + s + "\"/>";
  }
  imu += "</sensor>";
  std::string links = "<link name=\"base_link\"/>";
  std::string urdf_joints;
  for (const char * j : wheels) {
    links += std::string("<link name=\"") + j + "_l\"/>";
    urdf_joints += std::string("<joint name=\"") + j + "\" type=\"continuous\"><parent link=\"base_link\"/><child link=\"" +
      j + "_l\"/><axis xyz=\"0 1 0\"/></joint>";
  }
  for (const char * j : arm) {
    links += std::string("<link name=\"") + j + "_l\"/>";
    urdf_joints += std::string("<joint name=\"") + j + "\" type=\"revolute\"><parent link=\"base_link\"/><child link=\"" +
      j + "_l\"/><axis xyz=\"0 0 1\"/><limit lower=\"-3\" upper=\"3\" effort=\"1\" velocity=\"5\"/></joint>";
  }
  return "<?xml version=\"1.0\"?><robot name=\"t\">" + links + urdf_joints +
         "<ros2_control name=\"OpenCRSystem\" type=\"system\"><hardware><plugin>opencr_hardware/OpenCRHardware</plugin>"
         "<param name=\"ports\">" + ports + "</param><param name=\"comm_timeout_s\">0.5</param>"
         "<param name=\"joy_required\">false</param>" + extra_params + "</hardware>" + joints + imu +
         "</ros2_control></robot>";
}

TEST(OpenCRHardware, LoopbackThroughResourceManager)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
  const std::string ports = wheel.path() + "," + arm.path();
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(ports), clock, rclcpp::get_logger("rm"), true, 100);

  rclcpp::Time t = clock->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);
  auto cycle = [&](int n) {
      for (int i = 0; i < n; ++i) {
        EXPECT_EQ(rm.read(t, dt).result, hardware_interface::return_type::OK);
        EXPECT_EQ(rm.write(t, dt).result, hardware_interface::return_type::OK);
        std::this_thread::sleep_for(10ms);
        t = clock->now();
      }
    };
  cycle(5);
  // 활성화 직후 팔 명령 = 켜졌을 때 위치 (튀지 않음)
  auto c0 = wheel.last_cmd();
  auto a0 = arm.last_cmd();
  EXPECT_TRUE(c0.flags & FLAG_TORQUE_ON);
  EXPECT_TRUE(a0.flags & FLAG_TORQUE_ON);
  EXPECT_EQ(c0.role, ROLE_WHEEL);
  EXPECT_EQ(a0.role, ROLE_ARM);
  EXPECT_NEAR(a0.arm_pos[0], 0.3, 1e-5);
  EXPECT_NEAR(a0.arm_pos[5], 0.8, 1e-5);
  EXPECT_FLOAT_EQ(c0.wheel_vel[0], 0.f);

  {
    auto j1 = rm.claim_command_interface("joint1/position");
    auto wl = rm.claim_command_interface("wheel_left_front_joint/velocity");
    auto wr = rm.claim_command_interface("wheel_right_front_joint/velocity");
    auto wr5 = rm.claim_command_interface("wheel_right_rear_joint/velocity");
    ASSERT_TRUE(j1.set_value(-0.25));
    ASSERT_TRUE(wl.set_value(2.0));
    ASSERT_TRUE(wr.set_value(-1.5));
    ASSERT_TRUE(wr5.set_value(0.75));
    cycle(10);
  }
  const auto c = wheel.last_cmd();
  const auto a = arm.last_cmd();
  EXPECT_NEAR(a.arm_pos[0], -0.25, 1e-5);
  EXPECT_NEAR(c.wheel_vel[0], 2.0, 1e-5);
  EXPECT_NEAR(c.wheel_vel[3], -1.5, 1e-5);
  EXPECT_NEAR(c.wheel_vel[5], 0.75, 1e-5);
  EXPECT_FALSE(c.flags & FLAG_ESTOP);

  // STATE → 상태 인터페이스
  {
    auto p = rm.claim_state_interface("joint1/position");
    auto v = rm.claim_state_interface("wheel_left_front_joint/velocity");
    auto qw = rm.claim_state_interface("imu/orientation.w");
    auto az = rm.claim_state_interface("imu/linear_acceleration.z");
    EXPECT_NEAR(p.get_optional<double>().value(), -0.25, 1e-5);
    EXPECT_NEAR(v.get_optional<double>().value(), 2.0, 1e-5);
    EXPECT_NEAR(qw.get_optional<double>().value(), 1.0, 1e-6);
    EXPECT_NEAR(az.get_optional<double>().value(), 9.8, 1e-5);
  }
  EXPECT_GT(wheel.cmd_count(), 10);
  EXPECT_GT(arm.cmd_count(), 10);

  // 통신 끊김 → 100 Hz 로 계속 읽는 동안 comm_timeout(0.5 s) 남짓 지나면 read ERROR
  wheel.set_silent(true);
  arm.set_silent(true);
  const auto t_silent = std::chrono::steady_clock::now();
  double err_after = -1.0;
  while (std::chrono::steady_clock::now() - t_silent < 1500ms) {
    if (rm.read(clock->now(), dt).result == hardware_interface::return_type::ERROR) {
      err_after = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_silent).count();
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(err_after, 0.45);
  EXPECT_LT(err_after, 0.65);
}

TEST(OpenCRHardware, RejectsFirmwareVersionMismatch)
{
  FakeOpenCR wheel(ROLE_WHEEL, static_cast<uint8_t>(PROTOCOL_VERSION + 1));
  FakeOpenCR arm(ROLE_ARM);
  const std::string ports = wheel.path() + "," + arm.path();
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(ports), clock, rclcpp::get_logger("rm"), true, 100);
  const auto st = rm.get_components_status();
  ASSERT_EQ(st.count("OpenCRSystem"), 1u);
  EXPECT_NE(st.at("OpenCRSystem").state.label(), "active");   // 버전이 다르면 활성화하지 않음
  EXPECT_EQ(wheel.cmd_count_torque_on(), 0);                  // 잘못된 버전 보드에는 토크 ON 없음
  EXPECT_EQ(arm.cmd_count_torque_on(), 0);
}

// 이 아래 FakeOpenCR 시험은 호스트 동작만 본다 — FakeOpenCR 은 이상화된 테스트 더블이고 실제 펌웨어가 아니다
static std::string component_state(hardware_interface::ResourceManager & rm)
{
  const auto st = rm.get_components_status();
  return st.count("OpenCRSystem") ? st.at("OpenCRSystem").state.label() : "";
}

TEST(OpenCRHardware, RejectsV4Firmware)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM, 4);   // v5 호스트 + v4 팔 펌웨어
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path()), clock, rclcpp::get_logger("rm"), true, 100);
  EXPECT_NE(component_state(rm), "active");
  EXPECT_EQ(wheel.cmd_count_torque_on(), 0);
  EXPECT_EQ(arm.cmd_count_torque_on(), 0);
}

TEST(OpenCRHardware, RejectsDuplicateRole)
{
  FakeOpenCR a(ROLE_WHEEL), b(ROLE_WHEEL);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(a.path() + "," + b.path()), clock, rclcpp::get_logger("rm"), true, 100);
  EXPECT_NE(component_state(rm), "active") << "바퀴 역할 두 개 = 팔 보드 없음";
  EXPECT_EQ(a.cmd_count_torque_on() + b.cmd_count_torque_on(), 0);
}

TEST(OpenCRHardware, RejectsMissingBoard)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
  arm.set_silent(true);   // 포트는 열리지만 STATE 가 없음
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path()), clock, rclcpp::get_logger("rm"), true, 100);
  EXPECT_NE(component_state(rm), "active");
  EXPECT_EQ(wheel.cmd_count_torque_on(), 0);
}

TEST(OpenCRHardware, PortOrderDoesNotMatter)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(arm.path() + "," + wheel.path()), clock, rclcpp::get_logger("rm"), true, 100);
  ASSERT_EQ(component_state(rm), "active");
  rclcpp::Time t = clock->now();
  for (int i = 0; i < 5; ++i) {
    rm.read(t, rclcpp::Duration::from_seconds(0.01));
    rm.write(t, rclcpp::Duration::from_seconds(0.01));
    std::this_thread::sleep_for(10ms);
    t = clock->now();
  }
  EXPECT_EQ(wheel.last_cmd().role, ROLE_WHEEL);
  EXPECT_EQ(arm.last_cmd().role, ROLE_ARM);
  EXPECT_NEAR(arm.last_cmd().arm_pos[0], 0.3, 1e-5);
}

// 통신 확인 전용: 활성화 · read/write 를 돌려도 어떤 CMD 도 (토크 OFF · ESTOP 도) 보내지 않음
TEST(OpenCRHardware, ListenOnlySendsNoCommand)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path(), "<param name=\"listen_only\">true</param>"),
    clock, rclcpp::get_logger("rm"), true, 100);
  ASSERT_EQ(component_state(rm), "active") << "STATE 만으로 역할 · 버전 확인";
  rclcpp::Time t = clock->now();
  for (int i = 0; i < 30; ++i) {
    EXPECT_EQ(rm.read(t, rclcpp::Duration::from_seconds(0.01)).result, hardware_interface::return_type::OK);
    EXPECT_EQ(rm.write(t, rclcpp::Duration::from_seconds(0.01)).result, hardware_interface::return_type::OK);
    std::this_thread::sleep_for(10ms);
    t = clock->now();
  }
  EXPECT_EQ(wheel.cmd_count(), 0);
  EXPECT_EQ(arm.cmd_count(), 0);
}

// D5-2: 활성화 중에는 CMD (토크 OFF 포함) 를 보내지 않음 — 첫 CMD 는 write() 의 TORQUE_ON
TEST(OpenCRHardware, ActivationSendsNoCommand)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path()), clock, rclcpp::get_logger("rm"), true, 100);
  ASSERT_EQ(component_state(rm), "active");
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(wheel.cmd_count(), 0);
  EXPECT_EQ(arm.cmd_count(), 0);
  rm.write(clock->now(), rclcpp::Duration::from_seconds(0.01));
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(arm.cmd_count(), 1);
  EXPECT_TRUE(arm.last_cmd().flags & FLAG_TORQUE_ON);
}

// 리뷰 R2: 팔이 명령 한계 밖에 있어도 한계로 잘라 보냄 (펌웨어는 범위 밖 CMD 를 통째로 거부)
TEST(OpenCRHardware, ArmCommandClampedToUrdfLimits)
{
  FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);   // FakeOpenCR 팔 초기 위치: joint1 0.3, joint2 0.4
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path(), "",
    "<param name=\"min\">-0.2</param><param name=\"max\">0.2</param>"), clock, rclcpp::get_logger("rm"), true, 100);
  ASSERT_EQ(component_state(rm), "active");
  rclcpp::Time t = clock->now();
  for (int i = 0; i < 5; ++i) {
    rm.read(t, rclcpp::Duration::from_seconds(0.01));
    rm.write(t, rclcpp::Duration::from_seconds(0.01));
    std::this_thread::sleep_for(10ms);
    t = clock->now();
  }
  EXPECT_NEAR(arm.last_cmd().arm_pos[0], 0.2, 1e-6) << "현재 위치 0.3 이 한계 밖 → 0.2 로 잘라 보냄";
  EXPECT_NEAR(arm.last_cmd().arm_pos[1], 0.4, 1e-5) << "한계가 없는 관절은 그대로";
}

// 모터 없이 OpenCR 만 연결 (부팅 점검 실패 = MOTOR_CONFIG · FAULT): 일반 모드는 거부, listen_only 는 STATE 확인용으로 활성화
TEST(OpenCRHardware, ListenOnlyActivatesWithoutMotors)
{
  for (const bool listen : {false, true}) {
    FakeOpenCR wheel(ROLE_WHEEL), arm(ROLE_ARM);
    wheel.set_error_bits(ERR_MOTOR_CONFIG | ERR_FAULT);
    arm.set_error_bits(ERR_MOTOR_CONFIG | ERR_FAULT);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    hardware_interface::ResourceManager rm(make_urdf(wheel.path() + "," + arm.path(),
      listen ? "<param name=\"listen_only\">true</param>" : ""), clock, rclcpp::get_logger("rm"), true, 100);
    if (listen) {
      ASSERT_EQ(component_state(rm), "active");
      rclcpp::Time t = clock->now();
      for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(rm.read(t, rclcpp::Duration::from_seconds(0.01)).result, hardware_interface::return_type::OK);
        rm.write(t, rclcpp::Duration::from_seconds(0.01));
        std::this_thread::sleep_for(10ms);
        t = clock->now();
      }
      EXPECT_EQ(wheel.cmd_count() + arm.cmd_count(), 0);
    } else {
      EXPECT_NE(component_state(rm), "active");
      EXPECT_EQ(wheel.cmd_count_torque_on() + arm.cmd_count_torque_on(), 0);
    }
  }
}

TEST(SerialPort, ExclusiveOpen)
{
  FakeOpenCR fw(ROLE_WHEEL);
  opencr_hardware::SerialPort a, b;
  std::string err;
  ASSERT_TRUE(a.open(fw.path(), B115200, err)) << err;
  EXPECT_FALSE(b.open(fw.path(), B115200, err));   // 두 번째 프로세스(같은 장치)는 거부
  a.close();
  EXPECT_TRUE(b.open(fw.path(), B115200, err)) << err;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
