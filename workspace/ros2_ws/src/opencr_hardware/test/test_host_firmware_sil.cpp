// 통합 시험: 실제 호스트 (OpenCRHardware 를 ResourceManager 로 로드) ↔ 가상 시리얼(pty) ↔ 실제 펌웨어 (SIL 바퀴 · 팔).
//   호스트 · 펌웨어 = 실제 코드. Dynamixel 버스 · 시계 · USB 배선 = 테스트 더블 (sil/sil_board.hpp, pty).
//   펌웨어 static 상태는 프로세스에 하나뿐이라 두 보드를 한 번 부팅하고 단계를 이어서 시험한다.
//   조이스틱 · /joy 는 없다 (joy_required=false). 리셋 버튼 경로 대신 auto_rearm 단계로 REARM 을 확인한다.
#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

#include "config.h"
#include "hardware_interface/resource_manager.hpp"
#include "opencr_hardware/opencr_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sil_board.hpp"

using namespace std::chrono_literals;
using namespace opencr_proto;

// pty 반대편에서 SIL 보드 한 대를 실시간으로 돌린다. 받은 CMD 의 flags 를 기록 (관찰용)
class SilLink
{
public:
  explicit SilLink(SilBoard & board)
  : b_(board)
  {
    master_ = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(master_);
    unlockpt(master_);
    path_ = ptsname(master_);
    termios t{};
    tcgetattr(master_, &t);
    cfmakeraw(&t);
    tcsetattr(master_, TCSANOW, &t);
    fcntl(master_, F_SETFL, O_NONBLOCK);
    b_.setup();
    th_ = std::thread([this]() {loop();});
  }
  ~SilLink()
  {
    run_ = false;
    th_.join();
    close(master_);
  }
  std::string path() const {return path_;}
  void set_connected(bool c) {connected_ = c;}   // false: 케이블이 빠진 것처럼 양방향 바이트를 버림
  int cmd_count() const {return cmds_.load();}
  int torque_off_cmd_count() const {return torque_off_cmds_.load();}   // TORQUE_ON 비트가 없는 CMD
  uint8_t seen_flags() const {return seen_flags_.load();}
  uint8_t last_flags() const {return last_flags_.load();}
  template<class F>
  auto with_board(F f)
  {
    std::lock_guard<std::mutex> lk(m_);
    return f(b_);
  }

private:
  void loop()
  {
    const auto t0 = std::chrono::steady_clock::now();
    FrameParser p;
    while (run_) {
      uint8_t buf[512];
      const ssize_t n = read(master_, buf, sizeof(buf));
      {
        std::lock_guard<std::mutex> lk(m_);
        if (n > 0 && connected_) {
          b_.host_rx.insert(b_.host_rx.end(), buf, buf + n);
          for (ssize_t i = 0; i < n; ++i) {
            if (p.push(buf[i]) && p.type() == TYPE_CMD) {
              Cmd c;
              if (decode_cmd(p.payload(), p.length(), c)) {
                ++cmds_;
                if (!(c.flags & FLAG_TORQUE_ON)) {++torque_off_cmds_;}
                seen_flags_ |= c.flags;
                last_flags_ = c.flags;
              }
            }
          }
        }
        const uint64_t us = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
        if (us > b_.now_us) {b_.now_us = us;}   // 대역이 막힘 시간을 더했으면 그만큼 앞서 있음
        b_.loop();
        if (!b_.host_tx.empty()) {
          if (connected_ && write(master_, b_.host_tx.data(), b_.host_tx.size()) < 0) {}
          b_.host_tx.clear();
        }
      }
      std::this_thread::sleep_for(200us);
    }
  }

  SilBoard & b_;
  int master_ = -1;
  std::string path_;
  std::thread th_;
  std::mutex m_;
  std::atomic<bool> run_{true}, connected_{true};
  std::atomic<int> cmds_{0}, torque_off_cmds_{0};
  std::atomic<uint8_t> seen_flags_{0}, last_flags_{0};
};

static std::string make_urdf(const std::string & ports, const std::string & extra_params)
{
  std::string joints, links = "<link name=\"base_link\"/>", urdf_joints;
  const char * wheels[6] = {"wheel_left_front_joint", "wheel_left_mid_joint", "wheel_left_rear_joint",
    "wheel_right_front_joint", "wheel_right_mid_joint", "wheel_right_rear_joint"};
  const char * arm[6] = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"};
  for (int i = 0; i < 6; ++i) {
    joints += std::string("<joint name=\"") + wheels[i] + "\"><param name=\"role\">wheel</param><param name=\"slot\">" +
      std::to_string(i) + "</param><command_interface name=\"velocity\"/><state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/><state_interface name=\"effort\"/></joint>";
    links += std::string("<link name=\"") + wheels[i] + "_l\"/>";
    urdf_joints += std::string("<joint name=\"") + wheels[i] + "\" type=\"continuous\"><parent link=\"base_link\"/>"
      "<child link=\"" + wheels[i] + "_l\"/><axis xyz=\"0 1 0\"/></joint>";
  }
  for (int i = 0; i < 6; ++i) {
    joints += std::string("<joint name=\"") + arm[i] + "\"><param name=\"role\">arm</param><param name=\"slot\">" +
      std::to_string(i) + "</param><command_interface name=\"position\"/><state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/><state_interface name=\"effort\"/></joint>";
    links += std::string("<link name=\"") + arm[i] + "_l\"/>";
    urdf_joints += std::string("<joint name=\"") + arm[i] + "\" type=\"revolute\"><parent link=\"base_link\"/>"
      "<child link=\"" + arm[i] + "_l\"/><axis xyz=\"0 0 1\"/><limit lower=\"-3\" upper=\"3\" effort=\"1\" velocity=\"5\"/></joint>";
  }
  return "<?xml version=\"1.0\"?><robot name=\"t\">" + links + urdf_joints +
         "<ros2_control name=\"OpenCRSystem\" type=\"system\"><hardware><plugin>opencr_hardware/OpenCRHardware</plugin>"
         "<param name=\"ports\">" + ports + "</param><param name=\"comm_timeout_s\">0.5</param>"
         "<param name=\"joy_required\">false</param>" + extra_params + "</hardware>" + joints +
         "</ros2_control></robot>";
}

static int32_t arm_tick(float rad) {return TICK_CENTER + static_cast<int32_t>(lroundf(rad / RAD_PER_TICK));}

struct Host
{
  explicit Host(const std::string & urdf)
  : clock(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
    rm(urdf, clock, rclcpp::get_logger("rm"), true, 100) {}
  std::string state()
  {
    const auto st = rm.get_components_status();
    return st.count("OpenCRSystem") ? st.at("OpenCRSystem").state.label() : "";
  }
  // 100 Hz read/write. 반환: read 가 처음 ERROR 를 낸 주기 (없으면 -1)
  int cycle(int n)
  {
    int first_error = -1;
    for (int i = 0; i < n; ++i) {
      const rclcpp::Time t = clock->now();
      if (rm.read(t, rclcpp::Duration::from_seconds(0.01)).result != hardware_interface::return_type::OK && first_error < 0) {
        first_error = i;
      }
      rm.write(t, rclcpp::Duration::from_seconds(0.01));
      std::this_thread::sleep_for(10ms);
    }
    return first_error;
  }
  std::shared_ptr<rclcpp::Clock> clock;
  hardware_interface::ResourceManager rm;
};

TEST(HostFirmwareSil, PhasesOnOneBoot)
{
  SilBoard & wb = sil_wheel_board();
  SilBoard & ab = sil_arm_board();
  for (uint8_t id = 1; id <= 6; ++id) {wb.motors[id] = SilMotor{}; wb.motors[id].model = WHEEL_MODEL;}
  for (uint8_t id : ARM_IDS) {ab.motors[id] = SilMotor{}; ab.motors[id].model = ARM_MODEL;}
  SilLink wheel(wb), arm(ab);
  const std::string ports = wheel.path() + "," + arm.path();
  auto torque_on = [](SilLink & l, uint8_t id) {return l.with_board([id](SilBoard & b) {return b.motors[id].torque;});};
  auto goal = [](SilLink & l, uint8_t id) {return l.with_board([id](SilBoard & b) {return b.motors[id].goal_position;});};

  // A. listen_only: 실제 펌웨어 두 보드의 STATE 로 활성화되지만 CMD 는 한 바이트도 가지 않음
  {
    Host h(make_urdf(ports, "<param name=\"listen_only\">true</param>"));
    ASSERT_EQ(h.state(), "active");
    EXPECT_EQ(h.cycle(50), -1);
    EXPECT_EQ(wheel.cmd_count(), 0);
    EXPECT_EQ(arm.cmd_count(), 0);
    EXPECT_FALSE(torque_on(arm, 11));
    EXPECT_FALSE(torque_on(wheel, 1));
  }

  // B. 일반 모드, auto_rearm 없음: 토크는 켜지지만 팔은 HOLD — REARM 없이 새 목표로 움직이지 않음
  {
    Host h(make_urdf(ports, ""));
    ASSERT_EQ(h.state(), "active");
    h.cycle(50);
    EXPECT_TRUE(torque_on(arm, 11));
    EXPECT_EQ(goal(arm, 11), TICK_CENTER) << "토크 ON 목표 = 현재 위치";
    {
      auto j1 = h.rm.claim_command_interface("joint1/position");
      ASSERT_TRUE(j1.set_value(0.5));
      h.cycle(100);
    }
    EXPECT_EQ(goal(arm, 11), TICK_CENTER) << "운용자 REARM 없이는 팔이 새 목표로 가지 않음";
    EXPECT_FALSE(arm.seen_flags() & FLAG_REARM) << "joy_required=false · auto_rearm=false 이면 REARM 을 보내지 않음";
  }

  // C. 호스트 재시작 (토크가 켜진 팔) + auto_rearm (벤치 전용): 토크를 끄지 않고, 잠금이 풀리면 REARM 펄스 → 명령을 따름
  {
    const size_t writes_before = arm.with_board([](SilBoard & b) {return b.writes.size();});
    const int off_before = arm.torque_off_cmd_count();
    Host h(make_urdf(ports, "<param name=\"auto_rearm\">true</param>"));
    ASSERT_EQ(h.state(), "active");
    h.cycle(10);   // 펌웨어 스레드가 활성화 뒤 CMD 를 처리할 시간
    EXPECT_EQ(arm.torque_off_cmd_count(), off_before) << "D5-2: 호스트 재활성화가 TORQUE_ON 없는 CMD 를 보내지 않음";
    const bool torque_went_off = arm.with_board([writes_before](SilBoard & b) {
        for (size_t i = writes_before; i < b.writes.size(); ++i) {
          if (b.writes[i].item == sil_item::TORQUE_ENABLE && b.writes[i].value == 0) {return true;}
        }
        return false;
      });
    EXPECT_FALSE(torque_went_off) << "호스트를 다시 시작해도 팔 토크 유지";
    EXPECT_TRUE(torque_on(arm, 11));
    h.cycle(50);
    EXPECT_TRUE(arm.seen_flags() & FLAG_REARM);
    {
      auto j1 = h.rm.claim_command_interface("joint1/position");
      ASSERT_TRUE(j1.set_value(0.5));
      h.cycle(50);
      EXPECT_EQ(goal(arm, 11), arm_tick(0.5f)) << "REARM 뒤 명령을 따름";

      // D. 팔 링크만 끊김: 호스트는 read ERROR + 살아 있는 바퀴 보드에 ESTOP, 팔 펌웨어는 마지막 목표로 HOLD
      arm.set_connected(false);
      ASSERT_TRUE(j1.set_value(-0.5));
      const int err_at = h.cycle(100);
      ASSERT_GE(err_at, 0) << "팔 STATE 가 끊기면 read ERROR";
      EXPECT_GE(err_at, 30) << "comm_timeout 0.5 s 전에는 ERROR 아님 (주기 수, 10 ms + 처리 시간)";
      EXPECT_TRUE(wheel.last_flags() & FLAG_ESTOP) << "끊기지 않은 보드에 ESTOP";
      EXPECT_EQ(goal(arm, 11), arm_tick(0.5f)) << "끊긴 팔은 마지막 승인 목표 유지";
      for (uint8_t id = 1; id <= 6; ++id) {
        EXPECT_EQ(wheel.with_board([id](SilBoard & b) {return b.motors[id].goal_velocity;}), 0) << int(id);
      }
      // 링크 복구 뒤 ros2_control 이 컴포넌트를 어떻게 다루는지는 버전마다 다를 수 있어 관찰만 한다
      // (auto_rearm 은 설계상 복구 뒤 자동 REARM — REARM 없이 움직이지 않는지는 B 단계가 확인)
      arm.set_connected(true);
      h.cycle(50);
      std::printf("[관찰] 링크 복구 뒤 컴포넌트 상태 = %s, 팔 목표 tick = %d (0.5 rad = %d, -0.5 rad = %d)\n",
        h.state().c_str(), goal(arm, 11), arm_tick(0.5f), arm_tick(-0.5f));
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
