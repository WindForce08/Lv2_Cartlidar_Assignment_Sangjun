// 로봇 설명 시험 — urdf/robot.urdf.xacro (CAD cartridar_01.urdf + ros2_control) 를 xacro 로 펼쳐 ros2_control 의 실제 파서에 넣고,
//   관절 이름 · 칸 · 컨트롤러 설정 (config/controllers_4wd.yaml) 이 서로 맞는지 본다. xacro 가 없는 PC 에서는 건너뛴다.
//   hardware_interface 6.x 는 URDF 에 없는 ros2_control 관절을 거부하므로 (Joint '...' not found in URDF), 이름이 어긋나면 여기서 잡힌다.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// $(find opencr_hardware) 를 소스 폴더로 바꾼 사본을 xacro 로 펼친다 (설치 · ament 색인 없이)
bool expand_xacro(std::string & urdf)
{
  if (std::system("command -v xacro > /dev/null 2>&1") != 0) {return false;}
  std::string text = read_file(PKG_DIR "/urdf/robot.urdf.xacro");
  text = std::regex_replace(text, std::regex(R"(\$\(find opencr_hardware\))"), PKG_DIR);
  char tmp[] = "/tmp/robot_description_XXXXXX";
  const int fd = mkstemp(tmp);
  if (fd < 0) {return false;}
  close(fd);
  std::ofstream(tmp) << text;
  FILE * p = popen((std::string("xacro ") + tmp + " 2>&1").c_str(), "r");
  char buf[4096];
  size_t n;
  urdf.clear();
  while ((n = fread(buf, 1, sizeof(buf), p)) > 0) {urdf.append(buf, n);}
  const int rc = pclose(p);
  std::remove(tmp);
  return rc == 0;
}

// yaml 의 "key: [a, b, c]" 한 줄에서 이름 목록 (따옴표 무시)
std::vector<std::string> yaml_list(const std::string & yaml, const std::string & key)
{
  std::smatch m;
  if (!std::regex_search(yaml, m, std::regex(key + R"(:\s*\[([^\]]*)\])"))) {return {};}
  std::vector<std::string> out;
  const std::string body = m[1];
  const std::regex item(R"([A-Za-z0-9_]+)");
  for (auto it = std::sregex_iterator(body.begin(), body.end(), item); it != std::sregex_iterator(); ++it) {out.push_back(it->str());}
  return out;
}

}  // namespace

TEST(RobotDescription, ExpandsAndMatchesFirmwareSlotsAndControllers)
{
  std::string urdf;
  if (std::system("command -v xacro > /dev/null 2>&1") != 0) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  ASSERT_TRUE(expand_xacro(urdf)) << urdf;

  std::vector<hardware_interface::HardwareInfo> infos;
  ASSERT_NO_THROW(infos = hardware_interface::parse_control_resources_from_urdf(urdf));
  ASSERT_EQ(infos.size(), 1u);
  const auto & hw = infos[0];
  EXPECT_EQ(hw.hardware_plugin_name, "opencr_hardware/OpenCRHardware");

  // 칸 순서: 주행 펌웨어 WHEEL_IDS (왼앞 · 왼뒤 · 오른앞 · 오른뒤), 팔 펌웨어 ARM_IDS (joint1~4 · 손목 롤 · 그리퍼 모터)
  const std::map<std::string, std::pair<std::string, int>> expect = {
    {"wheel_fl_joint", {"wheel", 0}}, {"wheel_rl_joint", {"wheel", 1}}, {"wheel_fr_joint", {"wheel", 2}}, {"wheel_rr_joint", {"wheel", 3}},
    {"joint1", {"arm", 0}}, {"joint2", {"arm", 1}}, {"joint3", {"arm", 2}}, {"joint4", {"arm", 3}},
    {"wrist_roll_joint", {"arm", 4}}, {"gripper_motor_joint", {"arm", 5}}};
  std::set<std::string> names;
  for (const auto & j : hw.joints) {
    names.insert(j.name);
    const auto e = expect.find(j.name);
    ASSERT_NE(e, expect.end()) << "모르는 관절 " << j.name;
    EXPECT_EQ(j.parameters.at("role"), e->second.first) << j.name;
    EXPECT_EQ(std::stoi(j.parameters.at("slot")), e->second.second) << j.name;
  }
  EXPECT_EQ(names.size(), expect.size());

  // 팔 명령 자르기 한계 (command_interface min/max) ⊆ URDF <limit> ∩ 펌웨어 config.h ARM_MIN/MAX_RAD
  //   (펌웨어는 범위 밖 CMD 를 통째로 거부하므로 호스트가 넘겨 보내면 안 됨)
  const std::string cfg = read_file(PKG_DIR "/../opencr_firmware/config.h");
  const auto fw_range = [&cfg](const std::string & key) {
      std::smatch m;
      std::vector<double> out;
      if (!std::regex_search(cfg, m, std::regex(key + R"(\[ARM_COUNT\]\s*=\s*\{([^}]*)\})"))) {return out;}
      const std::string body = m[1];
      const std::regex num(R"(-?[0-9]+(\.[0-9]*)?)");
      for (auto it = std::sregex_iterator(body.begin(), body.end(), num); it != std::sregex_iterator(); ++it) {out.push_back(std::stod(it->str()));}
      return out;
    };
  const auto fw_min = fw_range("ARM_MIN_RAD"), fw_max = fw_range("ARM_MAX_RAD");
  ASSERT_EQ(fw_min.size(), 6u);
  ASSERT_EQ(fw_max.size(), 6u);
  for (const auto & j : hw.joints) {
    if (j.parameters.at("role") != "arm") {continue;}
    const int k = std::stoi(j.parameters.at("slot"));
    ASSERT_EQ(j.command_interfaces.size(), 1u) << j.name;
    const double lo = std::stod(j.command_interfaces[0].min), hi = std::stod(j.command_interfaces[0].max);
    std::smatch m;
    ASSERT_TRUE(std::regex_search(urdf, m, std::regex("<joint name=\"" + j.name + R"re(" type=[^>]*>[\s\S]*?<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)")re")))
      << j.name << " 의 URDF <limit> 없음";
    EXPECT_GE(lo, std::stod(m[1]) - 1e-6) << j.name << " 명령 하한이 URDF 보다 넓음";
    EXPECT_LE(hi, std::stod(m[2]) + 1e-6) << j.name << " 명령 상한이 URDF 보다 넓음";
    EXPECT_GE(lo, fw_min[k] - 1e-6) << j.name << " 명령 하한이 펌웨어 범위 밖";
    EXPECT_LE(hi, fw_max[k] + 1e-6) << j.name << " 명령 상한이 펌웨어 범위 밖";
  }
  ASSERT_EQ(hw.sensors.size(), 1u);
  EXPECT_EQ(hw.sensors[0].name, "imu");

  // 컨트롤러 설정의 관절은 모두 ros2_control 에 있어야 함
  const std::string yaml = read_file(PKG_DIR "/config/controllers_4wd.yaml");
  for (const std::string key : {"left_wheel_names", "right_wheel_names", "joints"}) {
    const auto list = yaml_list(yaml, key);
    EXPECT_FALSE(list.empty()) << key;
    for (const auto & n : list) {EXPECT_TRUE(names.count(n)) << key << ": " << n << " 가 ros2_control 에 없음";}
  }
  std::smatch gripper;
  ASSERT_TRUE(std::regex_search(yaml, gripper, std::regex(R"(gripper_controller:[\s\S]*?\bjoint:\s*(\w+))")));
  EXPECT_EQ(gripper[1].str(), "gripper_motor_joint");

  // 다른 노드가 쓰는 좌표 (Controller arm_base_link · Perception base_footprint · 센서) 가 URDF 에 있어야 함.
  //   D435 = realsense camera_name (top_camera_front/rear) 의 <camera_name>_link — 드라이버가 그 아래 optical frame 을 붙임
  for (const char * link : {"base_link", "base_footprint", "arm_base_link", "imu_link", "tfmini_link", "lds03_front_link",
      "lds03_rear_link", "top_camera_front_link", "top_camera_rear_link", "gripper_camera_optical_frame", "end_effector_link"})
  {
    EXPECT_NE(urdf.find(std::string("<link name=\"") + link + "\""), std::string::npos) << link;
  }
  // realsense 드라이버가 발행하는 좌표를 URDF 가 또 만들면 TF 부모가 둘이 됨
  EXPECT_EQ(urdf.find("_color_optical_frame\""), std::string::npos);
}
