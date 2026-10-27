// 로봇 설명 시험 (xacro 가 없는 PC 에서는 건너뛴다).
//   1) ros2_control 블록 (urdf/opencr.ros2_control.xacro) — 로봇 URDF 없이: 관절마다 빈 링크 · 넓은 한계의 가짜 관절을 만들어 펼친 뒤
//      ros2_control 의 실제 파서에 넣고, 칸 · 역할 · 명령 한계 (펌웨어 config.h 범위 안) · 컨트롤러 설정 (config/controllers_4wd.yaml) 을 본다.
//   2) 로봇 URDF (urdf/robot.urdf.xacro = 차체 URDF + 위 블록) — 있을 때만: 명령 한계가 URDF <limit> 안인지, 다른 노드가 쓰는 좌표가 있는지.
//      2026-10-27 1안 차체 (cartridar_01) 폐기로 로봇 URDF 를 지움 → 차체 2안 URDF 가 올 때까지 건너뜀.
//   hardware_interface 6.x 는 URDF 에 없는 ros2_control 관절을 거부하므로 (Joint '...' not found in URDF), 이름이 어긋나면 여기서 잡힌다.
#include <gtest/gtest.h>
#include <unistd.h>

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

bool have_xacro() {return std::system("command -v xacro > /dev/null 2>&1") == 0;}
bool exists(const std::string & path) {return std::ifstream(path).good();}

// xacro 글을 임시 파일로 써서 펼친다. $(find opencr_hardware) 는 소스 폴더로 (설치 · ament 색인 없이)
bool expand_xacro(std::string text, std::string & urdf)
{
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

// 펌웨어 config.h 의 "NAME[ARM_COUNT] = {..}" 숫자들
std::vector<double> fw_range(const std::string & key)
{
  const std::string cfg = read_file(PKG_DIR "/../opencr_firmware/config.h");
  std::smatch m;
  std::vector<double> out;
  if (!std::regex_search(cfg, m, std::regex(key + R"(\[ARM_COUNT\]\s*=\s*\{([^}]*)\})"))) {return out;}
  const std::string body = m[1];
  const std::regex num(R"(-?[0-9]+(\.[0-9]*)?)");
  for (auto it = std::sregex_iterator(body.begin(), body.end(), num); it != std::sregex_iterator(); ++it) {out.push_back(std::stod(it->str()));}
  return out;
}

hardware_interface::HardwareInfo parse_one(const std::string & urdf)
{
  std::vector<hardware_interface::HardwareInfo> infos;
  EXPECT_NO_THROW(infos = hardware_interface::parse_control_resources_from_urdf(urdf));
  EXPECT_EQ(infos.size(), 1u);
  return infos.empty() ? hardware_interface::HardwareInfo{} : infos[0];
}

}  // namespace

// 1) 로봇 URDF 없이 ros2_control 블록만
TEST(RobotDescription, Ros2ControlBlockMatchesFirmwareAndControllers)
{
  if (!have_xacro()) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  // 블록이 부르는 관절마다 가짜 관절 (빈 링크, 한계 ±10 - 명령 한계를 URDF 가 자르지 않게)
  const std::string block = read_file(PKG_DIR "/urdf/opencr.ros2_control.xacro");
  std::string body = "<link name=\"base_link\"/>";
  const std::regex call(R"re(<xacro:opencr_(wheel|arm_joint)\s+name="([A-Za-z0-9_]+)")re");
  int calls = 0;
  for (auto it = std::sregex_iterator(block.begin(), block.end(), call); it != std::sregex_iterator(); ++it, ++calls) {
    const std::string kind = (*it)[1], name = (*it)[2];
    body += "<link name=\"" + name + "_link\"/><joint name=\"" + name + "\" type=\"" + (kind == "wheel" ? "continuous" : "revolute") +
      "\"><parent link=\"base_link\"/><child link=\"" + name + "_link\"/><axis xyz=\"0 0 1\"/>" +
      (kind == "wheel" ? "" : "<limit lower=\"-10\" upper=\"10\" effort=\"10\" velocity=\"10\"/>") + "</joint>";
  }
  ASSERT_GT(calls, 0) << "opencr.ros2_control.xacro 에서 관절을 찾지 못함";
  const std::string wrapper = std::string("<?xml version=\"1.0\"?><robot name=\"t\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">") +
    "<xacro:include filename=\"$(find opencr_hardware)/urdf/opencr.ros2_control.xacro\"/>" + body +
    "<xacro:opencr_ros2_control/></robot>";
  std::string urdf;
  ASSERT_TRUE(expand_xacro(wrapper, urdf)) << urdf;
  const auto hw = parse_one(urdf);
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

  // 팔 명령 자르기 한계 (command_interface min/max) ⊆ 펌웨어 config.h ARM_MIN/MAX_RAD (펌웨어는 범위 밖 CMD 를 통째로 거부)
  const auto fw_min = fw_range("ARM_MIN_RAD"), fw_max = fw_range("ARM_MAX_RAD");
  ASSERT_EQ(fw_min.size(), 6u);
  ASSERT_EQ(fw_max.size(), 6u);
  for (const auto & j : hw.joints) {
    if (j.parameters.at("role") != "arm") {continue;}
    const int k = std::stoi(j.parameters.at("slot"));
    ASSERT_EQ(j.command_interfaces.size(), 1u) << j.name;
    EXPECT_GE(std::stod(j.command_interfaces[0].min), fw_min[k] - 1e-6) << j.name << " 명령 하한이 펌웨어 범위 밖";
    EXPECT_LE(std::stod(j.command_interfaces[0].max), fw_max[k] + 1e-6) << j.name << " 명령 상한이 펌웨어 범위 밖";
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
}

// 2) 로봇 URDF 가 있을 때: 명령 한계 ⊆ URDF <limit>, 다른 노드가 쓰는 좌표 (차체 2안 URDF 가 지킬 이름)
TEST(RobotDescription, RobotUrdfProvidesFramesWhenPresent)
{
  if (!have_xacro()) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  if (!exists(PKG_DIR "/urdf/robot.urdf.xacro")) {
    GTEST_SKIP() << "로봇 URDF 없음 (urdf/robot.urdf.xacro) — 1안 차체 폐기 (2026-10-27), 차체 2안 URDF 대기";
  }
  std::string urdf;
  ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf)) << urdf;
  const auto hw = parse_one(urdf);
  for (const auto & j : hw.joints) {
    if (j.parameters.at("role") != "arm") {continue;}
    std::smatch m;
    ASSERT_TRUE(std::regex_search(urdf, m, std::regex("<joint name=\"" + j.name + R"re(" type=[^>]*>[\s\S]*?<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)")re")))
      << j.name << " 의 URDF <limit> 없음";
    EXPECT_GE(std::stod(j.command_interfaces[0].min), std::stod(m[1]) - 1e-6) << j.name << " 명령 하한이 URDF 보다 넓음";
    EXPECT_LE(std::stod(j.command_interfaces[0].max), std::stod(m[2]) + 1e-6) << j.name << " 명령 상한이 URDF 보다 넓음";
  }
  // 다른 노드가 쓰는 좌표 (Controller arm_base_link · Perception base_footprint · 센서).
  //   D435 = realsense camera_name (top_camera_front/rear) 의 <camera_name>_link — 드라이버가 그 아래 optical frame 을 붙임
  for (const char * link : {"base_link", "base_footprint", "arm_base_link", "imu_link", "tfmini_link", "lds03_front_link",
      "lds03_rear_link", "top_camera_front_link", "top_camera_rear_link", "gripper_camera_optical_frame", "end_effector_link"})
  {
    EXPECT_NE(urdf.find(std::string("<link name=\"") + link + "\""), std::string::npos) << link;
  }
  // realsense 드라이버가 발행하는 좌표를 URDF 가 또 만들면 TF 부모가 둘이 됨
  EXPECT_EQ(urdf.find("_color_optical_frame\""), std::string::npos);
}
