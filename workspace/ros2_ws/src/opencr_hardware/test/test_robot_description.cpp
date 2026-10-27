// 로봇 설명 시험 (2026-10-27 arm_bench 이식, xacro 가 없는 PC 에서는 건너뛴다).
//   urdf/robot.urdf.xacro (기본 인자: 4륜 · use_gripper false) 를 펼쳐 ros2_control 의 실제 파서에 넣고
//   1) 관절 이름 · 역할 · 칸이 펌웨어 칸 순서와 같은지, 팔 명령 한계가 펌웨어 config.h 범위와 URDF <limit> 안인지
//      (펌웨어는 범위 밖 CMD 를 통째로 거부한다), 컨트롤러 설정 (config/controllers_4wd.yaml) 의 관절이 있는지
//   2) 다른 노드가 쓰는 좌표가 있는지 본다.
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

TEST(RobotDescription, Ros2ControlMatchesFirmwareAndControllers)
{
  if (!have_xacro()) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  std::string urdf;
  ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf)) << urdf;
  const auto hw = parse_one(urdf);
  EXPECT_EQ(hw.hardware_plugin_name, "opencr_hardware/OpenCRHardware");

  // 칸 순서: 주행 펌웨어 WHEEL_IDS (왼앞 · 왼뒤 · 오른앞 · 오른뒤), 팔 펌웨어 ARM_IDS (joint1~4 · 손목 롤 · 6 번째 칸 = 그리퍼 개폐 모터)
  const std::map<std::string, std::pair<std::string, int>> expect = {
    {"wheel_left_front_joint", {"wheel", 0}}, {"wheel_left_rear_joint", {"wheel", 1}},
    {"wheel_right_front_joint", {"wheel", 2}}, {"wheel_right_rear_joint", {"wheel", 3}},
    {"joint1", {"arm", 0}}, {"joint2", {"arm", 1}}, {"joint3", {"arm", 2}}, {"joint4", {"arm", 3}},
    {"joint5", {"arm", 4}}, {"gripper_roll", {"arm", 5}}};
  std::set<std::string> names;
  for (const auto & j : hw.joints) {
    names.insert(j.name);
    const auto e = expect.find(j.name);
    ASSERT_NE(e, expect.end()) << "모르는 관절 " << j.name;
    EXPECT_EQ(j.parameters.at("role"), e->second.first) << j.name;
    EXPECT_EQ(std::stoi(j.parameters.at("slot")), e->second.second) << j.name;
  }
  EXPECT_EQ(names.size(), expect.size());

  // 팔 명령 자르기 한계 (command_interface min/max) ⊆ 펌웨어 config.h ARM_MIN/MAX_RAD ∩ URDF <limit>
  const auto fw_min = fw_range("ARM_MIN_RAD"), fw_max = fw_range("ARM_MAX_RAD");
  ASSERT_EQ(fw_min.size(), 6u);
  ASSERT_EQ(fw_max.size(), 6u);
  for (const auto & j : hw.joints) {
    if (j.parameters.at("role") != "arm") {continue;}
    const int k = std::stoi(j.parameters.at("slot"));
    ASSERT_EQ(j.command_interfaces.size(), 1u) << j.name;
    const double lo = std::stod(j.command_interfaces[0].min), hi = std::stod(j.command_interfaces[0].max);
    EXPECT_GE(lo, fw_min[k] - 1e-6) << j.name << " 명령 하한이 펌웨어 범위 밖";
    EXPECT_LE(hi, fw_max[k] + 1e-6) << j.name << " 명령 상한이 펌웨어 범위 밖";
    std::smatch m;
    ASSERT_TRUE(std::regex_search(urdf, m, std::regex("<joint name=\"" + j.name + R"re(" type=[^>]*>[\s\S]*?<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)")re")))
      << j.name << " 의 URDF <limit> 없음";
    EXPECT_GE(lo, std::stod(m[1]) - 1e-6) << j.name << " 명령 하한이 URDF 보다 넓음";
    EXPECT_LE(hi, std::stod(m[2]) + 1e-6) << j.name << " 명령 상한이 URDF 보다 넓음";
  }
  ASSERT_EQ(hw.sensors.size(), 1u);
  EXPECT_EQ(hw.sensors[0].name, "imu");

  // 컨트롤러 설정의 관절은 모두 ros2_control 에 있어야 함 (gripper_controller 는 use_gripper:=true 때만이라 뺌)
  const std::string yaml = read_file(PKG_DIR "/config/controllers_4wd.yaml");
  for (const std::string key : {"left_wheel_names", "right_wheel_names", "joints"}) {
    const auto list = yaml_list(yaml, key);
    EXPECT_FALSE(list.empty()) << key;
    for (const auto & n : list) {EXPECT_TRUE(names.count(n)) << key << ": " << n << " 가 ros2_control 에 없음";}
  }
}

TEST(RobotDescription, RobotUrdfProvidesFramesUsedByOtherNodes)
{
  if (!have_xacro()) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  std::string urdf;
  ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf)) << urdf;
  // ekf · mission · lidar (base_link), Perception 찾기 (base_footprint), IMU broadcaster (imu_link), Controller (arm_base_link · tfmini_link)
  for (const char * link : {"base_link", "base_footprint", "imu_link", "arm_base_link", "tfmini_link"}) {
    EXPECT_NE(urdf.find(std::string("<link name=\"") + link + "\""), std::string::npos) << link;
  }
  // realsense 드라이버가 발행하는 좌표를 URDF 가 또 만들면 TF 부모가 둘이 됨
  EXPECT_EQ(urdf.find("_color_optical_frame\""), std::string::npos);
}
