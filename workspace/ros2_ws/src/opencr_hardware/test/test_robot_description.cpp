// 로봇 설명 시험 (2026-10-27 arm_bench 이식, xacro 가 없는 PC 에서는 건너뛴다).
//   urdf/robot.urdf.xacro (기본 인자: 4륜 · use_gripper false) 를 펼쳐 ros2_control 의 실제 파서에 넣고
//   1) 관절 이름 · 역할 · 칸이 펌웨어 칸 순서와 같은지, 팔 명령 한계가 펌웨어 config.h 범위와 URDF <limit> 안인지
//      (펌웨어는 범위 밖 CMD 를 통째로 거부한다), 컨트롤러 설정 (config/controllers_4wd.yaml) 의 관절이 있는지
//   2) 다른 노드가 쓰는 좌표가 있는지, 구동 방식 · 바퀴 수마다 TF 가 base_link 뿌리 하나인 나무인지 본다 (docs/frames_tf.md).
//   3) 구동 방식별 diff_drive 설정 (config/drive_<belt|direct>.yaml) 이 URDF 바퀴 간격 · 반지름 · 속도 한계와 맞는지 본다.
//   hardware_interface 6.x 는 URDF 에 없는 ros2_control 관절을 거부하므로 (Joint '...' not found in URDF), 이름이 어긋나면 여기서 잡힌다.
#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
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

// xacro 글을 임시 파일로 써서 펼친다 (args 예: "drive:=direct"). $(find opencr_hardware) 는 소스 폴더로 (설치 · ament 색인 없이)
bool expand_xacro(std::string text, std::string & urdf, const std::string & args = "")
{
  text = std::regex_replace(text, std::regex(R"(\$\(find opencr_hardware\))"), PKG_DIR);
  char tmp[] = "/tmp/robot_description_XXXXXX";
  const int fd = mkstemp(tmp);
  if (fd < 0) {return false;}
  close(fd);
  std::ofstream(tmp) << text;
  FILE * p = popen((std::string("xacro ") + tmp + " " + args + " 2>&1").c_str(), "r");
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

// yaml 의 첫 "key: 숫자" (key 는 정규식, 점은 \.)
double yaml_number(const std::string & yaml, const std::string & key)
{
  std::smatch m;
  return std::regex_search(yaml, m, std::regex(key + R"(:\s*(-?[0-9.]+))")) ? std::stod(m[1]) : NAN;
}

// 펼친 URDF 의 관절 (<ros2_control> 안의 <joint> 는 type 이 없어 빠진다)
struct UrdfJoint
{
  std::string type, parent, child;
  double xyz[3]{}, rpy[3]{};
};

std::map<std::string, UrdfJoint> urdf_joints(const std::string & urdf)
{
  const auto attr = [](const std::string & body, const std::string & re) {
      std::smatch m;
      return std::regex_search(body, m, std::regex(re)) ? m[1].str() : std::string();
    };
  std::map<std::string, UrdfJoint> out;
  const std::regex block(R"re(<joint name="([^"]+)" type="([^"]+)">([\s\S]*?)</joint>)re");
  for (auto it = std::sregex_iterator(urdf.begin(), urdf.end(), block); it != std::sregex_iterator(); ++it) {
    UrdfJoint j;
    j.type = (*it)[2];
    const std::string body = (*it)[3];
    j.parent = attr(body, R"re(<parent link="([^"]+)")re");
    j.child = attr(body, R"re(<child link="([^"]+)")re");
    std::istringstream(attr(body, R"re(<origin[^>]*xyz="([^"]+)")re")) >> j.xyz[0] >> j.xyz[1] >> j.xyz[2];
    std::istringstream(attr(body, R"re(<origin[^>]*rpy="([^"]+)")re")) >> j.rpy[0] >> j.rpy[1] >> j.rpy[2];
    out[(*it)[1]] = j;
  }
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
  // 구동 방식 · 바퀴 수가 달라도 TF 는 base_link 뿌리 하나인 나무 (링크마다 부모 하나 · 순환 · 끊김 없음)
  for (const std::string args : {"drive:=belt", "drive:=direct", "wheel_count:=6"}) {
    std::string urdf;
    ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf, args)) << args << "\n" << urdf;
    std::set<std::string> links;
    const std::regex link_re(R"re(<link name="([^"]+)")re");
    for (auto it = std::sregex_iterator(urdf.begin(), urdf.end(), link_re); it != std::sregex_iterator(); ++it) {links.insert((*it)[1]);}
    std::map<std::string, std::string> parent;
    for (const auto & [name, j] : urdf_joints(urdf)) {
      EXPECT_TRUE(links.count(j.parent) && links.count(j.child)) << args << ": " << name << " 의 링크가 없음";
      EXPECT_TRUE(parent.emplace(j.child, j.parent).second) << args << ": " << j.child << " 의 부모가 둘";
    }
    for (const auto & link : links) {
      std::string cur = link;
      for (size_t n = 0; parent.count(cur) && n <= links.size(); ++n) {cur = parent.at(cur);}
      EXPECT_EQ(cur, "base_link") << args << ": " << link << " 이 base_link 에 닿지 않음 (순환 또는 끊김)";
    }
  }

  std::string urdf;
  ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf)) << urdf;
  // ekf · mission · lidar (base_link), Perception 찾기 (base_footprint), IMU broadcaster (imu_link), Controller (arm_base_link),
  // TFmini (tfmini_link), /arm_range · /perception/arm_range 기준 (tool_link — perception arm.range.frame · arm.plane.range_frame,
  // manipulator range_frame), LiDAR 드라이버 frame_id (laser_front · laser_rear), Arducam UVC 드라이버 frame_id (arm_camera_optical_frame),
  // realsense2_camera TF 뿌리 (top_camera_front_link · top_camera_rear_link)
  for (const char * link : {"base_link", "base_footprint", "imu_link", "arm_base_link", "tfmini_link", "tool_link",
      "laser_front", "laser_rear", "arm_camera_optical_frame", "top_camera_front_link", "top_camera_rear_link"})
  {
    EXPECT_NE(urdf.find(std::string("<link name=\"") + link + "\""), std::string::npos) << link;
  }
  const auto joints = urdf_joints(urdf);
  // Arducam 은 그리퍼 몸체에 장착 (2026-10-28 확인). 그리퍼 개폐 모터(gripper_roll) 아래에 두면 여닫을 때 카메라 좌표가 돈다 → 손목 롤 뒤 link5
  // 광학 프레임: +z 앞 · +x 오른쪽 · +y 아래 → 몸체 프레임 기준 rpy (-π/2, 0, -π/2)
  ASSERT_EQ(joints.count("arm_camera_link_joint"), 1u);
  EXPECT_EQ(joints.at("arm_camera_link_joint").parent, "link5");
  ASSERT_EQ(joints.count("arm_camera_optical_frame_joint"), 1u);
  const auto & optical = joints.at("arm_camera_optical_frame_joint");
  EXPECT_EQ(optical.parent, "arm_camera_link");
  EXPECT_NEAR(optical.rpy[0], -M_PI / 2, 1e-5);
  EXPECT_NEAR(optical.rpy[1], 0.0, 1e-5);
  EXPECT_NEAR(optical.rpy[2], -M_PI / 2, 1e-5);
  // OpenCR 보드는 바로 놓여 있다 (2026-11-02 확인). roll 이 π 면 ekf 의 회전 속도 부호가 뒤집혀 odom 이 반대로 돈다
  ASSERT_EQ(joints.count("imu_link_joint"), 1u);
  EXPECT_EQ(joints.at("imu_link_joint").parent, "base_link");
  EXPECT_NEAR(joints.at("imu_link_joint").rpy[0], 0.0, 1e-5);
  EXPECT_NEAR(joints.at("imu_link_joint").rpy[1], 0.0, 1e-5);
  // LDS-03 은 뒤집어 장착 (roll π). 0 이면 스캔 좌우가 뒤집힌다
  for (const char * laser : {"laser_front_joint", "laser_rear_joint"}) {
    ASSERT_EQ(joints.count(laser), 1u) << laser;
    EXPECT_NEAR(std::abs(joints.at(laser).rpy[0]), M_PI, 1e-5) << laser;
  }
  // TFmini 는 차체 1층 플레이트 앞 (박스 앞면 거리, drive FINE · obstacle_stop) — 팔이 움직여도 거리 기준이 바뀌지 않아야 한다
  ASSERT_EQ(joints.count("tfmini_link_joint"), 1u);
  EXPECT_EQ(joints.at("tfmini_link_joint").parent, "base_link");
  // realsense 드라이버가 발행하는 좌표를 URDF 가 또 만들면 TF 부모가 둘이 됨
  EXPECT_EQ(urdf.find("_color_optical_frame\""), std::string::npos);
}

// 구동 방식별 diff_drive 설정 (drive_<belt|direct>.yaml) = URDF 바퀴 (README '실물에서 맞출 값': wheel_y · wheel_separation)
TEST(RobotDescription, DriveConfigMatchesUrdfWheels)
{
  if (!have_xacro()) {GTEST_SKIP() << "xacro 없음 (sudo apt install ros-lyrical-xacro)";}
  const double radius = yaml_number(read_file(PKG_DIR "/config/controllers_4wd.yaml"), "wheel_radius");
  ASSERT_FALSE(std::isnan(radius));
  for (const std::string drive : {"belt", "direct"}) {
    std::string urdf;
    ASSERT_TRUE(expand_xacro(read_file(PKG_DIR "/urdf/robot.urdf.xacro"), urdf, "drive:=" + drive)) << urdf;
    const auto joints = urdf_joints(urdf);
    ASSERT_EQ(joints.count("wheel_left_front_joint"), 1u);
    const auto & lf = joints.at("wheel_left_front_joint");
    for (const char * wheel : {"wheel_left_rear_joint", "wheel_right_front_joint", "wheel_right_rear_joint"}) {
      ASSERT_EQ(joints.count(wheel), 1u) << wheel;
      EXPECT_NEAR(std::abs(joints.at(wheel).xyz[1]), lf.xyz[1], 1e-6) << drive << ": " << wheel << " |y|";
      EXPECT_NEAR(joints.at(wheel).xyz[2], lf.xyz[2], 1e-6) << drive << ": " << wheel << " z";
    }
    const std::string yaml = read_file(std::string(PKG_DIR "/config/drive_") + drive + ".yaml");
    EXPECT_NEAR(yaml_number(yaml, "wheel_separation"), 2 * lf.xyz[1], 1e-4) << drive;
    // base_footprint = 바퀴 중심 아래 반지름만큼
    EXPECT_NEAR(lf.xyz[2] - joints.at("base_footprint_joint").xyz[2], radius, 1e-6) << drive;
    std::smatch m;
    ASSERT_TRUE(std::regex_search(urdf, m, std::regex(R"re(<joint name="wheel_left_front_joint" type=[^>]*>[\s\S]*?<limit[^>]*velocity="([^"]+)")re")));
    EXPECT_LE(yaml_number(yaml, R"(linear\.x\.max_velocity)"), std::stod(m[1]) * radius + 1e-6) << drive << ": 바퀴 속도 한계보다 빠름";
  }
}
