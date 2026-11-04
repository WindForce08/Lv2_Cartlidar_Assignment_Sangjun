// TargetLookPose — 퍽을 팔 카메라로 내려다보는 타겟체크자세 (#82, manipulator target_look skill)
//
// 왜: 45° 집기 (toppick_tilt) 동안에는 팔 카메라가 집게 축보다 위에서 앞을 봐서 퍽이 화면 아래로 빠진다
//     (잡는 점 6 cm 앞부터 화면 밖, 인지 #82 계산). 그래서 집기 전에 퍽을 내려다보는 자세에서 재고 그 값으로 집는다.
// 어떻게: 퍽 윗면 가운데 (aim, arm_base_link) 를 카메라 광축이 지나가게 —
//     카메라를 aim 에서 J1 쪽으로 distance, 위로 view_angle (광축이 수평 아래로 이루는 각) 만큼 떨어뜨리고,
//     카메라 장착 (집게 축 기준 camera_x · camera_z, 아래로 camera_tilt) 을 거꾸로 따라가 집게 끝 위치 · pitch 를 낸다.
//     후보를 선호 순서 (distance 먼저, 그 안에서 view_angle) 로 내고, 관절 한계 안의 IK 해가 나오는 첫 후보를 쓴다.
//     J1 은 aim 방향이라 퍽은 화면 가로 가운데. 인지 #82 의 4 자세 (A · B · C앞 · C뒤) 는 모두 약 70° · 0.15~0.21 m.
// 좌표: arm_base_link (x 앞 · y 왼 · z 위), 팔 평면 = J1 방향 수직면. 장착 치수 = robot.urdf.xacro arm_camera_link
//     (link5 기준 0.088256 0 0.077446, pitch 0.261788 — link5 는 joint4 축에서 0.04 → joint4 기준 0.128256)
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace control
{

struct TargetLookParams
{
  double camera_x{0.128256};      // joint4 축 → 팔 카메라, 집게 축 방향 (m)
  double camera_z{0.077446};      // 집게 축에 수직 위 (m)
  double camera_tilt{0.261788};   // 카메라 광축이 집게 축보다 아래로 (rad, 15°)
  double wrist_to_tool{0.167};    // joint4 축 → 집게 끝 tool_link (ArmIKParams 와 같게)
  std::vector<double> view_angles{1.2217, 1.2566, 1.1868, 1.3090, 1.1345, 1.3614, 1.3963, 1.0821, 1.4835, 1.0472};
  //                               70°     72°     68°     75°     65°     78°     80°     62°     85°     60°
  std::vector<double> distances{0.17, 0.16, 0.18, 0.15, 0.19, 0.20, 0.21, 0.22};   // 카메라 → aim (m)
  double min_clearance{0.04};     // 집게 끝이 aim 보다 이만큼 위 (퍽 · 상자에 닿지 않게, m)
};

struct TargetLookCandidate
{
  Eigen::Vector3d tool;   // 집게 끝 (arm_base_link)
  double pitch;           // 집게 pitch (위로 +, ArmIK 규약)
  double view_angle;      // 카메라 광축 수평 아래 각 (rad)
  double distance;        // 카메라 → aim (m)
};

// 집게 끝 · pitch · J1 yaw → 카메라 위치 · 광축 단위 벡터 (arm_base_link)
inline void targetLookCamera(
  const Eigen::Vector3d & tool, double pitch, double yaw, const TargetLookParams & p,
  Eigen::Vector3d & camera, Eigen::Vector3d & axis)
{
  const double th = -pitch;   // 집게 축이 수평 아래로 이루는 각
  const Eigen::Vector3d u(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d along = u * std::cos(th) - Eigen::Vector3d::UnitZ() * std::sin(th);   // 집게 축
  const Eigen::Vector3d up = u * std::sin(th) + Eigen::Vector3d::UnitZ() * std::cos(th);      // 집게 축에 수직 위
  const Eigen::Vector3d wrist = tool - along * p.wrist_to_tool;
  camera = wrist + along * p.camera_x + up * p.camera_z;
  const double phi = th + p.camera_tilt;
  axis = u * std::cos(phi) - Eigen::Vector3d::UnitZ() * std::sin(phi);
}

// aim 을 카메라 광축이 지나는 집게 끝 · pitch 후보 (선호 순서). 집게 끝이 aim + min_clearance 보다 낮은 후보는 뺀다
inline std::vector<TargetLookCandidate> targetLookCandidates(const Eigen::Vector3d & aim, const TargetLookParams & p)
{
  std::vector<TargetLookCandidate> out;
  const double yaw = std::atan2(aim.y(), aim.x());
  const Eigen::Vector3d u(std::cos(yaw), std::sin(yaw), 0.0);
  for (double d : p.distances) {
    for (double phi : p.view_angles) {
      const Eigen::Vector3d axis = u * std::cos(phi) - Eigen::Vector3d::UnitZ() * std::sin(phi);
      const Eigen::Vector3d camera = aim - axis * d;
      const double th = phi - p.camera_tilt;
      const Eigen::Vector3d along = u * std::cos(th) - Eigen::Vector3d::UnitZ() * std::sin(th);
      const Eigen::Vector3d up = u * std::sin(th) + Eigen::Vector3d::UnitZ() * std::cos(th);
      const Eigen::Vector3d tool = camera - along * p.camera_x - up * p.camera_z + along * p.wrist_to_tool;
      if (tool.z() < aim.z() + p.min_clearance) {continue;}
      out.push_back({tool, -th, phi, d});
    }
  }
  return out;
}

}  // namespace control
