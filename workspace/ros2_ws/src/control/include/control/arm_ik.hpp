// ArmIK — 5축 해석적 IK (joint1 요 · joint2~4 피치 · joint5 손목 롤). 6 번째 칸(gripper_roll)은 그리퍼 개폐라 IK 밖
//
// 기구 = 차체 URDF (opencr_hardware/urdf/robot.urdf.xacro) = 기준 모델 B안 = 강화학습 kinematics.py 와 같은 치수 · 규약
//   좌표: arm_base_link (x 앞 · y 왼 · z 위). joint1 축 = arm_base_link 원점의 z 축
//   어깨(joint2) = joint1 축 위 shoulder_height. 위팔 = joint2 → joint3 (upper_arm_x, upper_arm_z), 아래팔 = joint3 → joint4 (forearm)
//   손목(joint4) → 집게 끝(tool_link, 팔레트 궤적 TCP 와 같은 점) = wrist_to_tool
//   피치 관절 축은 +y — 양의 회전이 앞을 아래로 숙인다 (URDF 와 같음). 0 자세: 위팔은 (upper_arm_x, upper_arm_z) 방향, 아래팔 · 집게는 +x
// 집게 pitch (입력 · 출력): 집게 축의 수평 기준 각도, 위로 + (0 = 수평 앞, -pi/4 = 45° 아래, -pi/2 = 바로 아래)
//   joint2 + joint3 + joint4 = -pitch  (kinematics.py 의 pitch 는 아래로 + 라 부호만 반대)
// 실제 관절 명령 = sign * q + offset (manipulator.yaml 의 ik.joint_signs · ik.joint_offsets, 기본 1 · 0)

#pragma once

#include <array>
#include <optional>

#include <Eigen/Core>

namespace control
{

struct ArmIKParams
{
  double shoulder_height{0.077};  // arm_base_link 원점 → joint2 축 높이 (m, robot.urdf.xacro joint1 z 0.077)
  double upper_arm_x{0.024};      // joint2 → joint3 (0 자세에서 x · z 성분, m)
  double upper_arm_z{0.128};
  double forearm{0.124};          // joint3 → joint4 (m)
  double wrist_to_tool{0.167};    // joint4 → 집게 끝 tool_link (m) = 0.04 + 0.041 + 0.086 (URDF)
  std::array<double, 5> joint_offsets{};
  std::array<double, 5> joint_signs{1.0, 1.0, 1.0, 1.0, 1.0};
};

using ArmJoints = std::array<double, 5>;  // joint1..joint5 명령값 (rad)

class ArmIK
{
public:
  explicit ArmIK(const ArmIKParams & params);

  // 집게 끝 p (arm_base_link), 집게 pitch (위로 +), 손목 roll → 관절 명령값. 닿지 않으면 nullopt.
  // 관절 한계는 보지 않는다 (호출하는 쪽이 확인)
  std::optional<ArmJoints> solve(
    const Eigen::Vector3d & p, double pitch, double wrist_roll, bool elbow_up) const;

  // 관절 명령값 → 집게 끝 위치 (arm_base_link)
  Eigen::Vector3d forward(const ArmJoints & q_cmd) const;

  // 관절 명령값 → 집게 pitch (위로 +)
  double forwardPitch(const ArmJoints & q_cmd) const;

  // 집게 축 방향 단위 벡터 (yaw = joint1 방향, pitch 위로 +)
  static Eigen::Vector3d approachDirection(double yaw, double pitch);

private:
  ArmJoints toModel(const ArmJoints & q_cmd) const;

  ArmIKParams p_;
  double link_a_;   // 위팔 길이
  double alpha0_;   // 0 자세 위팔 방향각 (수평 기준, 위로 +)
};

}  // namespace control
