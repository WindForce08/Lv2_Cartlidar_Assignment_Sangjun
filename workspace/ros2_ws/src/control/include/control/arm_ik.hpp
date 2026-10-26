// ArmIK — 5축 해석적 IK (+ gripper_roll은 IK 밖에서 고정값)
//
// TODO(Controller): 기구 구조는 (추정)이다. URDF가 확정되면 아래 가정을 확인할 것.
//   joint1: arm_base_link z축 회전 (0 = +x 방향)
//   joint2: 어깨 pitch, 수평 기준 각도 (위로 +)
//   joint3: 팔꿈치 pitch, 상완 연장선 기준 상대 각도 (위로 +)
//   joint4: 손목 pitch, 상대 각도. 끝단 pitch = q2 + q3 + q4 (0 = 수평, -pi/2 = 아래를 향함)
//   joint5: 손목 roll (끝단 축 회전)
// 실제 관절 명령 = sign * q_ik + offset (manipulator.yaml의 ik.joint_signs · ik.joint_offsets)

#pragma once

#include <array>
#include <optional>

#include <Eigen/Core>

namespace control
{

struct ArmIKParams
{
  double base_height{0.0};    // arm_base_link 원점 → joint2 축 높이 (m)
  double upper_arm{0.0};      // joint2 → joint3 (m)
  double forearm{0.0};        // joint3 → joint4 (m)
  double wrist_to_tool{0.0};  // joint4 → 파지점 (m)
  std::array<double, 5> joint_offsets{};
  std::array<double, 5> joint_signs{1.0, 1.0, 1.0, 1.0, 1.0};
};

using ArmJoints = std::array<double, 5>;  // joint1..joint5 명령값 (rad)

class ArmIK
{
public:
  explicit ArmIK(const ArmIKParams & params);

  // 파지점 위치 p (arm_base_link), 끝단 pitch, 손목 roll → 관절 명령값. 도달 불가면 nullopt.
  std::optional<ArmJoints> solve(
    const Eigen::Vector3d & p, double pitch, double wrist_roll, bool elbow_up) const;

  // 관절 명령값 → 파지점 위치 (arm_base_link)
  Eigen::Vector3d forward(const ArmJoints & q_cmd) const;

  // 끝단(접근) 방향 단위 벡터
  static Eigen::Vector3d approachDirection(double yaw, double pitch);

private:
  ArmIKParams p_;
};

}  // namespace control
