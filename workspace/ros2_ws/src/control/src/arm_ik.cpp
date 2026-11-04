// ArmIK 구현 — tools/arm_bench/rl/arm_pick/kinematics.py 의 ik() · fk_tip() 과 같은 식 (pitch 부호만 반대)
#include "control/arm_ik.hpp"

#include <cmath>

namespace control
{

ArmIK::ArmIK(const ArmIKParams & params)
: p_(params),
  link_a_(std::hypot(params.upper_arm_x, params.upper_arm_z)),
  alpha0_(std::atan2(params.upper_arm_z, params.upper_arm_x))
{
}

std::optional<ArmJoints> ArmIK::solve(
  const Eigen::Vector3d & p, double pitch, double wrist_roll, bool elbow_up) const
{
  const double a = link_a_;
  const double b = p_.forearm;
  const double t = p_.wrist_to_tool;
  const double down = -pitch;   // kinematics.py 규약 (아래로 +)

  const double q1 = std::atan2(p.y(), p.x());
  const double d = std::hypot(p.x(), p.y());
  // 손목(joint4) 중심 — 어깨 기준 (수평 거리, 높이)
  const double vx = d - t * std::cos(down);
  const double vz = p.z() + t * std::sin(down) - p_.shoulder_height;
  const double r = std::hypot(vx, vz);
  if (r > a + b - 1e-4 || r < std::abs(a - b) + 1e-3) {
    return std::nullopt;
  }
  const double cb = (a * a + r * r - b * b) / (2.0 * a * r);
  if (cb < -1.0 || cb > 1.0) {
    return std::nullopt;
  }
  const double phi = std::atan2(vz, vx);
  const double th_a = elbow_up ? phi + std::acos(cb) : phi - std::acos(cb);   // 위팔 방향각
  const double th_b = std::atan2(vz - a * std::sin(th_a), vx - a * std::cos(th_a));   // 아래팔 방향각
  const double q2 = alpha0_ - th_a;
  const double q3 = -th_b - q2;
  const double q4 = down - q2 - q3;

  const ArmJoints q{q1, q2, q3, q4, wrist_roll};
  ArmJoints q_cmd;
  for (size_t i = 0; i < q_cmd.size(); ++i) {
    q_cmd[i] = std::remainder(p_.joint_signs[i] * q[i] + p_.joint_offsets[i], 2.0 * M_PI);
  }
  return q_cmd;
}

ArmJoints ArmIK::toModel(const ArmJoints & q_cmd) const
{
  ArmJoints q;
  for (size_t i = 0; i < q.size(); ++i) {
    q[i] = (q_cmd[i] - p_.joint_offsets[i]) / p_.joint_signs[i];
  }
  return q;
}

Eigen::Vector3d ArmIK::forward(const ArmJoints & q_cmd) const
{
  const ArmJoints q = toModel(q_cmd);
  const double th_a = alpha0_ - q[1];
  const double th_b = -(q[1] + q[2]);
  const double th_t = -(q[1] + q[2] + q[3]);
  const double d = link_a_ * std::cos(th_a) + p_.forearm * std::cos(th_b) + p_.wrist_to_tool * std::cos(th_t);
  const double z = p_.shoulder_height + link_a_ * std::sin(th_a) + p_.forearm * std::sin(th_b) +
    p_.wrist_to_tool * std::sin(th_t);
  return {d * std::cos(q[0]), d * std::sin(q[0]), z};
}

double ArmIK::forwardPitch(const ArmJoints & q_cmd) const
{
  const ArmJoints q = toModel(q_cmd);
  return -(q[1] + q[2] + q[3]);
}

Eigen::Vector3d ArmIK::approachDirection(double yaw, double pitch)
{
  return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
}

}  // namespace control
