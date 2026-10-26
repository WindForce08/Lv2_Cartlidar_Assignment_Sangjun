#include "control/arm_ik.hpp"

#include <cmath>

namespace control
{

ArmIK::ArmIK(const ArmIKParams & params)
: p_(params)
{
}

std::optional<ArmJoints> ArmIK::solve(
  const Eigen::Vector3d & p, double pitch, double wrist_roll, bool elbow_up) const
{
  const double l2 = p_.upper_arm;
  const double l3 = p_.forearm;
  const double l4 = p_.wrist_to_tool;

  const double q1 = std::atan2(p.y(), p.x());
  const double r = std::hypot(p.x(), p.y());
  const double z = p.z() - p_.base_height;

  // 손목(joint4) 중심
  const double rw = r - l4 * std::cos(pitch);
  const double zw = z - l4 * std::sin(pitch);

  const double d = (rw * rw + zw * zw - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
  if (d < -1.0 || d > 1.0) {
    return std::nullopt;
  }
  const double q3 = elbow_up ? -std::acos(d) : std::acos(d);
  const double q2 = std::atan2(zw, rw) - std::atan2(l3 * std::sin(q3), l2 + l3 * std::cos(q3));
  const double q4 = pitch - q2 - q3;

  const ArmJoints q_ik{q1, q2, q3, q4, wrist_roll};
  ArmJoints q_cmd;
  for (size_t i = 0; i < q_cmd.size(); ++i) {
    q_cmd[i] = std::remainder(p_.joint_signs[i] * q_ik[i] + p_.joint_offsets[i], 2.0 * M_PI);
  }
  return q_cmd;
}

Eigen::Vector3d ArmIK::forward(const ArmJoints & q_cmd) const
{
  ArmJoints q;
  for (size_t i = 0; i < q.size(); ++i) {
    q[i] = (q_cmd[i] - p_.joint_offsets[i]) / p_.joint_signs[i];
  }
  const double a2 = q[1];
  const double a3 = a2 + q[2];
  const double a4 = a3 + q[3];
  const double r = p_.upper_arm * std::cos(a2) + p_.forearm * std::cos(a3) +
    p_.wrist_to_tool * std::cos(a4);
  const double z = p_.base_height + p_.upper_arm * std::sin(a2) + p_.forearm * std::sin(a3) +
    p_.wrist_to_tool * std::sin(a4);
  return {r * std::cos(q[0]), r * std::sin(q[0]), z};
}

Eigen::Vector3d ArmIK::approachDirection(double yaw, double pitch)
{
  return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
}

}  // namespace control
