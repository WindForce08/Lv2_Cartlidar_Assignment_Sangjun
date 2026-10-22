// ArmIK — 5축 해석적 IK + 롤 (joint1 요 · joint2~4 피치 · joint5 손목 롤 · gripper_roll)
// 좌표: arm_base_link (x 앞 · y 왼 · z 위). 피치 관절 축 = +y (양수 = 앞으로 숙임), URDF 와 같은 규약.
// 0 자세: 위팔은 (l2x, l2z) 방향, 아래팔 · 공구는 +x.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace control
{

struct ArmGeometry
{
  double d1 = 0.077;     // arm_base_link → joint2 높이
  double l2x = 0.024;    // joint2 → joint3 (0 자세에서 x, z)
  double l2z = 0.128;
  double l3 = 0.124;     // joint3 → joint4
  double l_tool = 0.167; // joint4 → 공구 끝 (tool_link)
};

using Joints = std::array<double, 6>;   // joint1~5, gripper_roll

struct JointLimits
{
  Joints lower{{-2.8, -1.8, -1.6, -1.8, -2.8, -2.8}};
  Joints upper{{2.8, 1.6, 1.5, 2.0, 2.8, 2.8}};
  bool within(const Joints & q, double margin = 0.0) const;
  std::string violation(const Joints & q, double margin = 0.0) const;   // 빈 문자열 = 정상
};

struct IkRequest
{
  Eigen::Vector3d position;      // 공구 끝 목표 (arm_base_link)
  double tool_pitch = -1.5708;   // 공구 방향 앙각 (rad, -π/2 = 바로 아래)
  double object_yaw = 0.0;       // 물체 방향 (arm_base_link z 회전) — joint5 로 맞춤
  double gripper_roll = 0.0;
  std::vector<double> pitch_fallbacks{-1.3, -1.1, -0.9};   // 위 앙각으로 해가 없을 때 차례로 시도
  bool elbow_up = true;
};

class ArmIK
{
public:
  ArmIK(ArmGeometry g, JointLimits lim)
  : g_(g), lim_(lim) {}

  // 공구 끝 위치 (arm_base_link)
  Eigen::Vector3d fk_position(const Joints & q) const;
  // 공구 방향 앙각 = -(q2+q3+q4)
  static double tool_pitch(const Joints & q) {return -(q[1] + q[2] + q[3]);}

  // 해가 없거나 관절 한계를 넘으면 nullopt, why 에 이유
  std::optional<Joints> solve(const IkRequest & req, std::string & why) const;

  const JointLimits & limits() const {return lim_;}
  const ArmGeometry & geometry() const {return g_;}

private:
  std::optional<Joints> solve_pitch(const Eigen::Vector3d & p, double pitch, double yaw_obj,
    double roll, bool elbow_up, std::string & why) const;
  ArmGeometry g_;
  JointLimits lim_;
};

}  // namespace control
