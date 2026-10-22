#include "control/arm_ik.hpp"

#include <cmath>
#include <sstream>

namespace control
{

namespace
{
double wrap(double a)
{
  while (a > M_PI) {a -= 2 * M_PI;}
  while (a < -M_PI) {a += 2 * M_PI;}
  return a;
}
const char * JN[6] = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"};
}  // namespace

bool JointLimits::within(const Joints & q, double margin) const
{
  return violation(q, margin).empty();
}

std::string JointLimits::violation(const Joints & q, double margin) const
{
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(q[i]) || q[i] < lower[i] - margin || q[i] > upper[i] + margin) {
      std::ostringstream os;
      os << JN[i] << " = " << q[i] << " rad (한계 " << lower[i] << " ~ " << upper[i] << ")";
      return os.str();
    }
  }
  return "";
}

Eigen::Vector3d ArmIK::fk_position(const Joints & q) const
{
  const double L2 = std::hypot(g_.l2x, g_.l2z);
  const double alpha = std::atan2(g_.l2z, g_.l2x);
  const double e1 = alpha - q[1];               // 위팔 앙각
  const double e2 = -q[1] - q[2];               // 아래팔 앙각
  const double ep = -q[1] - q[2] - q[3];        // 공구 앙각
  const double r = L2 * std::cos(e1) + g_.l3 * std::cos(e2) + g_.l_tool * std::cos(ep);
  const double z = g_.d1 + L2 * std::sin(e1) + g_.l3 * std::sin(e2) + g_.l_tool * std::sin(ep);
  return {r * std::cos(q[0]), r * std::sin(q[0]), z};
}

std::optional<Joints> ArmIK::solve_pitch(const Eigen::Vector3d & p, double pitch, double yaw_obj,
  double roll, bool elbow_up, std::string & why) const
{
  const double L2 = std::hypot(g_.l2x, g_.l2z);
  const double L3 = g_.l3;
  const double alpha = std::atan2(g_.l2z, g_.l2x);

  Joints q{};
  q[0] = std::atan2(p.y(), p.x());
  const double r = std::hypot(p.x(), p.y());
  // 손목 점 (joint4) — 공구를 pitch 방향으로 뺀 점
  const double wr = r - g_.l_tool * std::cos(pitch);
  const double wz = p.z() - g_.d1 - g_.l_tool * std::sin(pitch);
  const double D2 = wr * wr + wz * wz;
  const double c = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  if (c > 1.0 || c < -1.0) {
    std::ostringstream os;
    os << "손목 점까지 " << std::sqrt(D2) << " m — 팔이 닿지 않음 (공구 앙각 " << pitch << ")";
    why = os.str();
    return std::nullopt;
  }
  // δ = e1 - e2 (위팔과 아래팔 앙각 차). 팔꿈치 위 = δ > 0
  const double delta = (elbow_up ? 1.0 : -1.0) * std::acos(c);
  const double e1 = std::atan2(wz, wr) + std::atan2(L3 * std::sin(delta), L2 + L3 * std::cos(delta));
  const double e2 = e1 - delta;
  q[1] = alpha - e1;
  q[2] = -e2 - q[1];
  q[3] = -pitch + e2;
  // 손목 롤: 공구가 아래를 볼 때 물체 방향과 맞춤 (그리퍼 대칭 → ±90° 안으로)
  double j5 = wrap(yaw_obj - q[0]);
  if (j5 > M_PI / 2) {j5 -= M_PI;}
  if (j5 < -M_PI / 2) {j5 += M_PI;}
  q[4] = j5;
  q[5] = roll;
  for (int i = 1; i < 4; ++i) {q[i] = wrap(q[i]);}

  const std::string v = lim_.violation(q);
  if (!v.empty()) {why = "관절 한계 초과: " + v; return std::nullopt;}
  return q;
}

std::optional<Joints> ArmIK::solve(const IkRequest & req, std::string & why) const
{
  std::vector<double> pitches{req.tool_pitch};
  pitches.insert(pitches.end(), req.pitch_fallbacks.begin(), req.pitch_fallbacks.end());
  std::string last;
  for (double pitch : pitches) {
    for (bool up : {req.elbow_up, !req.elbow_up}) {
      std::string w;
      auto q = solve_pitch(req.position, pitch, req.object_yaw, req.gripper_roll, up, w);
      if (q) {return q;}
      last = w;
    }
  }
  why = "IK 해 없음 — " + last;
  return std::nullopt;
}

}  // namespace control
