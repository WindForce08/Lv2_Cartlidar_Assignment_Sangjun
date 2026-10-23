// 라이다 여러 대의 점을 한 평면 스캔으로 합친다 (ROS 의존 없음 → 단위 시험)
//   점은 이미 출력 프레임(base_link) 좌표로 바꾼 뒤 넣는다. 라이다가 거꾸로 달려(roll π) 있어도
//   TF 로 바꾼 좌표를 쓰므로 각도 방향이 저절로 맞는다.
//   각 칸에는 가장 가까운 점 거리만 남긴다(장애물을 놓치지 않는 쪽). 점이 없는 칸은 +inf
//   차체에 맞은 점(self_box 안)은 버린다 — 라이다가 기둥·바퀴·팔을 장애물로 보지 않게
#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace perception
{

struct Box2
{
  double x_min = 0, x_max = 0, y_min = 0, y_max = 0;
  bool contains(double x, double y) const {return x >= x_min && x <= x_max && y >= y_min && y <= y_max;}
  bool empty() const {return !(x_max > x_min && y_max > y_min);}
};

struct MergeConfig
{
  int bins = 360;                     // 360 칸 = 1° 간격
  double angle_min = -M_PI;           // 첫 칸 중심 각도 [rad]
  double range_min = 0.0;             // 이보다 가까운 점은 버린다 (출력 원점 기준) [m]
  double range_max = 12.0;
  Box2 self_box;                      // 출력 프레임 기준 차체 상자. 비면 거르지 않는다
};

class ScanBinner
{
public:
  explicit ScanBinner(const MergeConfig & c)
  : c_(c), ranges_(static_cast<size_t>(c.bins), std::numeric_limits<float>::infinity()) {}

  double increment() const {return 2.0 * M_PI / c_.bins;}

  void reset() {std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());}

  // 출력 프레임 좌표의 점 하나. 넣었으면 true
  bool add(double x, double y)
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {return false;}
    if (!c_.self_box.empty() && c_.self_box.contains(x, y)) {return false;}
    const double r = std::hypot(x, y);
    if (r < c_.range_min || r > c_.range_max) {return false;}
    const int i = bin_of(std::atan2(y, x));
    if (r < ranges_[static_cast<size_t>(i)]) {ranges_[static_cast<size_t>(i)] = static_cast<float>(r);}
    return true;
  }

  // 각도 → 칸 번호. 칸 i 의 중심은 angle_min + i·increment, 경계는 중심 ± increment/2
  int bin_of(double a) const
  {
    const double inc = increment();
    double k = std::floor((a - c_.angle_min) / inc + 0.5);
    k = std::fmod(k, static_cast<double>(c_.bins));
    if (k < 0) {k += c_.bins;}
    return static_cast<int>(k);
  }

  const std::vector<float> & ranges() const {return ranges_;}

private:
  MergeConfig c_;
  std::vector<float> ranges_;
};

}  // namespace perception
