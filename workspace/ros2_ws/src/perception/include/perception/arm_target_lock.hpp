// ARM_ALIGN 목표 퍽 유지 (팔 카메라 검출 중 따라갈 박스 하나, ROS 의존 없음 → 단위 시험)
//
//   좌표는 image_offset(geometry.hpp)과 같은 정규화 값: x · y 화면 중심 대비 −1~1, w = 박스 폭 / 화면 폭
//   1) 잠그기: 잠기지 않았으면 seed(D435 추적 목표를 팔 영상에 투영한 점)에 가장 가까운 박스(거리 ≤ gate).
//      seed 가 있는데 gate 안 박스가 없으면 이번에는 잠그지 않는다. seed 가 없으면 점수가 가장 높은 박스
//   2) 따라가기: 잠근 뒤에는 점수를 보지 않고, 같은 클래스 · 직전 중심에서 gate 안 · 폭이 직전의 1/size_ratio ~ size_ratio 배인
//      박스 중 가장 가까운 것
//   3) 놓침: 맞는 박스가 없으면 -1(정렬 값 없음). 다른 퍽으로 넘어가지 않고, 직전 자리 근처에 다시 보이면 이어 간다
//   4) 풀기: reset() — perception_node 는 ARM_ALIGN 을 켜거나 끌 때 · OFF · 팔 영상 시각 재시작에서 부른다
#pragma once

#include <cmath>
#include <optional>
#include <vector>

namespace perception
{

struct ArmBox
{
  double x = 0, y = 0, w = 0;   // 정규화 중심 · 폭
  int cls = -1;
  double score = 0;
};

struct ArmLockConfig
{
  double gate = 0.25;        // ★ 정규화 거리. 15 Hz 정렬 중 한 프레임 이동보다 넉넉하고 이웃 퍽 사이보다 작게 잡은 가정값
  double size_ratio = 2.0;   // ★ 박스 폭 변화 허용 배율 (가정값)
};

struct ArmPoint
{
  double x = 0, y = 0;
};

class ArmTargetLock
{
public:
  explicit ArmTargetLock(ArmLockConfig c = {})
  : c_(c) {}

  void reset() {locked_ = false;}
  bool locked() const {return locked_;}

  // 따라갈 박스 번호, 없으면 -1
  int update(const std::vector<ArmBox> & boxes, const std::optional<ArmPoint> & seed)
  {
    int best = -1;
    double best_d = 0;
    for (size_t i = 0; i < boxes.size(); ++i) {
      const ArmBox & b = boxes[i];
      double d;
      if (locked_) {
        if (b.cls != last_.cls || b.w < last_.w / c_.size_ratio || b.w > last_.w * c_.size_ratio) {continue;}
        d = std::hypot(b.x - last_.x, b.y - last_.y);
        if (d > c_.gate) {continue;}
      } else if (seed) {
        d = std::hypot(b.x - seed->x, b.y - seed->y);
        if (d > c_.gate) {continue;}
      } else {
        d = -b.score;   // 점수가 높을수록 앞
      }
      if (best < 0 || d < best_d) {best = static_cast<int>(i); best_d = d;}
    }
    if (best >= 0) {
      locked_ = true;
      last_ = boxes[best];
    }
    return best;
  }

private:
  ArmLockConfig c_;
  bool locked_ = false;
  ArmBox last_;
};

}  // namespace perception
