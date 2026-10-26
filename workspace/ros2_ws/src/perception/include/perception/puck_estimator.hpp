// 태그 하나 → 퍽 중심 · 긴 축 · 색 (perception_node, ROS 의존 없음 → 단위 시험)
//   가정: 퍽 30 × 30 × 60 mm (원기둥은 지름 30 · 높이 60), 태그는 면 가운데에 면 모서리와 나란히 붙음
//   1) 색: 태그 평면 위 둘레 띠(ring_in~ring_out: 흰 여백 바깥 · 면 안)를 영상에 투영해 HSV 다수결.
//      띠에 색이 없으면 바깥 띠(ring2)를 한 번 더 본다
//   1-1) 띠가 태그 흰 여백과 같은 흰색이면 면 전체가 스티커인 원기둥 끝면으로 본다(가정: 원기둥 위 · 아래는
//      면 전체 스티커, 직육면체 면은 스티커 둘레에 퍽 색이 보임) → 끝면 · 원기둥
//   2) 어느 면인지: 태그 평면의 두 축 방향 ±axis_probe 지점이 양쪽 모두 퍽 색이면 그 축이 긴 축(옆면).
//      두 축 모두 아니면 끝면(긴 축 = 태그 법선). 색을 모르면 판단하지 않는다(orientation_valid = false, 옆면으로 봄)
//   3) 중심: 태그 중심에서 법선 반대로 옆면이면 half_width, 끝면이면 half_length
//   4) 끝면의 모양: 태그 대각선 네 방향 중심에서 corner_probe × √2(18 mm) 지점이 퍽 색인지.
//      직육면체 끝면(30 × 30)은 그 지점이 면 안, 원기둥 끝면(지름 30)은 면 밖이다.
//      퍽 색이 3곳 이상이면 직육면체, 1곳 이하면 원기둥, 아니면 모름
//      (과제 자료 2026-10-23: 태그는 윗면 · 아랫면의 20 × 20 mm 라벨이라 끝면 둘레에 퍽 색이 보인다)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"
#include "perception/tag_detector.hpp"

namespace perception
{

struct PuckGeometry
{
  double half_width = 0.015;     // 짧은 변 / 2 (원기둥 반지름)
  double half_length = 0.030;    // 긴 변 / 2
  double tag_half = 0.009;       // 태그 검은 테두리 바깥 한 변 / 2 (노드가 tag.size / 2 로 채움). 흰 여백은 한 칸(1/8) 더
  double ring_in = 0.0125;       // 태그 0.018 m + 흰 여백 1칸 = 0.0225 m → 반 0.01125 m 바깥
  double ring_out = 0.0145;      // 면 반폭 0.015 m 안
  double ring2_in = 0.016;
  double ring2_out = 0.020;
  double axis_probe = 0.022;     // 옆면 긴 축 방향은 면 안(0.03), 짧은 축 방향은 면 밖(0.015)
  double corner_probe = 0.0127;  // 끝면 모양: 태그 축 방향 성분. 대각 거리 18 mm(원기둥 반지름 15 밖 · 직육면체 모서리 21 안)
  double probe_half = 0.0015;    // 탐침 3 × 3 점 간격
  int min_votes = 6;             // 색을 정할 최소 화소 수
  double cap_white_ratio = 0.8;  // 띠 밝기(V) / 흰 여백 밝기가 이 이상이고 채도가 낮으면 흰 스티커
  int cap_max_s = 60;
};

struct PuckEstimate
{
  int color_cls = -1;            // Detection.class_id 와 같음: 0 red · 1 green · 2 blue · -1 모름
  double confidence = 0;         // 띠 화소 중 고른 색 비율
  bool orientation_valid = false;
  bool end_face = false;
  bool cylinder_cap = false;     // 면 전체가 흰 스티커인 원기둥 끝면
  int shape = 0;                 // 0 모름 · 1 원기둥 · 2 직육면체 (robot_interfaces/Puck SHAPE_* 와 같음)
  cv::Vec3d center;              // 카메라 광학 좌표 [m]
  cv::Matx33d axes = cv::Matx33d::eye();   // 열: x = 보이는 면 바깥 법선(옆면) 또는 태그 x(끝면) · y · z = 긴 축
};

// HSV 화소 하나 → 색 번호. 어느 범위에도 없으면 -1
inline int classify_hsv(const cv::Vec3b & p, const std::vector<HsvRange> & ranges)
{
  for (const auto & r : ranges) {
    bool in = true;
    for (int i = 0; i < 3; ++i) {in = in && p[i] >= r.lower[i] && p[i] <= r.upper[i];}
    if (in) {return r.cls;}
  }
  return -1;
}

namespace detail
{
// 태그 평면 점 (x, y, 0) → 화소. 카메라 뒤면 false
inline bool project(const cv::Matx33d & R, const cv::Vec3d & t, const cv::Mat & K, double x, double y,
  const cv::Size & size, cv::Point & px)
{
  const cv::Vec3d p = R * cv::Vec3d(x, y, 0.0) + t;
  if (p[2] <= 1e-6) {return false;}
  const double u = K.at<double>(0, 0) * p[0] / p[2] + K.at<double>(0, 2);
  const double v = K.at<double>(1, 1) * p[1] / p[2] + K.at<double>(1, 2);
  px = cv::Point(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
  return px.x >= 0 && px.y >= 0 && px.x < size.width && px.y < size.height;
}

// 정사각 띠 [r_in, r_out] 의 색 표. votes[3], 투영된 화소 수 반환
inline int ring_votes(const cv::Mat & hsv, const cv::Matx33d & R, const cv::Vec3d & t, const cv::Mat & K,
  const std::vector<HsvRange> & ranges, double r_in, double r_out, std::array<int, 3> & votes)
{
  votes = {0, 0, 0};
  int n = 0;
  const double step = 0.0005;
  for (double r = r_in; r <= r_out + 1e-9; r += step) {
    for (double s = -r; s <= r + 1e-9; s += step) {
      const double pts[4][2] = {{s, r}, {s, -r}, {r, s}, {-r, s}};
      for (const auto & q : pts) {
        cv::Point px;
        if (!project(R, t, K, q[0], q[1], hsv.size(), px)) {continue;}
        ++n;
        const int c = classify_hsv(hsv.at<cv::Vec3b>(px), ranges);
        if (c >= 0 && c < 3) {++votes[c];}
      }
    }
  }
  return n;
}

// 태그 평면 (x, y) 둘레 3 × 3 점 중 색 cls 가 절반 넘는지
inline bool probe_is(const cv::Mat & hsv, const cv::Matx33d & R, const cv::Vec3d & t, const cv::Mat & K,
  const std::vector<HsvRange> & ranges, double x, double y, double h, int cls)
{
  int hit = 0, n = 0;
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      cv::Point px;
      if (!project(R, t, K, x + i * h, y + j * h, hsv.size(), px)) {continue;}
      ++n;
      hit += classify_hsv(hsv.at<cv::Vec3b>(px), ranges) == cls;
    }
  }
  return n > 0 && 2 * hit > n;
}

// 원형 띠(반지름 r_in~r_out) 화소 중 흰 스티커(채도 max_s 미만, 밝기 ≥ ratio × ref_v)의 비율.
// 원기둥 끝면(반지름 15 mm) 안만 보도록 원으로 잰다 (정사각 띠는 모서리가 끝면 밖으로 나감)
inline double white_fraction(const cv::Mat & hsv, const cv::Matx33d & R, const cv::Vec3d & t, const cv::Mat & K,
  double r_in, double r_out, double ref_v, double ratio, int max_s)
{
  int n = 0, w = 0;
  for (double r = r_in; r <= r_out + 1e-9; r += 0.0005) {
    for (int k = 0; k < 180; ++k) {
      const double a = k * M_PI / 90.0;
      cv::Point px;
      if (!project(R, t, K, r * std::cos(a), r * std::sin(a), hsv.size(), px)) {continue;}
      ++n;
      const cv::Vec3b & p = hsv.at<cv::Vec3b>(px);
      w += p[1] < max_s && p[2] >= ratio * ref_v;
    }
  }
  return n > 0 ? static_cast<double>(w) / n : 0.0;
}

// 정사각 띠 화소 밝기(V) 중앙값
inline double ring_value_median(const cv::Mat & hsv, const cv::Matx33d & R, const cv::Vec3d & t, const cv::Mat & K,
  double r_in, double r_out)
{
  std::vector<int> v;
  const double step = 0.0005;
  for (double r = r_in; r <= r_out + 1e-9; r += step) {
    for (double s = -r; s <= r + 1e-9; s += step) {
      const double pts[4][2] = {{s, r}, {s, -r}, {r, s}, {-r, s}};
      for (const auto & q : pts) {
        cv::Point px;
        if (project(R, t, K, q[0], q[1], hsv.size(), px)) {v.push_back(hsv.at<cv::Vec3b>(px)[2]);}
      }
    }
  }
  if (v.empty()) {return 0.0;}
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

inline cv::Matx33d from_columns(const cv::Vec3d & x, const cv::Vec3d & y, const cv::Vec3d & z)
{
  return cv::Matx33d(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}
}  // namespace detail

// bgr: 왜곡 보정한 영상, K: 그 영상의 내부값(왜곡 0), tag: pose_valid 인 검출 (태그 z 축 = 면에서 카메라 쪽)
inline PuckEstimate estimate_puck(const cv::Mat & bgr, const cv::Mat & K, const TagDetection & tag,
  const std::vector<HsvRange> & ranges, const PuckGeometry & g)
{
  PuckEstimate e;
  cv::Matx33d R;
  cv::Rodrigues(tag.rvec, R);
  const cv::Vec3d t = tag.tvec;
  const cv::Vec3d ex(R(0, 0), R(1, 0), R(2, 0)), ey(R(0, 1), R(1, 1), R(2, 1)), ez(R(0, 2), R(1, 2), R(2, 2));
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  std::array<int, 3> votes;
  int n = detail::ring_votes(hsv, R, t, K, ranges, g.ring_in, g.ring_out, votes);
  if (*std::max_element(votes.begin(), votes.end()) < g.min_votes) {
    // 흰 여백(검은 테두리 바깥 ~ 여백 바깥) 밝기를 기준으로 띠가 같은 흰 스티커인지 본다
    const double margin_half = g.tag_half * 10.0 / 8.0;
    const double ref_v = detail::ring_value_median(hsv, R, t, K, g.tag_half + 0.0005, margin_half - 0.0005);
    e.cylinder_cap = ref_v > 0 &&
      detail::white_fraction(hsv, R, t, K, g.ring_in, g.ring_out, ref_v, g.cap_white_ratio, g.cap_max_s) > 0.7;
    n = detail::ring_votes(hsv, R, t, K, ranges, g.ring2_in, g.ring2_out, votes);
  }
  const int best = static_cast<int>(std::max_element(votes.begin(), votes.end()) - votes.begin());
  const int sum = votes[0] + votes[1] + votes[2];
  if (votes[best] >= g.min_votes && 2 * votes[best] > sum && n > 0) {
    e.color_cls = best;
    e.confidence = static_cast<double>(votes[best]) / n;
  }

  double depth = g.half_width;
  if (e.cylinder_cap) {               // 원기둥 끝면: 긴 축 = 법선 (색을 몰라도 자세는 안다)
    e.end_face = true;
    depth = g.half_length;
    e.axes = detail::from_columns(ex, ez.cross(ex), ez);
    e.orientation_valid = true;
    e.shape = 1;
  } else if (e.color_cls >= 0) {
    auto both = [&](double ux, double uy) {
      return detail::probe_is(hsv, R, t, K, ranges, ux * g.axis_probe, uy * g.axis_probe, g.probe_half, e.color_cls) &&
             detail::probe_is(hsv, R, t, K, ranges, -ux * g.axis_probe, -uy * g.axis_probe, g.probe_half, e.color_cls);
    };
    const bool along_x = both(1, 0), along_y = both(0, 1);
    if (along_x != along_y) {         // 옆면: 긴 축이 태그 평면 안
      const cv::Vec3d z = along_x ? ex : ey;
      e.axes = detail::from_columns(ez, z.cross(ez), z);
      e.orientation_valid = true;
    } else if (!along_x) {            // 끝면: 긴 축 = 법선
      e.end_face = true;
      depth = g.half_length;
      e.axes = detail::from_columns(ex, ez.cross(ex), ez);
      e.orientation_valid = true;
      int corners = 0;
      for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
          corners += detail::probe_is(hsv, R, t, K, ranges, sx * g.corner_probe, sy * g.corner_probe, g.probe_half,
            e.color_cls);
        }
      }
      e.shape = corners >= 3 ? 2 : corners <= 1 ? 1 : 0;
    }
  }
  e.center = t - ez * depth;
  return e;
}

}  // namespace perception
