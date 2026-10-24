// 신호등 찾기 · 프레임 상태 (ROS 의존 없음 → 단위 시험)
//   1) detect_light_blobs: 밝고(V ≥ v_min) 채도 있는(S ≥ s_min) 빨강 · 노랑 · 초록 영역 = 켜진 등 후보
//      (꺼진 등 · 퍽은 켜진 LED 만큼 밝지 않다고 가정 ★ 렌더 · 실물 영상으로 v_min 을 다시 정함)
//   2) 노드가 깊이로 3D 위치(odom)를 붙이고 크기 · 높이로 거른 뒤 SignalLightLocator 에 넣는다
//   3) SignalLightLocator: 같은 자리(lock_dist 안)에서 켜졌다 꺼지기를 lock_blinks 번 본 자리를 신호등으로 고정.
//      꺼짐은 자리가 화면 안인데 후보가 없는 프레임이 min_off_frames 번 이어져야 센다(검출이 한 프레임 끊긴 것은 꺼짐 아님).
//      퍽처럼 늘 켜져 보이는 것은 꺼짐이 없어 고정되지 않는다.
//      고정 뒤 프레임 상태: 그 자리 근처 후보가 있으면 그 색, 없고 자리가 화면 안이면 꺼짐, 화면 밖이면 모름
//   4) 가림: 후보가 없는 프레임에서 세 칸 중 하나라도 앞이 가려졌으면(노드가 깊이로 판단) 꺼짐이 아니라 모름.
//      가린 칸을 꺼짐으로 세면 점멸을 놓쳐 순서가 틀어지고, 모름(빠진 프레임)은 점멸 세기가 견딘다(U1 빠짐 5 %).
//      칸 위치: 본 색은 그 후보 위치, 못 본 색은 아래부터 초록 · 노랑 · 빨강이 segment_pitch 간격이라고 보고 채운다.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "perception/signal_sequence.hpp"
#include "perception/target_tracker.hpp"

namespace perception
{

struct LightBlobConfig
{
  int v_min = 200;               // ★ 켜진 LED 밝기 하한
  int s_min = 90;                // ★ 채도 하한 (가운데가 하얗게 날아가도 둘레는 색이 남는다고 가정)
  int red_h_max = 10, red2_h_min = 170;
  int yellow_h_min = 15, yellow_h_max = 35;
  int green_h_min = 40, green_h_max = 90;
  int min_area_px = 4;           // 3 m 의 등 한 칸(39.6 × 34.9 mm)이 D435 640 × 480(fx 약 606)에서 약 8 × 7 px
};

struct LightBlob
{
  Light color = Light::Off;
  cv::Rect box;
  cv::Point2f center;
  double area = 0;               // 화소 수
};

inline std::vector<LightBlob> detect_light_blobs(const cv::Mat & bgr, const LightBlobConfig & c)
{
  std::vector<LightBlob> out;
  if (bgr.empty()) {return out;}
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  cv::Mat bright = (ch[1] >= c.s_min) & (ch[2] >= c.v_min);
  const std::pair<Light, cv::Mat> masks[] = {
    {Light::Red, ((ch[0] <= c.red_h_max) | (ch[0] >= c.red2_h_min)) & bright},
    {Light::Yellow, (ch[0] >= c.yellow_h_min) & (ch[0] <= c.yellow_h_max) & bright},
    {Light::Green, (ch[0] >= c.green_h_min) & (ch[0] <= c.green_h_max) & bright},
  };
  for (const auto & [color, mask] : masks) {
    cv::Mat labels, stats, cents;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, cents, 8);
    for (int i = 1; i < n; ++i) {
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area < c.min_area_px) {continue;}
      LightBlob b;
      b.color = color;
      b.box = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
          stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
      b.center = cv::Point2f(static_cast<float>(cents.at<double>(i, 0)), static_cast<float>(cents.at<double>(i, 1)));
      b.area = area;
      out.push_back(b);
    }
  }
  return out;
}

// 켜진 등 후보 하나 (노드가 3D 위치를 붙인 것)
struct LightObs
{
  Light color = Light::Off;
  Vec3 pos;                      // odom 기준 [m]
  double area = 0;
};

struct LocatorConfig
{
  double lock_dist = 0.10;       // [m] 같은 자리로 보는 거리. 빨강 · 초록 칸 중심이 위아래 0.07 m 떨어져 있어 어느 색을 먼저 봐도 세 칸이 한 자리로 묶이게
  int lock_blinks = 3;           // 켜졌다 꺼지기를 이만큼 본 자리를 신호등으로 고정
  int min_off_frames = 2;        // 꺼짐으로 세는 연속 프레임 수
  double forget_s = 10.0;        // [s] 고정 전 자리를 이만큼 못 보면 지움
  double segment_pitch = 0.0349; // [m] 칸 중심 위아래 간격 (칸 길이 34.9 mm, 사용자 실측 2026-10-23)
  std::size_t max_sites = 20;
};

class SignalLightLocator
{
public:
  explicit SignalLightLocator(LocatorConfig c = {})
  : c_(c) {}

  void reset() {sites_.clear(); blocked_.clear(); locked_ = -1;}
  bool locked() const {return locked_ >= 0;}
  Vec3 position() const {return locked() ? sites_[locked_].pos : Vec3{};}
  using Segments = std::array<Vec3, 3>;   // 칸 중심 (아래부터 초록 · 노랑 · 빨강), odom [m]
  Segments segments() const {return locked() ? segments(sites_[locked_]) : Segments{};}

  // in_view(pos): 그 자리가 이 프레임 카메라 화면 안인지
  // occluded(칸 중심 3개): 이 프레임에서 그중 하나라도 앞이 가려졌는지 (없으면 가림을 보지 않음)
  // 반환: 고정 뒤 이 프레임의 신호등 상태. 고정 전이거나 자리가 화면 밖이거나 가려졌으면 nullopt
  std::optional<Light> update(double t, const std::vector<LightObs> & obs,
    const std::function<bool(const Vec3 &)> & in_view,
    const std::function<bool(const Segments &)> & occluded = nullptr)
  {
    std::vector<int> lit(sites_.size(), -1);   // 자리별로 이 프레임에 붙은 후보 (가장 큰 것)
    blocked_.assign(sites_.size(), false);
    for (std::size_t k = 0; k < obs.size(); ++k) {
      int best = -1;
      double bd = c_.lock_dist;
      for (std::size_t i = 0; i < sites_.size(); ++i) {
        const double d = dist(obs[k].pos, sites_[i].pos);
        if (d < bd) {bd = d; best = static_cast<int>(i);}
      }
      if (best < 0) {
        if (locked() || sites_.size() >= c_.max_sites) {continue;}   // 고정 뒤에는 새 자리를 만들지 않음
        sites_.push_back({obs[k].pos, t, false, 0, 0, {}, {}});
        sites_.back().seg[level(obs[k].color)] = obs[k].pos;
        sites_.back().seg_seen[level(obs[k].color)] = true;
        lit.push_back(static_cast<int>(k));
        blocked_.push_back(false);
        continue;
      }
      if (lit[best] < 0 || obs[k].area > obs[lit[best]].area) {lit[best] = static_cast<int>(k);}
    }
    for (std::size_t i = 0; i < sites_.size(); ++i) {
      Site & s = sites_[i];
      if (lit[i] >= 0) {
        const auto & o = obs[lit[i]];
        if (!s.on) {++s.blinks;}
        s.on = true;
        s.off_frames = 0;
        s.last_seen = t;
        // 위치를 천천히 맞춘다 (오도메트리가 흘러도 따라가게)
        s.pos = blend(s.pos, o.pos);
        const int l = level(o.color);
        s.seg[l] = s.seg_seen[l] ? blend(s.seg[l], o.pos) : o.pos;
        s.seg_seen[l] = true;
      } else if (in_view(s.pos)) {
        const bool hidden = occluded && occluded(segments(s));
        if (hidden) {
          blocked_[i] = true;   // 꺼짐으로 세지 않음 (연속 꺼짐 수도 그대로 둠)
        } else if (++s.off_frames >= c_.min_off_frames) {
          s.on = false;
        }
      }
    }
    if (!locked()) {
      for (std::size_t i = 0; i < sites_.size(); ++i) {
        if (sites_[i].blinks >= c_.lock_blinks) {locked_ = static_cast<int>(i); break;}
      }
      // 오래 못 본 자리 정리 (고정 전만, 순서가 바뀌므로 locked_ 를 정하기 전에는 지우지 않음)
      if (!locked()) {
        sites_.erase(std::remove_if(sites_.begin(), sites_.end(),
          [&](const Site & s) {return t - s.last_seen > c_.forget_s;}), sites_.end());
        return std::nullopt;
      }
    }
    const Site & s = sites_[locked_];
    if (lit[locked_] >= 0) {return obs[lit[locked_]].color;}
    if (in_view(s.pos) && !blocked_[locked_]) {return Light::Off;}
    return std::nullopt;
  }

  // 마지막 update 에서 고정된 자리가 가려졌는지
  bool occluded_last() const {return locked() && blocked_.size() > static_cast<std::size_t>(locked_) && blocked_[locked_];}

private:
  struct Site
  {
    Vec3 pos;
    double last_seen = 0;
    bool on = false;
    int blinks = 0;              // 꺼짐 → 켜짐 횟수 (처음 본 켜짐 포함)
    int off_frames = 0;
    Segments seg;                // 색별로 본 칸 중심 (level 순서)
    std::array<bool, 3> seg_seen{};
  };

  static int level(Light l) {return l == Light::Green ? 0 : l == Light::Yellow ? 1 : 2;}

  static Vec3 blend(const Vec3 & a, const Vec3 & b)
  {
    return {0.8 * a.x + 0.2 * b.x, 0.8 * a.y + 0.2 * b.y, 0.8 * a.z + 0.2 * b.z};
  }

  // 본 칸은 그 위치, 못 본 칸은 본 칸들에서 간격으로 미룬 위치 (하나도 못 봤으면 자리 위치 기준 가운데 = 노랑)
  Segments segments(const Site & s) const
  {
    double base = 0;
    int n = 0;
    for (int l = 0; l < 3; ++l) {
      if (s.seg_seen[l]) {base += s.seg[l].z - l * c_.segment_pitch; ++n;}
    }
    base = n ? base / n : s.pos.z - c_.segment_pitch;
    Segments out;
    for (int l = 0; l < 3; ++l) {
      out[l] = s.seg_seen[l] ? s.seg[l] : Vec3{s.pos.x, s.pos.y, base + l * c_.segment_pitch};
    }
    return out;
  }

  static double dist(const Vec3 & a, const Vec3 & b)
  {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
  }

  LocatorConfig c_;
  std::vector<Site> sites_;
  std::vector<bool> blocked_;    // 마지막 update 에서 자리별로 가려졌는지
  int locked_ = -1;
};

}  // namespace perception
