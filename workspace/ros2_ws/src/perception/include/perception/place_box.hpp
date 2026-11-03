// 장소 상자 윗면 찾기 (팔 Arducam 한 장, 깊이 없음. ROS 의존 없음 → 단위 시험)
//   놓기 칸을 발제 지도 좌표가 아니라 눈앞 상자 기준으로 정하려는 것 (상자가 돌아가 있거나 밀려 있을 수 있음).
//   가정: 상자는 바닥에 놓인 box × box (기본 205 mm) 정사각 윗면, 높이 height (기본 0.07 m). 태그 위치에는 기대지 않는다.
//   1) 윗면 높이 평면을 정사영 영상(res, 기본 1 mm / 화소)으로 펼침 (평면 → 영상은 호모그래피) → 윗면은 실제 크기 정사각형
//   2) 크기를 아는 정사각형을 (중심, 요 ±45°) 거친 격자 → 고운 격자로 찾음. 점수: 변마다 안 · 밖 띠 Lab 차이 중앙값의 크기를 더하고,
//      네 변 안쪽 색이 서로 다르면 깎고(윗면은 한 색), 가장 약한 변의 대비를 더함(네 변 모두 실제 경계).
//      퍽 색(빨강 · 초록, 검출기 HSV 범위) 화소는 표본에서 뺌 — 놓인 퍽은 원근으로 바깥에 기울어 보여 테두리를 가림
//   3) 테두리 밝기 기울기 중앙값으로 다시 다듬고, 변마다 경계점(화소 이하)을 정사각형에 Huber 가중 최소제곱으로 맞춤
//   4) 정사각형은 90° 마다 같음 → 사전 요 ±45° 안에서 고름 (상자가 그보다 많이 돌지 않는다고 가정)
//   근거(모의, Isaac Sim 렌더 2026-11-03, 상자 요 ±30° · 옆 ±5 cm · 놓인 퍽 0 ~ 9 개, 카메라 윗면 위 0.20 m):
//      Python 시제품 판정용 120/120 — 중심 최대 0.68 mm · 요 0.37° · 칸 0.78 mm (lv2_next_module 261103_1114_place_box)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

namespace perception
{

struct PlaceBoxConfig
{
  double box = 0.205;          // [m] 윗면 한 변
  double height = 0.07;        // [m] 윗면 높이 (평면 기준 프레임 z)
  double res = 0.001;          // [m / 화소] 정사영 영상
  double span = 0.44;          // [m] 정사영 영상 한 변 (사전 중심 둘레)
  double search = 0.08;        // [m] 사전 중심에서 찾는 범위 (±)
  double coarse_xy = 0.004;    // [m] 거친 격자
  double coarse_deg = 1.0;     // [°]
  double band_px = 4.0;        // 안 · 밖 띠 거리 [화소]
  double lam = 2.0;            // 안쪽 색 불일치 벌점
  double beta = 2.0;           // 가장 약한 변 대비 가중
  int mask_grow = 3;           // 퍽 색 화소 부풀림 [화소]
  // 퍽 색 HSV (OpenCV H 0~179). perception.yaml 검출기 범위와 같게
  std::vector<std::array<int, 6>> puck_hsv{{0, 80, 35, 10, 255, 255}, {170, 80, 35, 179, 255, 255}, {40, 80, 35, 85, 255, 255}};
};

struct PlaceBoxResult
{
  bool ok = false;
  double x = 0.0, y = 0.0, yaw = 0.0;   // 평면 기준 프레임 [m, rad]
  double score = 0.0;
};

namespace place_box_detail
{
struct View
{
  cv::Mat lab;       // CV_32FC3
  cv::Mat valid;     // CV_8U
  cv::Mat mask;      // CV_8U 퍽 색
  cv::Mat mag;       // CV_32F 기울기 크기
  cv::Matx33d S;     // (u, v, 1) → (X, Y, 1)
  int n = 0;
};

// 정사각형 (중심 0, 각 th, 한 변 side) 변 4 개 위 k 점 (모서리 근처 10 % 뺌) 과 바깥 법선
inline void perimeter(double th, double side, int k, std::vector<cv::Point2d> & off, std::array<cv::Point2d, 4> & nrm)
{
  off.clear();
  const double c = std::cos(th), s = std::sin(th);
  const cv::Point2d ax(c, s), ay(-s, c);
  const std::array<std::pair<double, int>, 4> sides{{{1, 0}, {-1, 0}, {1, 1}, {-1, 1}}};
  for (int si = 0; si < 4; ++si) {
    const double sign = sides[si].first;
    const cv::Point2d a = sides[si].second == 0 ? ax : ay, b = sides[si].second == 0 ? ay : ax;
    nrm[si] = a * sign;
    const cv::Point2d mid = a * (sign * side / 2);
    for (int i = 0; i < k; ++i) {
      const double t = (-0.45 + 0.9 * i / (k - 1)) * side;
      off.push_back(mid + b * t);
    }
  }
}

inline double median(std::vector<double> & v)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  const size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  if (v.size() % 2) {return v[m];}
  const double hi = v[m];
  return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + m));
}

inline bool at(const View & V, const cv::Point2d & p, int & u, int & v)
{
  u = static_cast<int>(std::lround(p.x));
  v = static_cast<int>(std::lround(p.y));
  u = std::clamp(u, 0, V.n - 1);
  v = std::clamp(v, 0, V.n - 1);
  return V.valid.at<uchar>(v, u) != 0;
}

// 자리 잡기 점수 (중심 c, 각 th)
inline double side_score(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, int k)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  const double side = cfg.box / cfg.res;
  perimeter(th, side, k, off, nrm);
  std::array<double, 4> contrast{}, seen{};
  std::array<cv::Vec3d, 4> med_in{};
  std::array<bool, 4> has{};
  for (int si = 0; si < 4; ++si) {
    std::array<std::vector<double>, 3> din, ddf;
    int cnt = 0;
    for (int i = 0; i < k; ++i) {
      const cv::Point2d p = c + off[si * k + i];
      int ui, vi, uo, vo;
      const bool okin = at(V, p - nrm[si] * cfg.band_px, ui, vi), okout = at(V, p + nrm[si] * cfg.band_px, uo, vo);
      if (!okin || !okout || V.mask.at<uchar>(vi, ui) || V.mask.at<uchar>(vo, uo)) {continue;}
      const cv::Vec3f a = V.lab.at<cv::Vec3f>(vi, ui), b = V.lab.at<cv::Vec3f>(vo, uo);
      for (int ch = 0; ch < 3; ++ch) {din[ch].push_back(a[ch]); ddf[ch].push_back(a[ch] - b[ch]);}
      ++cnt;
    }
    seen[si] = static_cast<double>(cnt) / k;
    has[si] = cnt > 0;
    cv::Vec3d md;
    for (int ch = 0; ch < 3; ++ch) {med_in[si][ch] = median(din[ch]); md[ch] = median(ddf[ch]);}
    contrast[si] = (seen[si] >= 0.15 && cnt > 0) ? cv::norm(md) : 0.0;
  }
  cv::Vec3d ref;
  for (int ch = 0; ch < 3; ++ch) {
    std::vector<double> v;
    for (int si = 0; si < 4; ++si) {if (has[si]) {v.push_back(med_in[si][ch]);}}
    ref[ch] = v.empty() ? 0.0 : median(v);
  }
  double sc = 0.0, cmin = std::numeric_limits<double>::infinity();
  for (int si = 0; si < 4; ++si) {
    const double spread = (seen[si] >= 0.15 && has[si]) ? cv::norm(med_in[si] - ref) : 0.0;
    sc += contrast[si] - cfg.lam * spread - 50.0 * std::max(0.0, 0.3 - seen[si]);
    cmin = std::min(cmin, contrast[si]);
  }
  return sc + cfg.beta * cmin;
}

// 다듬기 점수: 테두리 위 기울기 크기 (퍽 색 · 화면 밖 뺌) 변 중앙값의 평균
inline double grad_score(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, int k)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  perimeter(th, cfg.box / cfg.res, k, off, nrm);
  double sum = 0.0;
  for (int si = 0; si < 4; ++si) {
    std::vector<double> v;
    for (int i = 0; i < k; ++i) {
      int u, w;
      if (at(V, c + off[si * k + i], u, w) && !V.mask.at<uchar>(w, u)) {v.push_back(V.mag.at<float>(w, u));}
    }
    const double m = median(v);
    sum += std::isfinite(m) ? m : 0.0;
  }
  return sum / 4.0;
}

template<typename F>
inline void local_search(F && f, cv::Point2d & c, double & th, double & best, double step_px, double step_rad, int r)
{
  cv::Point2d bc = c;
  double bt = th;
  for (int it = -r; it <= r; ++it) {
    const double t = th + it * step_rad;
    for (int iu = -r; iu <= r; ++iu) {
      for (int iv = -r; iv <= r; ++iv) {
        const cv::Point2d q(c.x + iu * step_px, c.y + iv * step_px);
        const double s = f(q, t);
        if (s > best) {best = s; bc = q; bt = t;}
      }
    }
  }
  c = bc;
  th = bt;
}
}  // namespace place_box_detail

// bgr: 왜곡 보정한 팔 영상, K: 그 내부값, R_pc · t_pc: 카메라 광학 좌표 → 평면 기준 프레임 (z 위, 예 base_footprint),
// prior: 사전 상자 중심 (평면 기준 x, y), prior_yaw: 사전 요 (이것 ±45° 안에서 고름)
inline PlaceBoxResult find_place_box(const cv::Mat & bgr, const cv::Matx33d & K, const cv::Matx33d & R_pc, const cv::Vec3d & t_pc,
  cv::Point2d prior, double prior_yaw, const PlaceBoxConfig & cfg = PlaceBoxConfig())
{
  using namespace place_box_detail;
  PlaceBoxResult out;
  if (bgr.empty() || bgr.type() != CV_8UC3) {return out;}
  View V;
  V.n = static_cast<int>(std::lround(cfg.span / cfg.res));
  // 평면 z = h 의 (X, Y) → 화소: K [r1 r2 r3·h + t], 여기서 [r | t] = 평면 → 카메라
  const cv::Matx33d Rt = R_pc.t();
  const cv::Vec3d tt = -(Rt * t_pc);
  cv::Matx33d M;
  for (int i = 0; i < 3; ++i) {M(i, 0) = Rt(i, 0); M(i, 1) = Rt(i, 1); M(i, 2) = Rt(i, 2) * cfg.height + tt[i];}
  // 정사영 화소 (u, v): 위쪽 = +X (앞), 왼쪽 = +Y
  V.S = cv::Matx33d(0, -cfg.res, prior.x + cfg.span / 2, -cfg.res, 0, prior.y + cfg.span / 2, 0, 0, 1);
  const cv::Matx33d Hm = K * M * V.S;
  cv::Mat tv;
  cv::warpPerspective(bgr, tv, cv::Mat(Hm), cv::Size(V.n, V.n), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  cv::warpPerspective(cv::Mat(bgr.size(), CV_8U, cv::Scalar(255)), V.valid, cv::Mat(Hm), cv::Size(V.n, V.n),
    cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  cv::Mat lab8, labf, hsv;
  cv::cvtColor(tv, lab8, cv::COLOR_BGR2Lab);
  lab8.convertTo(labf, CV_32FC3);
  cv::GaussianBlur(labf, V.lab, cv::Size(0, 0), 1.5);
  cv::cvtColor(tv, hsv, cv::COLOR_BGR2HSV);
  V.mask = cv::Mat::zeros(tv.size(), CV_8U);
  for (const auto & r : cfg.puck_hsv) {
    cv::Mat m;
    cv::inRange(hsv, cv::Scalar(r[0], r[1], r[2]), cv::Scalar(r[3], r[4], r[5]), m);
    V.mask |= m;
  }
  if (cfg.mask_grow > 0) {
    cv::dilate(V.mask, V.mask, cv::Mat::ones(2 * cfg.mask_grow + 1, 2 * cfg.mask_grow + 1, CV_8U));
  }
  {
    std::vector<cv::Mat> ch;
    cv::split(V.lab, ch);
    V.mag = cv::Mat::zeros(tv.size(), CV_32F);
    for (const auto & c : ch) {
      cv::Mat gx, gy;
      cv::Sobel(c, gx, CV_32F, 1, 0, 3);
      cv::Sobel(c, gy, CV_32F, 0, 1, 3);
      V.mag += gx.mul(gx) + gy.mul(gy);
    }
    cv::sqrt(V.mag, V.mag);
  }
  const double c0 = V.n / 2.0, side = cfg.box / cfg.res;
  // 거친 격자: 각마다 따로 (여러 스레드), 끝나면 각 순서대로 가장 좋은 것 — 한 스레드로 돈 것과 같은 답
  const int ng = static_cast<int>(std::floor(cfg.search / cfg.coarse_xy + 1e-9));
  const int na = static_cast<int>(std::ceil(90.0 / cfg.coarse_deg - 1e-9));
  std::vector<double> abest(na, -std::numeric_limits<double>::infinity());
  std::vector<cv::Point2d> apos(na, cv::Point2d(c0, c0));
  cv::parallel_for_(cv::Range(0, na), [&](const cv::Range & rg) {
    for (int ia = rg.start; ia < rg.end; ++ia) {
      const double th = (-45.0 + ia * cfg.coarse_deg) * CV_PI / 180.0 + prior_yaw;
      for (int iu = -ng; iu <= ng; ++iu) {
        for (int iv = -ng; iv <= ng; ++iv) {
          const cv::Point2d q(c0 + iu * cfg.coarse_xy / cfg.res, c0 + iv * cfg.coarse_xy / cfg.res);
          const double s = side_score(V, cfg, q, th, 24);
          if (s > abest[ia]) {abest[ia] = s; apos[ia] = q;}
        }
      }
    }
  });
  double best = -std::numeric_limits<double>::infinity(), bt = 0.0;
  cv::Point2d bc(c0, c0);
  for (int ia = 0; ia < na; ++ia) {
    if (abest[ia] > best) {best = abest[ia]; bc = apos[ia]; bt = (-45.0 + ia * cfg.coarse_deg) * CV_PI / 180.0 + prior_yaw;}
  }
  const double d2r = CV_PI / 180.0;
  auto fs = [&](cv::Point2d q, double t) {return side_score(V, cfg, q, t, 60);};
  local_search(fs, bc, bt, best, 0.001 / cfg.res, 0.25 * d2r, 4);
  local_search(fs, bc, bt, best, 0.0005 / cfg.res, 0.1 * d2r, 3);
  local_search(fs, bc, bt, best, 0.00025 / cfg.res, 0.05 * d2r, 2);
  out.score = best;
  auto gs = [&](cv::Point2d q, double t) {return grad_score(V, cfg, q, t, 80);};
  double gbest = -std::numeric_limits<double>::infinity();
  local_search(gs, bc, bt, gbest, 0.001 / cfg.res, 0.25 * d2r, 3);
  local_search(gs, bc, bt, gbest, 0.0005 / cfg.res, 0.1 * d2r, 3);
  local_search(gs, bc, bt, gbest, 0.00025 / cfg.res, 0.05 * d2r, 2);
  // 변마다 경계점 → 정사각형 가중 최소제곱 (Huber 1 화소). 정사각형을 dθ 돌리면 변 위 점의 법선 성분은 −t·dθ
  for (int it = 0; it < 3; ++it) {
    std::vector<cv::Point2d> off;
    std::array<cv::Point2d, 4> nrm;
    const int k = 80;
    perimeter(bt, side, k, off, nrm);
    std::vector<std::array<double, 3>> A;
    std::vector<double> y;
    for (int si = 0; si < 4; ++si) {
      const cv::Point2d tang(-nrm[si].y, nrm[si].x);
      for (int i = 0; i < k; ++i) {
        const cv::Point2d p0 = bc + off[si * k + i];
        const double ts = off[si * k + i].dot(tang);
        std::array<double, 11> prof;
        bool bad = false;
        double mx = -1.0;
        int j = 0;
        for (int dl = -5; dl <= 5; ++dl) {
          const cv::Point2d q = p0 + nrm[si] * dl;
          const int u = static_cast<int>(std::lround(q.x)), v = static_cast<int>(std::lround(q.y));
          if (u < 0 || v < 0 || u >= V.n || v >= V.n || !V.valid.at<uchar>(v, u) || V.mask.at<uchar>(v, u)) {bad = true; break;}
          prof[dl + 5] = V.mag.at<float>(v, u);
          if (prof[dl + 5] > mx) {mx = prof[dl + 5]; j = dl + 5;}
        }
        if (bad || mx < 5.0 || j == 0 || j == 10) {continue;}
        const double a = prof[j - 1], b = prof[j], c = prof[j + 1], den = a - 2 * b + c;
        const double dd = (j - 5) + (den != 0.0 ? 0.5 * (a - c) / den : 0.0);
        A.push_back({nrm[si].x, nrm[si].y, -ts});
        y.push_back(dd);
      }
    }
    if (A.size() < 40) {break;}
    std::vector<double> w(A.size(), 1.0);
    cv::Vec3d sol;
    for (int h = 0; h < 5; ++h) {
      cv::Matx33d AtA = cv::Matx33d::zeros();
      cv::Vec3d Aty(0, 0, 0);
      for (size_t i = 0; i < A.size(); ++i) {
        const double ww = w[i] * w[i];
        for (int r = 0; r < 3; ++r) {
          Aty[r] += ww * A[i][r] * y[i];
          for (int c = 0; c < 3; ++c) {AtA(r, c) += ww * A[i][r] * A[i][c];}
        }
      }
      if (!cv::solve(cv::Mat(AtA), cv::Mat(Aty), cv::Mat(sol, false), cv::DECOMP_SVD)) {break;}
      for (size_t i = 0; i < A.size(); ++i) {
        const double res = std::abs(y[i] - (A[i][0] * sol[0] + A[i][1] * sol[1] + A[i][2] * sol[2]));
        w[i] = res <= 1.0 ? 1.0 : std::sqrt(1.0 / std::max(res, 1e-6));
      }
    }
    if (std::max(std::abs(sol[0]), std::abs(sol[1])) > 3.0 || std::abs(sol[2]) > 2.0 * d2r) {break;}
    bc += cv::Point2d(sol[0], sol[1]);
    bt += sol[2];
  }
  const cv::Vec3d XY = V.S * cv::Vec3d(bc.x, bc.y, 1.0);
  out.x = XY[0];
  out.y = XY[1];
  // 영상 (u, v) 방향 (cos, sin) → 평면 (X, Y) = (−sin, −cos)  (u = −Y, v = −X)
  double yaw = std::atan2(-std::cos(bt), -std::sin(bt)) - prior_yaw;
  yaw = std::fmod(yaw + CV_PI / 4, CV_PI / 2);
  if (yaw < 0) {yaw += CV_PI / 2;}
  out.yaw = yaw - CV_PI / 4 + prior_yaw;
  out.ok = std::isfinite(out.score) && out.score > 0.0;
  return out;
}

// 칸 9 개 중심 (평면 기준, 행 우선: dx −1 → 1 바깥 반복, dy −1 → 1 안쪽 반복). pitch 기본 0.06 m
inline std::array<cv::Point2d, 9> place_cells(const PlaceBoxResult & r, double pitch = 0.06)
{
  std::array<cv::Point2d, 9> c;
  const double cy = std::cos(r.yaw), sy = std::sin(r.yaw);
  int i = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      c[i++] = cv::Point2d(r.x + (cy * dx - sy * dy) * pitch, r.y + (sy * dx + cy * dy) * pitch);
    }
  }
  return c;
}

}  // namespace perception
