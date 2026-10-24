// 색 검출 덩어리를 깊이로 나누기 (ROS 의존 없음 → 단위 시험)
//   색 검출기는 같은 색 화소가 이어진 곳을 한 덩어리로 낸다. 선반 위 같은 색 퍽 2~4개가 화면에서 겹치거나 맞닿으면 한 덩어리가 되어
//   크기 상한에 걸려 버려지거나 중심이 퍽 사이 틈에 떨어진다(R4 렌더 2026-10-25: 선반 퍽 1.0 m 이내 284개 중 134개를 놓침, 모두 이 경우).
//   퍽끼리는 떨어져 있어 사이에 깊이가 끊기므로, 상자 안 같은 색 화소를 '이웃 화소 깊이 차 < tol_m' 로만 이어 조각마다 퍽 하나로 본다.
//   조각 중심 = 조각 화소의 평균 위치, 깊이 = 조각 깊이 중앙값. 색 검출기 전용(YOLO 는 퍽마다 상자를 따로 냄)
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"
#include "perception/geometry.hpp"

namespace perception
{

struct DepthSplitConfig
{
  double tol_m = 0.01;          // 이웃 화소 깊이 차가 이보다 작아야 한 조각
  double min_face_ratio = 0.3;  // 조각 화소 수 하한: 그 깊이에서 face_m × face_m 면의 화소 수에 곱하는 비율
  double face_m = 0.03;         // 퍽 짧은 변
  double min_fill = 0.35;       // 조각 화소 수 / 외접 상자 넓이 하한(검출기 min_fill_ratio 와 같게). 블러로 덩어리 둘레에 생긴
                                // 1 px 테두리는 배경 깊이로 따로 이어지는데, 채움이 작아 여기서 빠진다
  double min_m = 0.15, max_m = 4.0;
};

struct DepthPiece
{
  Detection det;       // box = 조각 외접 상자, score = 조각의 0.5·채움 + 0.5·가로세로비 (색 검출기와 같은 식)
  cv::Point center;    // 조각 화소 평균 위치
  double depth = 0.0;  // 조각 깊이 중앙값 [m]
};

// bgr: 컬러 영상, depth: 컬러에 맞춘 깊이(CV_16U 또는 CV_32F, scale 곱해 m), hsv: 검출기 HSV 범위(cls 별),
// blur_ksize · morph_kernel: 검출기와 같은 값. 상자가 겹쳐 같은 조각이 두 번 나오면(중심 2 px 안) 한 번만 낸다
inline std::vector<DepthPiece> split_by_depth(const std::vector<Detection> & dets, const cv::Mat & bgr, const cv::Mat & depth,
  double scale, const std::vector<HsvRange> & hsv, int blur_ksize, int morph_kernel, const PinholeK & K,
  const DepthSplitConfig & cfg, float score_threshold)
{
  std::vector<DepthPiece> out;
  const bool f32 = depth.type() == CV_32F;
  if (bgr.empty() || depth.size() != bgr.size() || (!f32 && depth.type() != CV_16U)) {return out;}
  const cv::Rect img(0, 0, bgr.cols, bgr.rows);
  for (const auto & d : dets) {
    const cv::Rect r = cv::Rect(d.box) & img;
    if (r.area() <= 0) {continue;}
    cv::Mat roi = bgr(r).clone(), hsv_roi;   // 원본 영상을 건드리지 않게 복사
    if (blur_ksize > 1) {cv::GaussianBlur(roi, roi, cv::Size(blur_ksize | 1, blur_ksize | 1), 0);}
    cv::cvtColor(roi, hsv_roi, cv::COLOR_BGR2HSV);
    cv::Mat mask = cv::Mat::zeros(r.size(), CV_8U), m;
    for (const auto & h : hsv) {
      if (h.cls != d.class_id) {continue;}
      cv::inRange(hsv_roi, h.lower, h.upper, m);
      mask |= m;
    }
    const int k = std::max(1, morph_kernel);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::Mat z(r.size(), CV_64F, cv::Scalar(0.0));
    for (int y = 0; y < r.height; ++y) {
      for (int x = 0; x < r.width; ++x) {
        const double v = f32 ? depth.at<float>(r.y + y, r.x + x) * scale : depth.at<uint16_t>(r.y + y, r.x + x) * scale;
        if (mask.at<uint8_t>(y, x) && std::isfinite(v) && v >= cfg.min_m && v <= cfg.max_m) {z.at<double>(y, x) = v;}
      }
    }
    cv::Mat label(r.size(), CV_32S, cv::Scalar(-1));
    std::vector<cv::Point> stack, pix;
    int n = 0;
    for (int y0 = 0; y0 < r.height; ++y0) {
      for (int x0 = 0; x0 < r.width; ++x0) {
        if (z.at<double>(y0, x0) <= 0.0 || label.at<int>(y0, x0) >= 0) {continue;}
        stack.assign(1, {x0, y0});
        label.at<int>(y0, x0) = n;
        pix.clear();
        while (!stack.empty()) {
          const cv::Point p = stack.back();
          stack.pop_back();
          pix.push_back(p);
          const double zp = z.at<double>(p);
          for (const cv::Point dp : {cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1)}) {
            const cv::Point q = p + dp;
            if (q.x < 0 || q.y < 0 || q.x >= r.width || q.y >= r.height || label.at<int>(q) >= 0) {continue;}
            const double zq = z.at<double>(q);
            if (zq > 0.0 && std::fabs(zq - zp) < cfg.tol_m) {label.at<int>(q) = n; stack.push_back(q);}
          }
        }
        ++n;
        std::vector<double> zs;
        zs.reserve(pix.size());
        double su = 0.0, sv = 0.0;
        for (const auto & p : pix) {zs.push_back(z.at<double>(p)); su += p.x; sv += p.y;}
        std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
        const double zm = zs[zs.size() / 2];
        const double face_px = (K.fx * cfg.face_m / zm) * (K.fy * cfg.face_m / zm);
        if (static_cast<double>(pix.size()) < cfg.min_face_ratio * face_px) {continue;}
        const cv::Rect b = cv::boundingRect(pix);
        const double fill = static_cast<double>(pix.size()) / b.area();
        if (fill < cfg.min_fill) {continue;}
        const double aspect = static_cast<double>(std::min(b.width, b.height)) / std::max(b.width, b.height);
        DepthPiece pc;
        pc.det.class_id = d.class_id;
        pc.det.box = cv::Rect2f(b + r.tl());
        pc.det.score = static_cast<float>(std::clamp(0.5 * fill + 0.5 * aspect, 0.0, 1.0));
        if (pc.det.score < score_threshold) {continue;}
        pc.center = cv::Point(static_cast<int>(std::lround(su / pix.size())) + r.x, static_cast<int>(std::lround(sv / pix.size())) + r.y);
        pc.depth = zm;
        const bool dup = std::any_of(out.begin(), out.end(), [&](const DepthPiece & o) {
            return o.det.class_id == pc.det.class_id && std::abs(o.center.x - pc.center.x) <= 2 &&
            std::abs(o.center.y - pc.center.y) <= 2;});
        if (!dup) {out.push_back(pc);}
      }
    }
  }
  return out;
}

}  // namespace perception
