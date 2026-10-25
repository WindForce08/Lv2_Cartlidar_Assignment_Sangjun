#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"

namespace perception
{

std::vector<Detection> ColorDetector::detect(const cv::Mat & bgr)
{
  std::vector<Detection> out;
  if (bgr.empty()) {return out;}
  // 블러는 새 버퍼에 한다. 예전에는 src 가 bgr 와 버퍼를 같이 써서 입력 영상을 제자리에서 흐리게 했다
  // → 노드가 같은 영상으로 이어서 찾는 태그 · 장소 태그 · 깊이 나누기가 흐린 영상을 받았다(2026-10-25 발견)
  cv::Mat src = bgr, hsv;
  if (cfg_.blur_ksize > 1) {
    cv::Mat blurred;
    cv::GaussianBlur(bgr, blurred, cv::Size(cfg_.blur_ksize | 1, cfg_.blur_ksize | 1), 0);
    src = blurred;
  }
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  const double img_area = static_cast<double>(bgr.cols) * bgr.rows;
  const int k = std::max(1, cfg_.morph_kernel);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
  for (int cls = 0; cls < 3; ++cls) {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U), m;
    bool any = false;
    for (const auto & h : cfg_.hsv) {   // 같은 색의 범위는 합침 (빨강 두 범위)
      if (h.cls != cls || (cfg_.target_color != "any" && cfg_.target_color != h.color)) {continue;}
      cv::inRange(hsv, h.lower, h.upper, m);
      mask |= m;
      any = true;
    }
    if (!any) {continue;}
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto & c : contours) {
      const double area = cv::contourArea(c);
      if (area < cfg_.min_area_ratio * img_area) {continue;}
      const cv::Rect r = cv::boundingRect(c);
      // 채움률은 실제 마스크 픽셀로 셈 (외곽 컨투어 넓이는 속 빈 테두리도 1 로 나옴). 사각 · 원 퍽은 0.75~1
      const double fill = cv::countNonZero(mask(r)) / std::max(1.0, static_cast<double>(r.area()));
      if (fill < cfg_.min_fill_ratio) {continue;}
      const double aspect = static_cast<double>(std::min(r.width, r.height)) / std::max(r.width, r.height);
      Detection d;
      d.box = cv::Rect2f(r);
      d.class_id = cls;
      d.score = static_cast<float>(std::clamp(0.5 * fill + 0.5 * aspect, 0.0, 1.0));
      if (d.score >= cfg_.score_threshold) {out.push_back(d);}
    }
  }
  return out;
}

int best_detection(const std::vector<Detection> & d, int target_class)
{
  int best = -1;
  for (int i = 0; i < static_cast<int>(d.size()); ++i) {
    if (target_class >= 0 && d[i].class_id != target_class) {continue;}
    if (best < 0 || d[i].score > d[best].score) {best = i;}
  }
  return best;
}

#ifdef USE_TENSORRT
std::unique_ptr<Detector> make_tensorrt_detector(const DetectorConfig & c, std::string & info);
#endif

std::unique_ptr<Detector> make_detector(const DetectorConfig & c, std::string & info)
{
  if (c.type == "tensorrt" || c.type == "auto") {
#ifdef USE_TENSORRT
    if (!c.engine_path.empty()) {
      auto d = make_tensorrt_detector(c, info);
      if (d) {return d;}
    } else {
      info = "engine_path 없음";
    }
#else
    info = "TensorRT 없이 빌드됨";
#endif
    if (c.type == "tensorrt") {return nullptr;}
    info += " → ColorDetector 사용";
  }
  return std::make_unique<ColorDetector>(c);
}

}  // namespace perception
