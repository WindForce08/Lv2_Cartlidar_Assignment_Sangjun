// YOLO 엔진 출력 → 검출 (TensorRTDetector 가 씀, TensorRT 없이 시험할 수 있게 따로 둠)
//   출력 1×C×N(C = 4 + 클래스 수): 앞 4 줄 cx · cy · w · h(레터박스 입력 px), 나머지 클래스 점수. NMS 없는 모델
//   좌표는 실수 그대로 원본 화면으로 되돌리고(정수로 자르지 않음), NMS 는 클래스마다 따로 한다
//   (다른 클래스 퍽이 겹쳐 보여도 둘 다 남김). 근거: YOLO 후보 B 의 ONNX Runtime 평가와 같은 후처리
#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include <opencv2/dnn.hpp>

#include "perception/detector.hpp"

namespace perception
{

constexpr size_t kYoloMaxDetections = 300;   // 한 장의 최대 검출 수 (Ultralytics 기본 max_det)

// raw: 엔진 출력(채널 우선, channels × count), scale · pad_x · pad_y: 레터박스(입력 = 원본 × scale + pad),
// width · height: 원본 화면. 점수는 0~1 이 아니거나 유한하지 않으면 버린다
inline std::vector<Detection> decode_yolo(
  const std::vector<float> & raw, int channels, int count,
  float scale, int pad_x, int pad_y, int width, int height,
  float score_threshold, float nms_threshold, int target_class = -1)
{
  std::vector<Detection> out;
  if (channels < 5 || count <= 0 || raw.size() != static_cast<size_t>(channels) * count ||
    !std::isfinite(scale) || scale <= 0 || width <= 0 || height <= 0)
  {
    return out;
  }
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  std::map<int, std::vector<int>> by_class;   // 클래스 → boxes 번호
  for (int i = 0; i < count; ++i) {
    float best_score = 0.f;
    int best = -1;
    for (int c = 0; c < channels - 4; ++c) {
      const float v = raw[static_cast<size_t>(4 + c) * count + i];
      if (std::isfinite(v) && v <= 1.f && v > best_score) {best_score = v; best = c;}
    }
    if (best < 0 || best_score < score_threshold) {continue;}
    if (target_class >= 0 && best != target_class) {continue;}
    const float cx = raw[i], cy = raw[count + i], w = raw[2 * count + i], h = raw[3 * count + i];
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0) {
      continue;
    }
    by_class[best].push_back(static_cast<int>(boxes.size()));
    boxes.emplace_back((cx - w / 2 - pad_x) / scale, (cy - h / 2 - pad_y) / scale, w / scale, h / scale);
    scores.push_back(best_score);
    classes.push_back(best);
  }
  std::vector<int> keep;
  for (const auto & [cls, idx] : by_class) {
    (void)cls;
    std::vector<cv::Rect2d> cb;
    std::vector<float> cs;
    for (int k : idx) {cb.push_back(boxes[k]); cs.push_back(scores[k]);}
    std::vector<int> chosen;
    cv::dnn::NMSBoxes(cb, cs, score_threshold, nms_threshold, chosen);
    for (int k : chosen) {keep.push_back(idx[k]);}
  }
  std::stable_sort(keep.begin(), keep.end(), [&](int a, int b) {return scores[a] > scores[b];});
  if (keep.size() > kYoloMaxDetections) {keep.resize(kYoloMaxDetections);}
  for (int k : keep) {
    Detection d;
    d.box = cv::Rect2f(boxes[k]) & cv::Rect2f(0, 0, static_cast<float>(width), static_cast<float>(height));
    if (d.box.width <= 0 || d.box.height <= 0) {continue;}   // 화면 밖 상자
    d.score = scores[k];
    d.class_id = classes[k];
    out.push_back(d);
  }
  return out;
}

}  // namespace perception
