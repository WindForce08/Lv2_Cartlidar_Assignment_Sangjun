// 물체 검출기 인터페이스 (perception_node)
//   TensorRTDetector : YOLOv8 형식 엔진(.engine), TensorRT 가 있을 때만 빌드 (USE_TENSORRT)
//   ColorDetector    : HSV 색 영역 (퍽 R · G · B) — 엔진이 없을 때 대체, 조명 보정 전 시험용
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct Detection
{
  cv::Rect2f box;     // 픽셀
  float score = 0.f;
  int class_id = -1;  // ColorDetector: 0 red · 1 green · 2 blue
};

// HSV 범위 하나 (OpenCV H 0~179). 빨강은 H 가 0 근처와 179 근처로 나뉘어 두 개
struct HsvRange
{
  int cls = 0;                 // 0 red · 1 green · 2 blue
  std::string color;
  cv::Scalar lower, upper;
};

// 기본 범위 (근거는 config/perception.yaml detector.hsv 주석)
inline std::vector<HsvRange> default_hsv()
{
  return {
    {0, "red", cv::Scalar(0, 80, 35), cv::Scalar(10, 255, 255)},
    {0, "red", cv::Scalar(170, 80, 35), cv::Scalar(179, 255, 255)},
    {1, "green", cv::Scalar(40, 80, 35), cv::Scalar(85, 255, 255)},
    {2, "blue", cv::Scalar(95, 80, 25), cv::Scalar(120, 255, 255)},
  };
}

struct DetectorConfig
{
  std::string type = "auto";          // auto | tensorrt | color
  std::string engine_path;
  int input_size = 640;
  float score_threshold = 0.5f;
  float nms_threshold = 0.45f;
  int target_class = -1;              // -1 = 모든 클래스
  std::string target_color = "any";   // any | red | green | blue (ColorDetector)
  double min_area_ratio = 0.002;      // 화면 넓이 대비 최소 영역
  std::vector<HsvRange> hsv = default_hsv();
  int blur_ksize = 5;                 // 0 = 끔, 홀수
  int morph_kernel = 5;               // open · close 타원 커널 (px)
  double min_fill_ratio = 0.5;        // 컨투어 넓이 / bbox 넓이 — 낮으면 무늬 · 조각 (Xmen min_fill)
};

class Detector
{
public:
  virtual ~Detector() = default;
  virtual std::vector<Detection> detect(const cv::Mat & bgr) = 0;
  virtual std::string name() const = 0;
};

class ColorDetector : public Detector
{
public:
  explicit ColorDetector(const DetectorConfig & c)
  : cfg_(c) {}
  std::vector<Detection> detect(const cv::Mat & bgr) override;
  std::string name() const override {return "color(" + cfg_.target_color + ")";}

private:
  DetectorConfig cfg_;
};

// 설정에 맞는 검출기. tensorrt 를 원했는데 못 만들면 nullptr, auto 면 color 로 대체. info 에 사유.
std::unique_ptr<Detector> make_detector(const DetectorConfig & c, std::string & info);

// 점수가 가장 높은 검출 (target_class 필터). 없으면 -1
int best_detection(const std::vector<Detection> & d, int target_class);

}  // namespace perception
