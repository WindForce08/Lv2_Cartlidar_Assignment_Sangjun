// 물체 검출기 인터페이스 (perception_node)
//   TensorRTDetector : YOLOv8 형식 엔진(.engine), TensorRT 가 있을 때만 빌드 (USE_TENSORRT)
//   ColorDetector    : HSV 색 영역 (퍽 R · G · B) — 엔진이 없을 때 대체, 조명 보정 전 시험용.
//                      label_aware 면 위에서 본 퍽의 흰 라벨을 써서 조각 묶기 · 나누기 (팔 카메라)
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
  double tracking_size = 0;  // 색 검출의 최소회전사각형 sqrt(넓이), px. 0이면 기존 폭 비교
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
  // 라벨 인식 (팔 카메라로 위에서 본 퍽, ColorDetector). 끝면 가운데 흰 라벨(태그 16 mm + 여백 = 20 mm)이 색 덩어리의
  // 구멍이 되고, 원기둥(지름 30 mm)은 색이 라벨 네 귀퉁이 틈에만 남아 조각난다. 그래서
  //   태그(어둡고 채도 낮은 정사각형) → 라벨 = 태그 × label_scale → 라벨 바로 바깥 띠의 색으로 라벨 색을 정함 →
  //   그 색 마스크에 라벨을 더해 조각을 묶음 → 덩어리 구멍 채움 → 라벨 2개 이상이면 가까운 라벨로 나눔(맞닿은 같은 색 퍽) →
  //   채움 · 가로세로비는 회전 외접 사각형으로 (화면에서 돈 직육면체)
  bool label_aware = false;
  bool label_merge_same_id = false;  // 팔: 해독된 같은 퍽 ID의 여러 면은 한 물체
  int label_dark_v_max = 90;          // 태그 검은 칸: V 이하 · S 이하
  int label_dark_s_max = 90;
  int label_dark_window = 31;       // 주변 평균보다 어두운 태그만 (0=끔)
  double label_dark_c = 20.0;
  int label_body_v_min = 15;        // 팔 라벨 검출의 그림자 몸체만, 일반 D435 HSV는 유지
  int label_tag_close = 5;            // [px] 태그 무늬를 한 덩어리로 닫는 커널
  double label_tag_min_px = 30;       // [px²] 태그 덩어리 최소 넓이
  double label_square_min = 0.5;      // 태그 회전 사각형의 짧은 변 / 긴 변, 넓이 / 사각형 넓이 하한
  double label_scale = 1.25;          // 라벨 한 변 / 태그 한 변 (20 / 16 mm)
  double label_ring_frac = 0.2;       // 라벨 색을 보는 바깥 띠 폭 / 라벨 한 변
  double label_ring_min = 0.25;       // 띠에서 그 색 화소 비율 하한 (못 넘으면 퍽 라벨로 안 봄)
};

class Detector
{
public:
  virtual ~Detector() = default;
  virtual std::vector<Detection> detect(const cv::Mat & bgr) = 0;
  virtual std::string name() const = 0;
  virtual int num_classes() const {return -1;}   // 엔진 출력 클래스 수 (-1 = 정해진 번호, ColorDetector)
};

class ColorDetector : public Detector
{
public:
  explicit ColorDetector(const DetectorConfig & c)
  : cfg_(c) {}
  std::vector<Detection> detect(const cv::Mat & bgr) override;
  std::vector<Detection> detect_with_reference(const cv::Mat & bgr, const cv::Mat & reference);
  std::string name() const override {return std::string(cfg_.label_aware ? "color+label(" : "color(") + cfg_.target_color + ")";}

private:
  std::vector<Detection> detect_labeled(const cv::Mat & hsv, const cv::Mat & kernel, const cv::Mat & reference);
  DetectorConfig cfg_;
};

// 설정에 맞는 검출기. tensorrt 를 원했는데 못 만들면 nullptr, auto 면 color 로 대체. info 에 사유.
std::unique_ptr<Detector> make_detector(const DetectorConfig & c, std::string & info);

// 점수가 가장 높은 검출 (target_class 필터). 없으면 -1
int best_detection(const std::vector<Detection> & d, int target_class);

}  // namespace perception
