// 바코드 문자열 읽기 — 팔 카메라 (zbar, ROS 의존 없음 → 단위 시험)
//   기준 Python sim_render/proto_r3/zbar_check.py(sha256 앞 16자 ba6a17bac600cb5c, R3 설계 44차)와 같은 순서 · 설정:
//   1) 회색조 → 감마 LUT out = round(255·(g/255)^gamma) (기본 2: 막대 1.5~1.8 화소에서 흐려진 좁은 흰 칸을 살림)
//   2) scale 배 확대(1 = 원본, 기본. 1 이 아니면 INTER_CUBIC). 입력 해상도는 가정하지 않는다(팔 카메라는 640 × 480)
//   3) zbar: CODE128 만 켬, X · Y 훑기 간격 1(모든 열 · 행)
//   4) quality(그 문자열을 읽은 스캔 수) ≥ min_quality 인 기호만 씀 (기본 4: 체크섬이 상쇄되는 잘못 읽기 P2-0x → PF-&x 는
//      몇 스캔에서만 나옴 — 설계 자료에서 감마 2 최대 2, 맞는 문자열 최소 34)
//   한 장에서 서로 다른 문자열이 둘 이상이면 어느 것이 맞는지 모르므로 '못 읽음'
//   (판정 1차 틀린 문자열 1건이 ['PF-&3', 'P2-03'] 두 개였음). 문자열 형식 검사는 하지 않는다.
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct BarcodeRead
{
  bool ok = false;                  // 서로 다른 문자열이 정확히 하나
  std::string text;                 // ok 일 때
  std::vector<std::string> symbols; // quality 거르기를 통과한 기호 문자열 (zbar 가 낸 순서)
  int dropped = 0;                  // quality 가 모자라 뺀 기호 수
};

// 한 장의 기호 문자열들 → 결정 (Py: decide)
BarcodeRead decide_barcode(const std::vector<std::string> & symbols);

class BarcodeReader
{
public:
  explicit BarcodeReader(double scale = 1.0, double gamma = 2.0, int min_quality = 4, int x_density = 1, int y_density = 1);
  ~BarcodeReader();
  BarcodeReader(const BarcodeReader &) = delete;
  BarcodeReader & operator=(const BarcodeReader &) = delete;

  // img: 회색조(CV_8U) 또는 BGR(CV_8UC3, 회색조로 바꿈). 한 스레드에서만 부른다
  BarcodeRead read(const cv::Mat & img);

private:
  double scale_ = 1.0;
  int min_quality_ = 4;
  cv::Mat lut_;                     // 감마 (1 × 256, CV_8U). 감마 1 이면 비움
  void * scanner_ = nullptr;        // zbar_image_scanner_t
};

}  // namespace perception
