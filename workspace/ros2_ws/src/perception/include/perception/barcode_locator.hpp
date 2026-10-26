// 바코드 라벨 찾기 — 앞 D435 컬러 + 깊이 (ROS 의존 없음 → 단위 시험 · Python 비교 도구)
//   R3: 바코드 라벨(Code128, 35 × 20 mm, 흰 테두리 · 막대 띠 · 아래 글자 줄)은 벽판 또는 선반 윗면에 붙는다.
//   D435 로 라벨 자리를 찾고, 팔 카메라로 가까이 가서 읽는다(barcode_reader.hpp).
//   기준 Python 시제품(sim_render/proto_r3/locate_v2.py, sha256 앞 16자 21f7ddbd7ec1fdd5, R3 설계 45차)을 같은 순서로 옮겼다.
//   설계 근거 · 결과는 sim_render/README.md 'R3' 절, 문턱 기본값은 그 CFG2 값이다.
//
//   1) surfaces: 깊이 → 3D 점 → 상자 평균 뒤 화소마다 법선(base_footprint 기준)
//      → 법선이 수평이면 벽, 수직이면 윗면 후보. 법선이 꺾이거나 깊이가 끊긴 화소는 빼고 이어진 덩어리 = 평면
//      → 윗면은 작은 창에서 '높이 퍼짐 / 수평 퍼짐' 이 작은 화소도 더함(좁은 틈은 법선 창에 옆 면이 섞여 법선이 틀어짐)
//      → 평면 하나로 맞지 않는 덩어리(잡음으로 모서리 너머 면과 이어짐)는 RANSAC 으로 평면 하나씩 떼어 냄
//   2) 평면마다 정면으로 편 영상(mm 격자)을 만들고, 화소 하나가 평면에서 덮는 넓이만큼 템플릿을 흐려
//      라벨 그림을 rot_step 마다 돌려 정규 상관(마스크 matchTemplate). 깊이가 평면 위인 칸만 본다
//   3) 봉우리마다 모양 거르기(채도 · 막대 띠/흰 테두리 · 흰 테두리/둘레 · 둘레 고름 · 선명도 · 한 변 어두움 · 영상 안)
//   4) 점수 순으로 nms 안 겹침 제거
//   깊이는 로봇이 멈춰서 여러 장 평균한 것을 넣는다(노드 barcode.depth_frames, 설계 전제).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

// 기본값 = locate_v2.py CFG2 (45차). 길이 [m], 각도 [°], 밝기 · 채도 0~255, 'px' 는 편 영상 칸
struct BarcodeLocatorConfig
{
  // 평면 만들기
  double mm = 1.5;               // 편 영상 한 칸 [mm]
  double rng = 1.2;              // base_footprint 수평 거리 상한 [m]
  double min_z = 0.06;           // base_footprint 높이 하한 [m] (바닥 빼기)
  double nz_wall = 0.25;         // 벽: |법선 z| ≤ 이 값
  double nz_top = 0.9;           // 윗면: 법선 z ≥ 이 값
  double top_h = 0.2;            // 윗면 더하기: top_hk 창의 sqrt(높이 분산 / 수평 분산) ≤ 이 값 (0 = 끔)
  int top_hk = 5;                //   창 크기 [화소] (깊이 점프 둘레도 이만큼 뺌)
  double edge_deg = 12.0;        // 이웃(nk 칸) 법선 각 차가 이보다 크면 경계
  double edge_dz_ratio = 0.02;   // 이웃 깊이 차 > 이 값 × max(z, edge_dz_floor) 이면 경계
  double edge_dz_floor = 0.2;
  int smooth = 15;               // 법선용 상자 평균 크기 [화소]
  int nk = 3;                    // 법선 차분 간격 [화소]
  int median = 5;                // 깊이 중앙값 거르기 크기 (0 = 끔, OpenCV float 는 3 · 5 만)
  double depth_min = 0.15, depth_max = 4.0;   // 쓰는 깊이 [m]
  int min_comp = 150;            // 평면 덩어리 화소 수 하한
  bool split = true;             // 평면 하나로 맞지 않는 덩어리를 RANSAC 으로 나눔
  double split_tol = 0.008;      // RANSAC 평면 거리 허용 [m] (rms ≤ split_tol / 2 이면 나누지 않음)
  int split_iter = 300;
  int constrain = 0;             // 1: 윗면 법선 = base z, 벽 법선 수평으로 고정 · 2: 윗면만 · 0: 끔
  double plane_rms = 0.004;      // 평면 rms > plane_rms + 2 · 잰 잡음 이면 평면 아님 [m]
  double wall_min_top = 0.20;    // 벽 평면 높이(98 %) 하한 [m]: 상자 옆면 빼기
  double wall_min_w = 0.06;      // 벽 평면 가로 폭(1~99 %) 하한 [m]: 미로 기둥 옆면 빼기
  double rect_margin = 0.06;     // 평면 둘레로 더 펴는 폭 [m]
  double max_cells = 4e6;        // 편 영상 칸 수 상한
  // 깊이로 평면 위인지
  double depth_tol = 0.006;      // [m]
  double tol_k = 3.0;            // 허용치에 더하는 잰 잡음 배수 (거리 제곱 비례)
  bool tol_inc = true;           // 허용치를 1 / cos(입사각) 배 (최소 cos 0.2)
  // 템플릿
  double label_w_mm = 35.0, label_h_mm = 20.0;
  double rot_step = 7.5;         // [°], 한 바퀴 다 (글자 줄 때문에 위아래가 다름)
  bool use_text = true;          // false: 글자 줄(위 15.5 mm 아래)을 마스크에서 뺌
  double text_cut_mm = 15.5;
  int tpl_white = 230;           // 템플릿에서 흰 테두리로 보는 밝기 하한
  int tpl_dark = 180;            // 막대 띠로 보는 밝기 상한
  double psf_px = 0.9;           // 카메라 흐림 σ [카메라 화소]
  double max_blur = 20.0;        // 흐림 커널 반경(3σ) 상한 [편 영상 칸]: 넘으면 그 평면을 건너뜀(시선과 나란한 면, 메모리 폭주 방지)
  bool local_psf = false;        // 후보 자리에서 흐림을 다시 구해 점수를 다시 잼
  int win_erode = 1;             // 라벨 창이 평면 경계에서 이만큼 안쪽 [칸]
  double valid_frac = 0.90;      // 라벨 창 안 평면 위 칸 비율 하한
  double min_std = 4.0;          // 라벨 창 밝기 표준편차 하한
  double thr = 0.65;             // 정규 상관 문턱
  double rescore_m = 0.05;       // thr − rescore_m 이상 봉우리는 평면 위 칸만으로 상관을 다시 재어 thr 과 견줌(조금 가린 라벨)
  double nms = 0.03;             // 봉우리 · 겹침 제거 거리 [m]
  // 모양 거르기 (적힌 순서대로 검사)
  double lift_max = 9.0;         // 라벨 창이 평면에서 떠 있는 높이 중앙값 상한 [m] (9 = 사실상 끔)
  double sat_max = 40.0;         // 라벨 창 평균 채도 상한
  double sat_hi_max = 999.0;     // 라벨 안쪽 채도 90 % 상한 (999 = 끔)
  double bw_min = 0.4;           // 막대 띠 평균 ≥ bw_min × 흰 테두리 평균
  double white_ring_min = 0.98;  // 흰 테두리 평균 ≥ 이 값 × 둘레 평균
  double bw_max = 9.0;           // 막대 띠 평균 ≤ bw_max × 흰 테두리 평균 (9 = 끔)
  double con_max = 1.08;         // 무늬 명암비 fit_a · 255 / 흰 테두리 상한 (비스듬한 장소 태그 빼기, 99 = 끔)
  double ring_in = 4.0, ring_out = 10.0;   // 둘레 띠 [mm, 라벨 가장자리에서]
  bool ring_mad = false;         // 둘레 퍼짐을 중앙값 기반으로
  bool ring_drop_worst = true;   // 가장 얼룩진 한 변을 빼고 둘레 퍼짐
  bool ring_lin = true;          // 둘레 밝기 기울기(1 차 평면)를 빼고 퍼짐
  double side_min_frac = 0.3;    // 둘레 한 변의 평면 위 칸 / 그 변 전체 칸 이 이보다 작으면 그 변은 빼고 봄 (0 = 끔)
  double ring_max = 0.35;        // 둘레 퍼짐 / (흰 테두리 − 막대 띠) 상한
  double ring_rel_max = 0.06;    // 둘레 퍼짐 / 흰 테두리 상한
  double resw_min = -99.0;       // 흰 테두리 잔차 2 % 하한 (−99 = 끔)
  double sharp_min = -0.05;      // 선명도(흐린 템플릿 상관 − 두 배 흐린 템플릿 상관) 하한
  double side_min = -0.45;       // 흰 테두리 가장 어두운 변의 평균 잔차 하한
  int img_margin = 4;            // 라벨 네 모서리가 영상 끝에서 이만큼 안 [화소]
  double ring_min_valid = 0.0;   // 둘레 띠 중 평면 위 비율 하한
};

// 설정 이름 ↔ 값. 노드 파라미터 barcode.<이름> 과 비교 도구의 키=값 이 이 목록 하나를 쓴다
//   v(이름, double& | int& | bool&) 로 부른다
template<class V>
void visit_barcode_params(BarcodeLocatorConfig & c, V && v)
{
  v("mm", c.mm); v("rng", c.rng); v("min_z", c.min_z); v("nz_wall", c.nz_wall); v("nz_top", c.nz_top);
  v("top_h", c.top_h); v("top_hk", c.top_hk); v("rescore_m", c.rescore_m); v("side_min_frac", c.side_min_frac);
  v("edge_deg", c.edge_deg); v("edge_dz_ratio", c.edge_dz_ratio); v("edge_dz_floor", c.edge_dz_floor);
  v("smooth", c.smooth); v("nk", c.nk); v("median", c.median); v("depth_min", c.depth_min); v("depth_max", c.depth_max);
  v("min_comp", c.min_comp); v("split", c.split); v("split_tol", c.split_tol); v("split_iter", c.split_iter);
  v("constrain", c.constrain); v("plane_rms", c.plane_rms); v("wall_min_top", c.wall_min_top); v("wall_min_w", c.wall_min_w);
  v("rect_margin", c.rect_margin); v("max_cells", c.max_cells); v("depth_tol", c.depth_tol); v("tol_k", c.tol_k);
  v("tol_inc", c.tol_inc); v("label_w_mm", c.label_w_mm); v("label_h_mm", c.label_h_mm); v("rot_step", c.rot_step);
  v("use_text", c.use_text); v("text_cut_mm", c.text_cut_mm); v("tpl_white", c.tpl_white); v("tpl_dark", c.tpl_dark);
  v("psf_px", c.psf_px); v("max_blur", c.max_blur); v("local_psf", c.local_psf); v("win_erode", c.win_erode);
  v("valid_frac", c.valid_frac); v("min_std", c.min_std); v("thr", c.thr); v("nms", c.nms); v("lift_max", c.lift_max);
  v("sat_max", c.sat_max); v("sat_hi_max", c.sat_hi_max); v("bw_min", c.bw_min); v("white_ring_min", c.white_ring_min);
  v("bw_max", c.bw_max); v("con_max", c.con_max); v("ring_in", c.ring_in); v("ring_out", c.ring_out); v("ring_mad", c.ring_mad);
  v("ring_drop_worst", c.ring_drop_worst); v("ring_lin", c.ring_lin); v("ring_max", c.ring_max);
  v("ring_rel_max", c.ring_rel_max); v("resw_min", c.resw_min); v("sharp_min", c.sharp_min); v("side_min", c.side_min);
  v("img_margin", c.img_margin); v("ring_min_valid", c.ring_min_valid);
}

// 카메라 내부 값 (컬러 광학 좌표, 왜곡 없음)
struct BarcodeK
{
  double fx = 0, fy = 0, cx = 0, cy = 0;
};

struct BarcodeDetection
{
  enum Surface : uint8_t {WALL = 0, TOP = 1};
  Surface surface = WALL;
  cv::Vec3d center;              // 라벨 가운데 (카메라 광학 좌표) [m]
  cv::Vec3d normal;              // 평면 법선 (카메라 쪽, 카메라 광학 좌표)
  double score = 0;              // 정규 상관
  double theta = 0;              // 템플릿 회전 [°]
  cv::Point2d px;                // 영상 화소
  // 기록용 (Python 출력과 같은 이름)
  double sat = 0, ring_c = 0, ring_std = 0, ring_mean = 0, ring_valid = 0, white_mean = 0, band_mean = 0, sat_hi = 0;
  double lift = 0, pl_w = 0, resw_lo = 0, side_lo = 0, sharp = 0, fit_a = 0, con = 0;
};

// 평면 하나 (기록 · 시험용)
struct BarcodePlane
{
  BarcodeDetection::Surface kind = BarcodeDetection::WALL;
  cv::Vec3f c, n;                // 중심 · 법선 (카메라 광학 좌표, float32 = Python 과 같음)
  double rms = 0, sig = 0, zmed = 0, ztop = 0;
  std::vector<int> ys, xs;       // 평면 화소 (행 순서)
};

class BarcodeLocator
{
public:
  // tex_path: 라벨 그림(회색조, 흰 테두리 포함 35 × 20 mm 전체). 읽지 못하면 ok() = false
  BarcodeLocator(const BarcodeLocatorConfig & cfg, const std::string & tex_path);
  BarcodeLocator(const BarcodeLocatorConfig & cfg, const cv::Mat & tex_gray);
  bool ok() const {return !tpls_.empty();}
  const BarcodeLocatorConfig & config() const {return cfg_;}

  // bgr: 컬러 (CV_8UC3), depth: 컬러에 맞춘 깊이 [m] (CV_32F, 없는 값 0 또는 NaN),
  // T_base_cam: base_footprint ← 컬러 광학 좌표 (4 × 4, CV_64F 또는 Matx44d). 결과는 점수 높은 순
  std::vector<BarcodeDetection> locate(const cv::Mat & bgr, const cv::Mat & depth, const BarcodeK & K,
    const cv::Matx44d & T_base_cam) const;

  // 1) 평면만 (시험 · 비교용). P: 3D 점 (CV_32FC3), zf: 거른 깊이 (CV_32F)
  std::vector<BarcodePlane> surfaces(const cv::Mat & depth, const BarcodeK & K, const cv::Matx44d & T_base_cam,
    cv::Mat * P = nullptr, cv::Mat * zf = nullptr) const;

  struct Template
  {
    double theta;                // [°]
    cv::Mat tp, mk;              // 돌린 그림 · 마스크 (CV_32F, D × D)
    cv::Mat bright, dark;        // 흰 테두리 · 막대 띠 자리 (CV_8U 0/1)
  };
  const std::vector<Template> & templates() const {return tpls_;}

private:
  void build_templates(const cv::Mat & tex_gray);
  void split_component(const cv::Mat & P, std::vector<int> ys, std::vector<int> xs, int depth,
    std::vector<std::pair<std::vector<int>, std::vector<int>>> & out) const;

  BarcodeLocatorConfig cfg_;
  std::vector<Template> tpls_;
};

}  // namespace perception
