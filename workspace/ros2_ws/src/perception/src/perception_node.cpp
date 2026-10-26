// PerceptionNode (perception_container)
//   카메라 입력은 모두 ROS 영상 토픽이다. 시뮬레이션은 Isaac Sim, 실물은 realsense2_camera(D435 × 2)와
//   UVC 드라이버(Arducam)가 같은 토픽 이름으로 발행한다. 그래서 실물과 시뮬레이션이 같은 코드로 돈다.
//   드라이버를 같은 컨테이너에 넣으면 intra-process 로 영상을 복사 없이 받는다.
//
//   /perception/set_mode (서비스): 비트 SEARCH_FRONT 1 · ARM_ALIGN 2 · READ_TAG 4 · SEARCH_REAR 8 · READ_MARKER 16 ·
//      READ_SIGNAL 32, 0 = 전부 정지
//      켜진 비트의 카메라 토픽만 구독한다. OFF 를 받으면 대상 추적을 초기화하고 mode=0 result 를 한 번 발행한다
//      (받는 쪽 초기화 신호).
//   /perception/result (토픽): publish_rate 마다 확인해, 새 영상이 하나라도 들어왔을 때만 발행한다
//      stamp = 결과에 쓴 영상 중 가장 오래된 촬영 시각 (받는 쪽 300 ms 규칙의 기준)
//      쓸 수 있는 영상이 하나도 없으면 발행하지 않는다 (카메라 정지 = 침묵, 받는 쪽은 300 ms 뒤 "모름")
//   영상 시각 규칙: 같은·과거 stamp(재전송)는 버린다. 1 s 넘게 뒤로 가면 시각 재시작으로 보고 추적 상태를 초기화한다
//   SEARCH_* : 검출 → (색 검출기면) 덩어리를 깊이가 끊긴 곳에서 나눔 → 조각 깊이 중앙값 → 컬러 광학 좌표 3D
//              → TF 로 base_link · odom → TargetTracker → object_pose
//   ARM_ALIGN: camera_info 로 왜곡 보정한 영상을 detect_width 로 줄여 검출 → 화면 중심 대비 arm_offset
//   READ_TAG : 왜곡 보정 원본에서 AprilTag → tag_id · tag_pose (Arducam 광학 프레임 기준, x 오른쪽 · y 아래 · z 앞)
//   /perception/pucks (토픽, PuckArray): SEARCH_* 또는 READ_TAG 가 켜져 있으면 result 와 같은 주기로 발행한다
//      Arducam 태그 → 번호 · 색 · 중심 · 긴 축(puck_estimator.hpp), D435 색 검출 → 색 · 중심(번호 -1)
//      같은 퍽(merge_dist 안)이면 태그로 읽은 것을 남긴다. 새 영상이 없는 카메라는 그 영상으로 낸 결과를 다시 쓴다
//   /perception/task_marker (토픽, TaskMarker): READ_MARKER 가 켜져 있으면 앞 · 뒤 D435 (팔 카메라가 켜져 있으면 그것도)
//      에서 장소 태그(marker.family · ids)를 찾아 가장 크게 보이는 것 하나. 없으면 found = false
//   /perception/signal (토픽, SignalSequence): READ_SIGNAL 이 켜져 있으면 앞 · 뒤 D435 의 컬러 프레임을 모두(빠짐 없이)
//      처리 주기마다 차례로 본다: 켜진 등 후보(signal_light.hpp) → 깊이로 크기 · 높이 거름 → 같은 자리 점멸로 신호등 고정
//      → 프레임 상태(켜진 등이 없고 칸 앞을 무엇이 가리면 꺼짐이 아니라 모름) → 점멸 순서 세기(signal_sequence.hpp).
//      READ_SIGNAL 을 끄면 센 것을 지운다
//   /perception/check_dice (서비스, CheckDice): 가장 최근 /perception/pucks 목록으로 선반 위 3×3 주사위 판정(dice_check.hpp)
//   /perception/barcode (토픽, Barcode): READ_BARCODE 가 켜져 있으면 두 가지를 낸다
//      LOCATE: barcode.camera D435 깊이를 barcode.depth_frames 장 모아 평균(로봇이 멈춘 동안, 평균 중 odom 으로
//              움직였으면 그 프레임부터 다시 모음) → 라벨 찾기(barcode_locator.hpp) → base_link 위치. 묶음마다 한 번
//      READ  : 팔 카메라(cameras.arm.image_topic) 새 영상마다 zbar 로 CODE128 읽기(barcode_reader.hpp, barcode.read_scale 배)
//      둘 다 일꾼 스레드에서 돌아 다른 모드 처리 주기를 막지 않는다. 일꾼이 도는 동안 온 팔 영상은 건너뛴다
#include <algorithm>
#include <atomic>
#include <deque>
#include <chrono>
#include <future>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_interfaces/msg/barcode.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/msg/puck_array.hpp"
#include "robot_interfaces/msg/signal_sequence.hpp"
#include "robot_interfaces/msg/task_marker.hpp"
#include "robot_interfaces/names.hpp"
#include "robot_interfaces/srv/check_dice.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "perception/barcode_locator.hpp"
#include "perception/barcode_reader.hpp"
#include "perception/depth_split.hpp"
#include "perception/detector.hpp"
#include "perception/dice_check.hpp"
#include "perception/frame_gate.hpp"
#include "perception/geometry.hpp"
#include "perception/puck_estimator.hpp"
#include "perception/signal_light.hpp"
#include "perception/signal_sequence.hpp"
#include "perception/tag_depth.hpp"
#include "perception/tag_detector.hpp"
#include "perception/tag_upright.hpp"
#include "perception/target_tracker.hpp"
#include "perception/tf_helper.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace perception
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;
using SetMode = robot_interfaces::srv::SetPerceptionMode;
using Result = robot_interfaces::msg::PerceptionResult;
using Puck = robot_interfaces::msg::Puck;
using PuckArray = robot_interfaces::msg::PuckArray;
using TaskMarker = robot_interfaces::msg::TaskMarker;
using SignalMsg = robot_interfaces::msg::SignalSequence;
using CheckDice = robot_interfaces::srv::CheckDice;
using BarcodeMsg = robot_interfaces::msg::Barcode;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

namespace
{
int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1'000'000'000LL + t.nanosec;
}

// 결과 하나에 쓴 영상들: 가장 오래된 촬영 시각과 새 영상이 있었는지
struct StampSet
{
  std::optional<rclcpp::Time> oldest;
  bool fresh = false;
  void note(const rclcpp::Time & t, bool f)
  {
    if (!oldest || t < *oldest) {oldest = t;}
    fresh = fresh || f;
  }
};

struct MarkerHit
{
  TaskMarker msg;
  double area = 0;
};
}  // namespace

// 깊이 카메라 하나 (D435). 컬러 + 컬러에 맞춘 깊이 + 컬러 camera_info
struct DepthCamera
{
  std::string name;
  uint8_t bit = 0;
  std::string color_topic, depth_topic, info_topic, frame_id;   // frame_id 가 비면 영상 header 값을 쓴다

  std::mutex m;
  Image::ConstSharedPtr color;
  int64_t used_ns = -1;                                         // 마지막 결과에 쓴 컬러 stamp
  NearestCache<Image::ConstSharedPtr> depth{5};
  CameraInfo::ConstSharedPtr info;
  StampGate gate;
  bool restarted = false;
  std::vector<Puck> pucks;                                      // used_ns 영상으로 낸 결과 (처리 그룹에서만 씀)
  std::optional<MarkerHit> marker;
  std::deque<Image::ConstSharedPtr> signal_q;                   // READ_SIGNAL: 아직 처리하지 않은 컬러 프레임
  std::deque<Image::ConstSharedPtr> barcode_q;                  // READ_BARCODE: 아직 평균에 넣지 않은 깊이 프레임

  rclcpp::Subscription<Image>::SharedPtr color_sub, depth_sub;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub;
  bool subscribed() const {return static_cast<bool>(color_sub);}
};

// 단안 카메라 (Arducam). 영상 + camera_info
struct MonoCamera
{
  std::string name;
  std::string image_topic, info_topic, frame_id;

  std::mutex m;
  Image::ConstSharedPtr image;
  int64_t used_ns = -1;
  CameraInfo::ConstSharedPtr info;
  StampGate gate;
  bool restarted = false;
  std::vector<Puck> pucks;
  std::optional<MarkerHit> marker;

  rclcpp::Subscription<Image>::SharedPtr image_sub;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub;
  bool subscribed() const {return static_cast<bool>(image_sub);}
};

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception_node", options)
  {
    auto cam_params = [this](DepthCamera & c, const std::string & key, const std::string & ns) {
      c.name = key;
      c.color_topic = declare_parameter("cameras." + key + ".color_topic", ns + "/color/image_raw");
      c.depth_topic = declare_parameter("cameras." + key + ".depth_topic", ns + "/aligned_depth_to_color/image_raw");
      c.info_topic = declare_parameter("cameras." + key + ".info_topic", ns + "/color/camera_info");
      c.frame_id = declare_parameter("cameras." + key + ".frame_id", std::string(""));
    };
    cam_params(front_, "front", "/top_camera_front");
    cam_params(rear_, "rear", "/top_camera_rear");
    front_.bit = SetMode::Request::SEARCH_FRONT;
    rear_.bit = SetMode::Request::SEARCH_REAR;
    arm_.name = "arm";
    arm_.image_topic = declare_parameter("cameras.arm.image_topic", std::string("/arm_camera/image_raw"));
    arm_.info_topic = declare_parameter("cameras.arm.info_topic", std::string("/arm_camera/camera_info"));
    arm_.frame_id = declare_parameter("cameras.arm.frame_id", std::string(""));

    const std::string qos_name = declare_parameter("image_qos", std::string("sensor_data"));
    const int qos_depth = static_cast<int>(declare_parameter("image_qos_depth", 2));
    image_qos_ = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (qos_name == "reliable") {image_qos_.reliable();} else {image_qos_.best_effort();}

    depth_min_ = declare_parameter("depth.min_m", 0.15);
    depth_max_ = declare_parameter("depth.max_m", 4.0);
    depth_win_ = static_cast<int>(declare_parameter("depth.window", 3));
    depth_tol_ns_ = static_cast<int64_t>(declare_parameter("depth.sync_tol_s", 0.02) * 1e9);
    detect_w_ = static_cast<int>(declare_parameter("arm.detect_width", 640));

    DetectorConfig dc;
    dc.type = declare_parameter("detector.type", dc.type);
    dc.engine_path = declare_parameter("detector.engine_path", dc.engine_path);
    dc.score_threshold = static_cast<float>(declare_parameter("detector.score_threshold", 0.3));
    dc.nms_threshold = static_cast<float>(declare_parameter("detector.nms_threshold", 0.45));
    dc.target_class = static_cast<int>(declare_parameter("detector.target_class", -1));
    dc.target_color = declare_parameter("detector.target_color", dc.target_color);
    dc.min_area_ratio = declare_parameter("detector.min_area_ratio", dc.min_area_ratio);
    dc.blur_ksize = static_cast<int>(declare_parameter("detector.blur_ksize", dc.blur_ksize));
    dc.morph_kernel = static_cast<int>(declare_parameter("detector.morph_kernel", dc.morph_kernel));
    dc.min_fill_ratio = declare_parameter("detector.min_fill_ratio", dc.min_fill_ratio);
    {   // HSV 범위: detector.hsv.<key>.lower/upper = [H, S, V]. 조명이 바뀌면 yaml 만 고친다
      const char * keys[] = {"red", "red2", "green", "blue"};
      for (size_t i = 0; i < dc.hsv.size(); ++i) {
        auto & h = dc.hsv[i];
        const std::string p = std::string("detector.hsv.") + keys[i];
        const auto lo = declare_parameter(p + ".lower", std::vector<int64_t>{(int64_t)h.lower[0], (int64_t)h.lower[1], (int64_t)h.lower[2]});
        const auto hi = declare_parameter(p + ".upper", std::vector<int64_t>{(int64_t)h.upper[0], (int64_t)h.upper[1], (int64_t)h.upper[2]});
        if (lo.size() == 3) {h.lower = cv::Scalar(lo[0], lo[1], lo[2]);}
        if (hi.size() == 3) {h.upper = cv::Scalar(hi[0], hi[1], hi[2]);}
      }
    }
    target_class_ = dc.target_class;
    hsv_ = dc.hsv;
    blur_ksize_ = dc.blur_ksize;
    morph_kernel_ = dc.morph_kernel;
    score_threshold_ = dc.score_threshold;
    split_on_ = declare_parameter("detector.depth_split.enabled", true);
    split_.tol_m = declare_parameter("detector.depth_split.tol_m", split_.tol_m);
    split_.min_face_ratio = declare_parameter("detector.depth_split.min_face_ratio", split_.min_face_ratio);
    split_.min_fill = dc.min_fill_ratio;

    TrackerConfig tc;
    auto ex_min = declare_parameter("target.exclude_min", std::vector<double>{tc.exclude_min.x, tc.exclude_min.y, tc.exclude_min.z});
    auto ex_max = declare_parameter("target.exclude_max", std::vector<double>{tc.exclude_max.x, tc.exclude_max.y, tc.exclude_max.z});
    if (ex_min.size() == 3) {tc.exclude_min = {ex_min[0], ex_min[1], ex_min[2]};}
    if (ex_max.size() == 3) {tc.exclude_max = {ex_max[0], ex_max[1], ex_max[2]};}
    tc.min_confidence = declare_parameter("target.min_confidence", tc.min_confidence);
    tc.gate = declare_parameter("target.gate", tc.gate);
    tc.lost_timeout = declare_parameter("target.lost_timeout", tc.lost_timeout);
    tracker_ = TargetTracker(tc);
    min_confidence_ = tc.min_confidence;
    size_min_m_ = declare_parameter("target.size_min_m", 0.015);
    size_max_m_ = declare_parameter("target.size_max_m", 0.09);

    const double tag_size = declare_parameter("tag.size", 0.10);
    tag_ = std::make_unique<TagDetector>(declare_parameter("tag.family", std::string("tag36h11")), tag_size);
    puck_geom_.tag_half = tag_size / 2.0;
    for (auto id : declare_parameter<std::vector<int64_t>>("tag.ids", std::vector<int64_t>{})) {
      tag_ids_.push_back(static_cast<int>(id));
    }
    marker_ = std::make_unique<TagDetector>(declare_parameter("marker.family", std::string("DICT_4X4_50")),
        declare_parameter("marker.size", 0.10));
    for (auto id : declare_parameter<std::vector<int64_t>>("marker.ids", std::vector<int64_t>{})) {
      marker_ids_.push_back(static_cast<int>(id));
    }
    puck_geom_.half_width = declare_parameter("puck.half_width", puck_geom_.half_width);
    puck_geom_.half_length = declare_parameter("puck.half_length", puck_geom_.half_length);
    puck_geom_.ring_in = declare_parameter("puck.ring_in", puck_geom_.ring_in);
    puck_geom_.ring_out = declare_parameter("puck.ring_out", puck_geom_.ring_out);
    puck_geom_.ring2_in = declare_parameter("puck.ring2_in", puck_geom_.ring2_in);
    puck_geom_.ring2_out = declare_parameter("puck.ring2_out", puck_geom_.ring2_out);
    puck_geom_.axis_probe = declare_parameter("puck.axis_probe", puck_geom_.axis_probe);
    puck_geom_.corner_probe = declare_parameter("puck.corner_probe", puck_geom_.corner_probe);
    puck_geom_.min_votes = static_cast<int>(declare_parameter("puck.min_votes", puck_geom_.min_votes));
    merge_dist_ = declare_parameter("puck.merge_dist", 0.04);
    upright_gap_px_ = declare_parameter("puck.upright_gap_px", 1.0);

    LightBlobConfig & lb = light_cfg_;
    lb.v_min = static_cast<int>(declare_parameter("signal.v_min", lb.v_min));
    lb.s_min = static_cast<int>(declare_parameter("signal.s_min", lb.s_min));
    lb.red_h_max = static_cast<int>(declare_parameter("signal.red_h_max", lb.red_h_max));
    lb.red2_h_min = static_cast<int>(declare_parameter("signal.red2_h_min", lb.red2_h_min));
    lb.yellow_h_min = static_cast<int>(declare_parameter("signal.yellow_h_min", lb.yellow_h_min));
    lb.yellow_h_max = static_cast<int>(declare_parameter("signal.yellow_h_max", lb.yellow_h_max));
    lb.green_h_min = static_cast<int>(declare_parameter("signal.green_h_min", lb.green_h_min));
    lb.green_h_max = static_cast<int>(declare_parameter("signal.green_h_max", lb.green_h_max));
    lb.min_area_px = static_cast<int>(declare_parameter("signal.min_area_px", lb.min_area_px));
    seg_w_ = declare_parameter("signal.segment_width_m", 0.0396);
    seg_h_ = declare_parameter("signal.segment_height_m", 0.0349);
    area_ratio_min_ = declare_parameter("signal.area_ratio_min", 0.15);
    area_ratio_max_ = declare_parameter("signal.area_ratio_max", 2.5);
    light_z_min_ = declare_parameter("signal.height_min_m", 0.05);
    light_z_max_ = declare_parameter("signal.height_max_m", 0.20);
    signal_q_max_ = static_cast<size_t>(declare_parameter("signal.queue_frames", 30));
    // 신호등은 쌓아 둔 컬러 프레임을 한꺼번에 처리하므로 깊이도 그만큼 보관해야 짝을 찾는다
    // (5장이면 처리가 밀린 묶음의 앞 프레임이 깊이 없이 '꺼짐'으로 셈 — R2 재생에서 발견)
    for (auto * c : {&front_, &rear_}) {c->depth.set_capacity(std::max<size_t>(5, signal_q_max_));}
    LocatorConfig lc;
    lc.lock_dist = declare_parameter("signal.lock_dist_m", lc.lock_dist);
    lc.lock_blinks = static_cast<int>(declare_parameter("signal.lock_blinks", lc.lock_blinks));
    lc.min_off_frames = static_cast<int>(declare_parameter("signal.min_off_frames", lc.min_off_frames));
    lc.forget_s = declare_parameter("signal.forget_s", lc.forget_s);
    lc.segment_pitch = seg_h_;
    occl_margin_ = declare_parameter("signal.occlusion_margin_m", occl_margin_);
    locator_ = SignalLightLocator(lc);
    SignalConfig sc;
    sc.blinks_per_cycle = static_cast<int>(declare_parameter("signal.blinks_per_cycle", sc.blinks_per_cycle));
    sc.term_ratio = declare_parameter("signal.term_ratio", sc.term_ratio);
    sc.term_margin_frames = declare_parameter("signal.term_margin_frames", sc.term_margin_frames);
    sc.on_ratio = declare_parameter("signal.on_ratio", sc.on_ratio);
    sc.merge_ratio = declare_parameter("signal.merge_ratio", sc.merge_ratio);
    sc.max_gap = declare_parameter("signal.max_gap_s", sc.max_gap);
    sc.confirm_cycles = static_cast<int>(declare_parameter("signal.confirm_cycles", sc.confirm_cycles));
    sequence_ = SignalSequenceTracker(sc);
    dice_tolerance_ = declare_parameter("dice.tolerance_m", 0.015);
    dice_margin_ = declare_parameter("dice.region_margin_m", 0.04);
    dice_max_age_ = declare_parameter("dice.max_age_s", 1.0);

    // 바코드 (R3): 찾기 문턱은 barcode.<이름> (기본값 = BarcodeLocatorConfig = R3 설계 45차 locate_v2.py CFG2)
    BarcodeLocatorConfig bcc;
    visit_barcode_params(bcc, [this](const char * k, auto & x) {
        using T = std::decay_t<decltype(x)>;
        const std::string name = std::string("barcode.") + k;
        if constexpr (std::is_same_v<T, int>) {
          x = static_cast<int>(declare_parameter(name, static_cast<int64_t>(x)));
        } else {
          x = declare_parameter(name, x);
        }
      });
    std::string tex = declare_parameter("barcode.template_path", std::string(""));
    if (tex.empty()) {
      tex = (ament_index_cpp::get_package_share_path("perception") / "data" / "barcode_P2-01.png").string();
    }
    bc_locator_ = std::make_unique<BarcodeLocator>(bcc, tex);
    const double rs = declare_parameter("barcode.read_scale", 1.0), rg = declare_parameter("barcode.read_gamma", 2.0);
    bc_reader_ = std::make_unique<BarcodeReader>(rs, rg, static_cast<int>(declare_parameter("barcode.read_min_quality", 4)));
    bc_cam_ = declare_parameter("barcode.camera", std::string("front")) == "rear" ? &rear_ : &front_;
    bc_frames_ = std::max(1, static_cast<int>(declare_parameter("barcode.depth_frames", 5)));
    bc_min_valid_ = static_cast<int>(declare_parameter("barcode.depth_min_valid", 0));
    bc_base_frame_ = declare_parameter("barcode.base_frame", std::string("base_footprint"));
    bc_max_move_ = declare_parameter("barcode.max_motion_m", bc_max_move_);
    bc_max_rot_ = declare_parameter("barcode.max_rotation_deg", bc_max_rot_) * M_PI / 180.0;
    bc_arm_.name = "arm_barcode";
    bc_arm_.image_topic = arm_.image_topic;   // 팔 카메라와 같은 토픽. 따로 구독해 ARM_ALIGN · READ_TAG 와 처리한 영상 기록을 나눔
    bc_arm_.info_topic = arm_.info_topic;   // 읽기에는 안 쓰지만 MonoCamera 구독이 같이 만든다
    bc_arm_.frame_id = arm_.frame_id;
    if (!bc_locator_->ok()) {RCLCPP_ERROR(get_logger(), "바코드 라벨 그림을 읽지 못함: %s (READ_BARCODE 거절)", tex.c_str());}

    const double rate = declare_parameter("publish_rate", 15.0);
    frame_max_age_ = declare_parameter("frame_max_age_s", 0.3);
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    odom_frame_ = declare_parameter("frames.odom", std::string("odom"));

    std::string info;
    detector_ = make_detector(dc, info);
    if (!detector_) {
      RCLCPP_FATAL(get_logger(), "검출기 생성 실패: %s", info.c_str());
      throw std::runtime_error("detector");
    }
    split_on_ = split_on_ && detector_->name() != "tensorrt";   // 깊이 나누기는 색 검출기 덩어리에만
    split_.min_m = depth_min_;
    split_.max_m = depth_max_;
    RCLCPP_INFO(get_logger(), "검출기: %s %s · 태그 %s · 마커 %s", detector_->name().c_str(), info.c_str(),
      tag_->ok() ? "사용 가능" : "패밀리 오류", marker_->ok() ? "사용 가능" : "사전 이름 오류");

    tf_ = std::make_unique<TfHelper>(*this);

    // 영상 콜백은 받아 두기만 한다(병렬). 모드 변경과 처리는 한 그룹에서 차례로 돌아 구독 생성·삭제와 겹치지 않는다
    cam_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ctl_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_ = create_publisher<Result>(names::PERCEPTION_RESULT, qos::result());
    puck_pub_ = create_publisher<PuckArray>(names::PERCEPTION_PUCKS, qos::result());
    marker_pub_ = create_publisher<TaskMarker>(names::PERCEPTION_TASK_MARKER, qos::result());
    signal_pub_ = create_publisher<SignalMsg>(names::PERCEPTION_SIGNAL, qos::result());
    barcode_pub_ = create_publisher<BarcodeMsg>(names::PERCEPTION_BARCODE, qos::result());
    dice_srv_ = create_service<CheckDice>(names::PERCEPTION_CHECK_DICE,
        [this](CheckDice::Request::SharedPtr req, CheckDice::Response::SharedPtr res) {on_check_dice(*req, *res);},
        rclcpp::ServicesQoS(), ctl_cbg_);
    srv_ = create_service<SetMode>(names::PERCEPTION_SET_MODE,
        [this](SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res) {on_set_mode(*req, *res);},
        rclcpp::ServicesQoS(), ctl_cbg_);
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {process();}, ctl_cbg_);
  }

private:
  static constexpr uint8_t SEARCH_BITS = SetMode::Request::SEARCH_FRONT | SetMode::Request::SEARCH_REAR;
  static constexpr uint8_t ARM_BITS = SetMode::Request::ARM_ALIGN | SetMode::Request::READ_TAG;
  static constexpr uint8_t MARKER_BIT = SetMode::Request::READ_MARKER;
  static constexpr uint8_t SIGNAL_BIT = SetMode::Request::READ_SIGNAL;
  static constexpr uint8_t BARCODE_BIT = SetMode::Request::READ_BARCODE;

  // ------------------------------ set_mode ------------------------------
  void on_set_mode(const SetMode::Request & req, SetMode::Response & res)
  {
    if (req.mode & ~(SEARCH_BITS | ARM_BITS | MARKER_BIT | SIGNAL_BIT | BARCODE_BIT)) {res.success = false; res.message = "모르는 모드 비트"; return;}
    if ((req.mode & SetMode::Request::SEARCH_FRONT) && front_.color_topic.empty()) {
      res.success = false; res.message = "전방 카메라 토픽 미설정"; return;
    }
    if ((req.mode & SetMode::Request::SEARCH_REAR) && rear_.color_topic.empty()) {
      res.success = false; res.message = "후방 카메라 토픽 미설정"; return;
    }
    if ((req.mode & ARM_BITS) && arm_.image_topic.empty()) {res.success = false; res.message = "팔 카메라 토픽 미설정"; return;}
    if ((req.mode & SetMode::Request::READ_TAG) && !tag_->ok()) {res.success = false; res.message = "태그 패밀리 오류"; return;}
    if ((req.mode & MARKER_BIT) && !marker_->ok()) {res.success = false; res.message = "마커 사전 이름 오류"; return;}
    if ((req.mode & BARCODE_BIT) && !bc_locator_->ok()) {res.success = false; res.message = "바코드 라벨 그림 없음"; return;}
    if ((req.mode & BARCODE_BIT) && (bc_cam_->color_topic.empty() || bc_arm_.image_topic.empty())) {
      res.success = false; res.message = "바코드 카메라 토픽 미설정"; return;
    }

    const uint8_t old = mode_;
    mode_ = req.mode;
    if ((old ^ req.mode) & SIGNAL_BIT) {   // 켜거나 끌 때 센 것을 지운다
      locator_.reset();
      sequence_.reset();
      signal_last_t_ = -1;
      for (auto * c : {&front_, &rear_}) {std::lock_guard<std::mutex> lk(c->m); c->signal_q.clear();}
    }
    signal_on_ = req.mode & SIGNAL_BIT;
    if ((old ^ req.mode) & BARCODE_BIT) {   // 켜거나 끌 때 모으던 깊이를 지우고, 돌고 있는 일꾼 결과는 내지 않는다
      bc_acc_ = DepthAverage();
      ++bc_gen_;
      std::lock_guard<std::mutex> lk(bc_cam_->m);
      bc_cam_->barcode_q.clear();
    }
    barcode_on_ = req.mode & BARCODE_BIT;
    update_subscriptions();
    for (auto * c : {&front_, &rear_}) {std::lock_guard<std::mutex> lk(c->m); c->used_ns = -1;}   // 새 모드로 지금 영상을 다시 처리
    {std::lock_guard<std::mutex> lk(arm_.m); arm_.used_ns = -1;}
    if (req.mode == SetMode::Request::OFF) {
      tracker_.reset();
      Result r;   // 받는 쪽 초기화 신호 (manipulator 기억 삭제)
      r.header.stamp = now();
      r.header.frame_id = base_frame_;
      r.mode = 0;
      pub_->publish(r);
    }
    res.success = true;
    res.message = "모드 " + std::to_string(old) + " → " + std::to_string(req.mode);
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  void update_subscriptions()
  {
    const uint8_t bc = mode_ & BARCODE_BIT;
    set_depth_subscribed(front_, (mode_ & (front_.bit | MARKER_BIT | SIGNAL_BIT)) || (bc && bc_cam_ == &front_));
    set_depth_subscribed(rear_, (mode_ & (rear_.bit | MARKER_BIT | SIGNAL_BIT)) || (bc && bc_cam_ == &rear_));
    set_mono_subscribed(arm_, mode_ & ARM_BITS);
    set_mono_subscribed(bc_arm_, bc);
  }

  void set_depth_subscribed(DepthCamera & c, bool on)
  {
    if (on == c.subscribed()) {return;}
    rclcpp::SubscriptionOptions o;
    o.callback_group = cam_cbg_;
    if (on) {
      c.color_sub = create_subscription<Image>(c.color_topic, image_qos_,
          [this, &c](Image::ConstSharedPtr m) {on_depth_color(c, m);}, o);
      c.depth_sub = create_subscription<Image>(c.depth_topic, image_qos_,
          [this, &c](Image::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lk(c.m);
            c.depth.push(stamp_ns(m->header.stamp), m);
            if (barcode_on_ && &c == bc_cam_) {   // 바코드 깊이 평균은 프레임을 빠짐 없이 쓴다
              c.barcode_q.push_back(m);
              while (c.barcode_q.size() > static_cast<size_t>(2 * bc_frames_)) {c.barcode_q.pop_front();}
            }
          }, o);
      c.info_sub = create_subscription<CameraInfo>(c.info_topic, image_qos_,
          [&c](CameraInfo::ConstSharedPtr m) {std::lock_guard<std::mutex> lk(c.m); c.info = m;}, o);
      RCLCPP_INFO(get_logger(), "%s 카메라 구독 시작 %s", c.name.c_str(), c.color_topic.c_str());
    } else {
      c.color_sub.reset();
      c.depth_sub.reset();
      c.info_sub.reset();
      std::lock_guard<std::mutex> lk(c.m);
      c.color.reset();
      c.depth.clear();
      c.gate.reset();
      c.used_ns = -1;
      c.pucks.clear();
      c.marker.reset();
      c.signal_q.clear();
      c.barcode_q.clear();
      RCLCPP_INFO(get_logger(), "%s 카메라 구독 끝", c.name.c_str());
    }
  }

  void set_mono_subscribed(MonoCamera & c, bool on)
  {
    if (on == c.subscribed()) {return;}
    rclcpp::SubscriptionOptions o;
    o.callback_group = cam_cbg_;
    if (on) {
      c.image_sub = create_subscription<Image>(c.image_topic, image_qos_,
          [this, &c](Image::ConstSharedPtr m) {on_mono_image(c, m);}, o);
      c.info_sub = create_subscription<CameraInfo>(c.info_topic, image_qos_,
          [&c](CameraInfo::ConstSharedPtr m) {std::lock_guard<std::mutex> lk(c.m); c.info = m;}, o);
      RCLCPP_INFO(get_logger(), "%s 카메라 구독 시작 %s", c.name.c_str(), c.image_topic.c_str());
    } else {
      c.image_sub.reset();
      c.info_sub.reset();
      std::lock_guard<std::mutex> lk(c.m);
      c.image.reset();
      c.gate.reset();
      c.used_ns = -1;
      c.pucks.clear();
      c.marker.reset();
      RCLCPP_INFO(get_logger(), "%s 카메라 구독 끝", c.name.c_str());
    }
  }

  // ------------------------------ 영상 수신 ------------------------------
  // 같은·과거 stamp 는 버린다(Duplicate). 1 s 넘게 뒤로 가면 Restart: 이전 시각 기준 상태를 지운다
  template<class Cam>
  GateResult gate_frame(Cam & c, const Image::ConstSharedPtr & m)
  {
    const GateResult g = c.gate.check(stamp_ns(m->header.stamp));
    if (g == GateResult::Restart) {
      RCLCPP_WARN(get_logger(), "%s 영상 시각이 1 s 넘게 뒤로 감(시각 재시작): 추적 상태를 초기화", c.name.c_str());
      c.restarted = true;
      c.used_ns = -1;
    }
    return g;
  }

  void on_depth_color(DepthCamera & c, const Image::ConstSharedPtr & m)
  {
    std::lock_guard<std::mutex> lk(c.m);
    const GateResult g = gate_frame(c, m);
    if (g == GateResult::Duplicate) {return;}
    if (g == GateResult::Restart) {c.depth.clear(); c.signal_q.clear();}   // 재시작 전 시각의 깊이와 짝짓지 않게
    c.color = m;
    if (signal_on_) {   // 신호등은 점멸이 짧을 수 있어 처리 주기와 상관없이 모든 프레임을 본다
      c.signal_q.push_back(m);
      while (c.signal_q.size() > signal_q_max_) {c.signal_q.pop_front();}
    }
  }

  void on_mono_image(MonoCamera & c, const Image::ConstSharedPtr & m)
  {
    std::lock_guard<std::mutex> lk(c.m);
    if (gate_frame(c, m) != GateResult::Duplicate) {c.image = m;}
  }

  // ------------------------------ 처리 ------------------------------
  // 카메라 광학 좌표 점 → base_link · odom. D435 는 차체에 고정이라 base 변환은 한 번 찾아 기억하고, odom 은 촬영 시각
  bool to_base_odom(const cv::Point3d & p, const std::string & cam_frame, const rclcpp::Time & stamp, Vec3 & base, Vec3 & odom)
  {
    const auto t_bc = tf_->fixed(base_frame_, cam_frame);
    if (!t_bc) {return false;}
    const tf2::Vector3 b = *t_bc * tf2::Vector3(p.x, p.y, p.z);
    base = {b.x(), b.y(), b.z()};
    odom = base;
    try {
      geometry_msgs::msg::TransformStamped t_ob;
      const tf2::TimePoint tp = tf2_ros::fromRclcpp(stamp);
      auto & buf = tf_->buffer();
      if (buf.canTransform(odom_frame_, base_frame_, tp)) {
        t_ob = buf.lookupTransform(odom_frame_, base_frame_, tp);
      } else {
        t_ob = buf.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      }
      tf2::Transform t;
      tf2::fromMsg(t_ob.transform, t);
      const tf2::Vector3 o = t * b;
      odom = {o.x(), o.y(), o.z()};
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "TF %s→%s 없음: base_link 기준으로 추적", base_frame_.c_str(), odom_frame_.c_str());
    }
    return true;
  }

  // 쓸 수 있는 최신 컬러(+깊이)를 꺼낸다. 300 ms 보다 오래되면 쓰지 않는다
  struct DepthSnapshot
  {
    Image::ConstSharedPtr color, depth;
    CameraInfo::ConstSharedPtr info;
    bool fresh = false;
  };
  std::optional<DepthSnapshot> take(DepthCamera & c, const rclcpp::Time & now_t)
  {
    std::lock_guard<std::mutex> lk(c.m);
    if (!c.color) {return std::nullopt;}
    const int64_t ns = stamp_ns(c.color->header.stamp);
    if ((now_t - rclcpp::Time(c.color->header.stamp, now_t.get_clock_type())).seconds() > frame_max_age_) {return std::nullopt;}
    DepthSnapshot s;
    s.color = c.color;
    s.info = c.info;
    if (auto d = c.depth.nearest(ns, depth_tol_ns_)) {s.depth = d->second;}
    s.fresh = ns != c.used_ns;
    c.used_ns = ns;
    if (c.restarted) {c.restarted = false; restart_seen_ = true;}
    return s;
  }

  // 검출 → 3D 후보(추적용) · 퍽(색 · 중심, 번호 없음). 퍽 중심은 보이는 겉면에서 시선 방향으로 half_width 더 들어간 점
  void collect(DepthCamera & c, const DepthSnapshot & s, const cv::Mat & color, uint8_t cam_idx, std::vector<Candidate> & out)
  {
    if (!s.depth || !s.info) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: %s 없음 — 3D 위치를 낼 수 없음",
        c.name.c_str(), !s.info ? "camera_info" : "stamp 맞는 깊이 영상");
      return;
    }
    const double scale = depth_scale_for_encoding(s.depth->encoding);
    if (scale <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: 모르는 깊이 인코딩 %s", c.name.c_str(), s.depth->encoding.c_str());
      return;
    }
    if (s.depth->width != s.color->width || s.depth->height != s.color->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: 깊이 %ux%u 와 컬러 %ux%u 크기가 다름 (컬러에 맞춘 깊이가 아님)",
        c.name.c_str(), s.depth->width, s.depth->height, s.color->width, s.color->height);
      return;
    }
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(s.depth);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 깊이 변환 실패: %s", c.name.c_str(), e.what());
      return;
    }
    const PinholeK K{s.info->k[0], s.info->k[4], s.info->k[2], s.info->k[5]};
    const std::string frame = c.frame_id.empty() ? s.color->header.frame_id : c.frame_id;
    const rclcpp::Time stamp(s.color->header.stamp, get_clock()->get_clock_type());
    // 색 검출 덩어리는 깊이가 끊긴 곳에서 나눠 조각마다 퍽 하나로 본다(depth_split.hpp). 끄면 상자 하나 = 퍽 하나, 중심 = 상자 가운데
    std::vector<DepthPiece> pieces;
    const auto dets = detector_->detect(color);
    if (split_on_) {
      pieces = split_by_depth(dets, color, depth->image, scale, hsv_, blur_ksize_, morph_kernel_, K, split_, score_threshold_);
    } else {
      for (const auto & d : dets) {
        DepthPiece pc;
        pc.det = d;
        pc.center = cv::Point(static_cast<int>(d.box.x + d.box.width / 2), static_cast<int>(d.box.y + d.box.height / 2));
        pc.depth = median_depth(depth->image, pc.center, depth_win_, scale, depth_min_, depth_max_);
        pieces.push_back(pc);
      }
    }
    for (const auto & pc : pieces) {
      const auto & d = pc.det;
      if (target_class_ >= 0 && d.class_id != target_class_) {continue;}
      const cv::Point ctr = pc.center;
      const double z = pc.depth;
      if (!std::isfinite(z)) {continue;}
      if (size_min_m_ > 0 && !size_plausible(d.box, z, K, size_min_m_, size_max_m_)) {continue;}
      Candidate k;
      k.confidence = d.score;
      k.camera = cam_idx;
      const cv::Point3d sp = deproject(K, ctr.x, ctr.y, z);
      if (!to_base_odom(sp, frame, stamp, k.base, k.odom)) {continue;}
      out.push_back(k);
      const double n = cv::norm(sp);
      const cv::Point3d cp = sp * ((n + puck_geom_.half_width) / n);
      const tf2::Vector3 b = *tf_->fixed(base_frame_, frame) * tf2::Vector3(cp.x, cp.y, cp.z);   // to_base_odom 이 이미 찾아 둠
      Puck p;
      p.tag_id = -1;
      p.color = (d.class_id >= 0 && d.class_id < 3) ? static_cast<uint8_t>(d.class_id + 1) : Puck::COLOR_UNKNOWN;
      p.shape = Puck::SHAPE_UNKNOWN;
      p.pose.position.x = b.x();
      p.pose.position.y = b.y();
      p.pose.position.z = b.z();
      p.pose.orientation.w = 1.0;
      p.confidence = d.score;
      p.camera = cam_idx;
      c.pucks.push_back(p);
    }
  }

  // 움직이는 카메라(팔) 변환 target ← source, 촬영 시각 기준. 그 시각 변환이 없으면 가장 최근 것
  std::optional<tf2::Transform> lookup_at(const std::string & target, const std::string & source, const rclcpp::Time & stamp)
  {
    try {
      auto & buf = tf_->buffer();
      const tf2::TimePoint tp = tf2_ros::fromRclcpp(stamp);
      const auto m = buf.canTransform(target, source, tp) ? buf.lookupTransform(target, source, tp) :
        buf.lookupTransform(target, source, tf2::TimePointZero);
      tf2::Transform t;
      tf2::fromMsg(m.transform, t);
      return t;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s→%s 없음: %s", source.c_str(), target.c_str(), e.what());
      return std::nullopt;
    }
  }

  static tf2::Matrix3x3 to_tf(const cv::Matx33d & R)
  {
    return tf2::Matrix3x3(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
  }

  // Arducam 태그 → 퍽. 같은 번호가 여러 면에 보이면 가장 크게 보이는 면 하나
  std::vector<Puck> pucks_from_tags(const std::vector<TagDetection> & tags, const cv::Mat & bgr, const cv::Mat & K,
    const tf2::Transform & T)
  {
    std::vector<const TagDetection *> order;
    for (const auto & t : tags) {if (t.pose_valid) {order.push_back(&t);}}
    std::sort(order.begin(), order.end(), [](auto * a, auto * b) {return a->area > b->area;});
    std::vector<Puck> out;
    std::vector<int> seen;
    for (const auto * t : order) {
      if (std::find(seen.begin(), seen.end(), t->id) != seen.end()) {continue;}
      seen.push_back(t->id);
      // PnP 두 해의 오차가 거의 같으면 법선이 base_link 수직 · 수평에 가까운 해 (tag_upright.hpp)
      const tf2::Matrix3x3 & B = T.getBasis();
      const cv::Matx33d R_base_cam(B[0][0], B[0][1], B[0][2], B[1][0], B[1][1], B[1][2], B[2][0], B[2][1], B[2][2]);
      const PuckEstimate e = estimate_puck(bgr, K, prefer_upright(*t, R_base_cam, upright_gap_px_), hsv_, puck_geom_);
      Puck p;
      p.tag_id = t->id;
      p.color = e.color_cls >= 0 ? static_cast<uint8_t>(e.color_cls + 1) : Puck::COLOR_UNKNOWN;
      p.shape = static_cast<uint8_t>(e.shape);   // 0 모름 · 1 원기둥 · 2 직육면체 (Puck.SHAPE_* 와 같음)
      const tf2::Vector3 c = T * tf2::Vector3(e.center[0], e.center[1], e.center[2]);
      p.pose.position.x = c.x();
      p.pose.position.y = c.y();
      p.pose.position.z = c.z();
      p.pose.orientation.w = 1.0;
      if (e.orientation_valid) {
        tf2::Matrix3x3 Rb = T.getBasis() * to_tf(e.axes);
        const tf2::Vector3 z = Rb.getColumn(2);
        // 긴 축 부호: 서 있으면 위로, 누워 있으면 base_link x+ (옆으로 누우면 y+) 쪽
        const bool flip = std::abs(z.z()) > 0.5 ? z.z() < 0 : (std::abs(z.x()) >= std::abs(z.y()) ? z.x() < 0 : z.y() < 0);
        if (flip) {Rb = Rb * tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);}
        p.orientation_valid = true;
        p.standing = Rb.getColumn(2).z() > std::cos(M_PI / 6);
        tf2::Quaternion q;
        Rb.getRotation(q);
        p.pose.orientation = tf2::toMsg(q);
      }
      p.confidence = static_cast<float>(e.confidence);
      p.camera = Puck::CAMERA_ARM;
      out.push_back(p);
    }
    return out;
  }

  // 장소 태그 검출 중 가장 크게 보이는 것 → base_link 기준 마커
  std::optional<MarkerHit> best_marker(const std::vector<TagDetection> & dets, const tf2::Transform & T,
    const builtin_interfaces::msg::Time & stamp, uint8_t cam)
  {
    const TagDetection * best = nullptr;
    for (const auto & d : dets) {
      if (!d.pose_valid) {continue;}
      if (!marker_ids_.empty() && std::find(marker_ids_.begin(), marker_ids_.end(), d.id) == marker_ids_.end()) {continue;}
      if (!best || d.area > best->area) {best = &d;}
    }
    if (!best) {return std::nullopt;}
    cv::Matx33d R;
    cv::Rodrigues(best->rvec, R);
    const tf2::Vector3 c = T * tf2::Vector3(best->tvec[0], best->tvec[1], best->tvec[2]);
    tf2::Quaternion q;
    (T.getBasis() * to_tf(R)).getRotation(q);
    MarkerHit h;
    h.area = best->area;
    h.msg.header.stamp = stamp;
    h.msg.header.frame_id = base_frame_;
    h.msg.found = true;
    h.msg.marker_id = best->id;
    h.msg.pose.position.x = c.x();
    h.msg.pose.position.y = c.y();
    h.msg.pose.position.z = c.z();
    h.msg.pose.orientation = tf2::toMsg(q);
    h.msg.distance = static_cast<float>(cv::norm(best->tvec));
    h.msg.camera = cam;
    return h;
  }

  // D435 컬러(보정 전 원본)에서 마커. 왜곡 계수는 camera_info 값
  std::optional<MarkerHit> marker_on_depth_cam(DepthCamera & c, const DepthSnapshot & s, const cv::Mat & color, uint8_t cam)
  {
    if (!s.info || s.info->k[0] <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: camera_info 없음 — 마커 자세를 낼 수 없음", c.name.c_str());
      return std::nullopt;
    }
    cv::Mat K(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {K.at<double>(i / 3, i % 3) = s.info->k[i];}
    cv::Mat D(1, static_cast<int>(s.info->d.size()), CV_64F);
    for (size_t i = 0; i < s.info->d.size(); ++i) {D.at<double>(0, static_cast<int>(i)) = s.info->d[i];}
    const std::string frame = c.frame_id.empty() ? s.color->header.frame_id : c.frame_id;
    const auto T = tf_->fixed(base_frame_, frame);
    if (!T) {return std::nullopt;}
    auto dets = marker_->detect(color, K, s.info->d.empty() ? cv::Mat() : D);
    // 정렬 깊이로 태그 거리를 맞춘다 (tag_depth.hpp). 깊이가 없거나 크기가 다르면 PnP 값 그대로
    const double scale = s.depth ? depth_scale_for_encoding(s.depth->encoding) : 0.0;
    if (scale > 0.0 && s.depth->width == static_cast<uint32_t>(color.cols) &&
      s.depth->height == static_cast<uint32_t>(color.rows))
    {
      try {
        const auto dimg = cv_bridge::toCvShare(s.depth);
        for (auto & d : dets) {
          if (d.pose_valid) {scale_tag_to_depth(d.tvec, d.corners, dimg->image, scale, depth_min_, depth_max_);}
        }
      } catch (const cv_bridge::Exception &) {
      }
    }
    return best_marker(dets, *T, s.color->header.stamp, cam);
  }

  void process()
  {
    const uint8_t mode = mode_;
    if (mode == SetMode::Request::OFF) {return;}
    const rclcpp::Time now_t = now();
    const bool want_marker = mode & MARKER_BIT;
    if (mode & SIGNAL_BIT) {process_signal();}
    if (mode & BARCODE_BIT) {process_barcode(now_t);}
    if (!(mode & ~(SIGNAL_BIT | BARCODE_BIT))) {return;}
    Result r;
    r.mode = mode;
    StampSet res_st, puck_st, mark_st;
    std::vector<const std::vector<Puck> *> puck_lists;     // 이번 결과에 쓰는 카메라별 퍽 (팔이 맨 앞)
    std::vector<const MarkerHit *> markers;

    std::vector<Candidate> cands;   // 추적에는 새 영상의 후보만 쓴다
    bool search_have = false;
    std::vector<DepthCamera *> depth_used;
    for (auto * c : {&front_, &rear_}) {
      const bool search = mode & c->bit;
      if (!search && !want_marker) {continue;}
      auto s = take(*c, now_t);
      if (!s) {continue;}
      const rclcpp::Time st(s->color->header.stamp, now_t.get_clock_type());
      if (search) {res_st.note(st, s->fresh); puck_st.note(st, s->fresh); search_have = true;}
      if (want_marker) {mark_st.note(st, s->fresh);}
      if (s->fresh) {   // 새 영상만 처리. 같은 영상이면 그 영상으로 낸 퍽 · 마커를 다시 쓴다
        c->pucks.clear();
        c->marker.reset();
        cv_bridge::CvImageConstPtr color;
        try {
          color = cv_bridge::toCvShare(s->color, "bgr8");
        } catch (const cv_bridge::Exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 영상 변환 실패: %s", c->name.c_str(), e.what());
          continue;
        }
        const uint8_t idx = c == &front_ ? Puck::CAMERA_FRONT : Puck::CAMERA_REAR;
        if (search) {collect(*c, *s, color->image, idx, cands);}
        if (want_marker) {c->marker = marker_on_depth_cam(*c, *s, color->image, idx);}
      }
      if (search) {puck_lists.push_back(&c->pucks);}
      if (want_marker && c->marker) {markers.push_back(&*c->marker);}
    }
    if (restart_seen_) {tracker_.reset(); restart_seen_ = false;}
    if (search_have && res_st.fresh) {
      if (auto t = tracker_.update(cands, res_st.oldest->seconds())) {
        r.object_found = true;
        r.object_pose.header.frame_id = base_frame_;
        r.object_pose.header.stamp = *res_st.oldest;
        r.object_pose.pose.position.x = t->base.x;
        r.object_pose.pose.position.y = t->base.y;
        r.object_pose.pose.position.z = t->base.z;
        r.object_pose.pose.orientation.w = 1.0;
        r.confidence = static_cast<float>(t->confidence);
      }
    }

    if (mode & ARM_BITS) {
      Image::ConstSharedPtr img;
      CameraInfo::ConstSharedPtr info;
      bool fresh = false;
      {
        std::lock_guard<std::mutex> lk(arm_.m);
        if (arm_.image && (now_t - rclcpp::Time(arm_.image->header.stamp, now_t.get_clock_type())).seconds() <= frame_max_age_) {
          img = arm_.image;
          info = arm_.info;
          const int64_t ns = stamp_ns(img->header.stamp);
          fresh = ns != arm_.used_ns;
          arm_.used_ns = ns;
          arm_.restarted = false;
        }
      }
      if (img) {
        const rclcpp::Time st(img->header.stamp, now_t.get_clock_type());
        res_st.note(st, fresh);
        if (mode & SetMode::Request::READ_TAG) {puck_st.note(st, fresh);}
        if (want_marker) {mark_st.note(st, fresh);}
        if (fresh) {
          arm_.pucks.clear();
          arm_.marker.reset();
          process_arm(img, info, mode, r);
        }
        if (mode & SetMode::Request::READ_TAG) {puck_lists.insert(puck_lists.begin(), &arm_.pucks);}
        if (want_marker && arm_.marker) {markers.push_back(&*arm_.marker);}
      }
    }

    if (!res_st.oldest && !puck_st.oldest && !mark_st.oldest) {   // 쓸 수 있는 영상이 하나도 없음 → 발행하지 않음
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAMERA_STALL: 모드 0x%02x 에 쓸 수 있는 영상 없음", mode);
      return;
    }
    // 새 영상이 없으면 같은 결과를 다시 내지 않는다
    if (res_st.oldest && res_st.fresh) {
      r.header.stamp = *res_st.oldest;
      r.header.frame_id = base_frame_;
      pub_->publish(r);
    }
    if (puck_st.oldest && puck_st.fresh) {publish_pucks(puck_lists, *puck_st.oldest);}
    if (!puck_st.oldest) {last_pucks_.reset();}
    if (mark_st.oldest && mark_st.fresh) {
      const MarkerHit * best = nullptr;
      for (const auto * m : markers) {if (!best || m->area > best->area) {best = m;}}
      TaskMarker msg;
      if (best) {
        msg = best->msg;
      } else {
        msg.header.stamp = *mark_st.oldest;
        msg.header.frame_id = base_frame_;
        msg.found = false;
        msg.marker_id = -1;
        msg.pose.orientation.w = 1.0;
      }
      marker_pub_->publish(msg);
    }
  }

  // 태그로 읽은 퍽(팔)을 먼저 넣고, D435 퍽은 그 merge_dist 안이면 뺀다. 수평 거리 순 정렬, 집을 후보 고르기
  void publish_pucks(const std::vector<const std::vector<Puck> *> & lists, const rclcpp::Time & stamp)
  {
    PuckArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;
    for (const auto * l : lists) {
      for (const auto & p : *l) {
        bool dup = false;
        for (const auto & q : msg.pucks) {
          if (q.tag_id < 0 || p.tag_id >= 0) {continue;}
          const double dx = p.pose.position.x - q.pose.position.x, dy = p.pose.position.y - q.pose.position.y,
            dz = p.pose.position.z - q.pose.position.z;
          dup = dup || std::sqrt(dx * dx + dy * dy + dz * dz) < merge_dist_;
        }
        if (!dup) {msg.pucks.push_back(p);}
      }
    }
    auto hd = [](const Puck & p) {return std::hypot(p.pose.position.x, p.pose.position.y);};
    std::stable_sort(msg.pucks.begin(), msg.pucks.end(), [&](const Puck & a, const Puck & b) {return hd(a) < hd(b);});
    msg.target_index = -1;
    for (size_t i = 0; i < msg.pucks.size(); ++i) {
      const auto & p = msg.pucks[i];
      const Vec3 v{p.pose.position.x, p.pose.position.y, p.pose.position.z};
      if ((p.tag_id >= 0 || p.confidence >= min_confidence_) && !tracker_.excluded(v)) {
        msg.target_index = static_cast<int32_t>(i);
        break;
      }
    }
    puck_pub_->publish(msg);
    last_pucks_ = msg;
  }

  // ------------------------------ 바코드 ------------------------------
  static bool busy(const std::future<void> & f)
  {
    return f.valid() && f.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  }

  // 깊이 한 장을 평균에 넣는다. 평균 중 차체가 움직였거나 시각이 뒤로 갔으면 이 장부터 다시 모은다
  void add_barcode_depth(DepthCamera & c, const Image::ConstSharedPtr & m)
  {
    const int64_t ns = stamp_ns(m->header.stamp);
    DepthAverage & a = bc_acc_;
    if (a.n > 0 && ns <= a.last_ns) {
      if (a.last_ns - ns <= 1'000'000'000LL) {return;}   // 재전송 · 순서 바뀜
      a = DepthAverage();                                 // 시각 재시작
    }
    const double scale = depth_scale_for_encoding(m->encoding);
    if (scale <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 깊이 형식 %s 은 바코드 평균에 못 씀", c.name.c_str(), m->encoding.c_str());
      return;
    }
    cv_bridge::CvImageConstPtr d;
    try {
      d = cv_bridge::toCvShare(m);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 깊이 변환 실패: %s", c.name.c_str(), e.what());
      return;
    }
    const rclcpp::Time stamp(m->header.stamp, get_clock()->get_clock_type());
    const auto T_ob = lookup_at(odom_frame_, base_frame_, stamp);
    if (a.n > 0) {
      bool moved = d->image.size() != a.sum.size();
      if (!moved && T_ob && a.odom0) {
        const tf2::Transform dT = a.odom0->inverse() * *T_ob;
        moved = dT.getOrigin().length() > bc_max_move_ || dT.getRotation().getAngleShortestPath() > bc_max_rot_;
      }
      if (moved) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "바코드: 깊이 평균 중 차체가 움직임 — 다시 모음");
        a = DepthAverage();
      }
    }
    if (a.n == 0) {
      a.sum = cv::Mat::zeros(d->image.size(), CV_64F);
      a.cnt = cv::Mat::zeros(d->image.size(), CV_32S);
      a.odom0 = T_ob;
      a.first_ns = ns;
    }
    cv::Mat z;
    d->image.convertTo(z, CV_32F, scale);
    for (int y = 0; y < z.rows; ++y) {
      const float * zp = z.ptr<float>(y);
      double * sp = a.sum.ptr<double>(y);
      int * cp = a.cnt.ptr<int>(y);
      for (int x = 0; x < z.cols; ++x) {
        if (std::isfinite(zp[x]) && zp[x] > 0.0f) {sp[x] += zp[x]; ++cp[x];}
      }
    }
    ++a.n;
    a.last_ns = ns;
  }

  void process_barcode(const rclcpp::Time & now_t)
  {
    DepthCamera & c = *bc_cam_;
    std::deque<Image::ConstSharedPtr> q;
    Image::ConstSharedPtr color;
    CameraInfo::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lk(c.m);
      q.swap(c.barcode_q);
      color = c.color;
      info = c.info;
    }
    std::sort(q.begin(), q.end(), [](const auto & x, const auto & y) {return stamp_ns(x->header.stamp) < stamp_ns(y->header.stamp);});
    for (const auto & m : q) {
      if (bc_acc_.n < bc_frames_) {add_barcode_depth(c, m);}
    }
    // 찾기: 다 모였고 일꾼이 쉬고 있으면. 컬러는 평균을 시작한 뒤 찍힌 것(멈춘 동안)이어야 한다
    if (bc_acc_.n >= bc_frames_ && !busy(bc_locate_job_) && color && info &&
      stamp_ns(color->header.stamp) >= bc_acc_.first_ns - depth_tol_ns_)
    {
      launch_barcode_locate(c, color, info);
    }
    // 읽기: 팔 카메라 새 영상, 일꾼이 쉬고 있으면
    if (busy(bc_read_job_)) {return;}
    Image::ConstSharedPtr img;
    {
      std::lock_guard<std::mutex> lk(bc_arm_.m);
      if (bc_arm_.image && (now_t - rclcpp::Time(bc_arm_.image->header.stamp, now_t.get_clock_type())).seconds() <= frame_max_age_) {
        const int64_t ns = stamp_ns(bc_arm_.image->header.stamp);
        if (ns != bc_arm_.used_ns) {img = bc_arm_.image; bc_arm_.used_ns = ns;}
        bc_arm_.restarted = false;
      }
    }
    if (!img) {return;}
    cv_bridge::CvImageConstPtr im;
    try {
      im = cv_bridge::toCvShare(img, img->encoding == "mono8" ? "mono8" : "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "바코드 팔 영상 변환 실패: %s", e.what());
      return;
    }
    const uint64_t gen = bc_gen_;
    bc_read_job_ = std::async(std::launch::async, [this, im, gen]() {
        const BarcodeRead r = bc_reader_->read(im->image);
        if (gen != bc_gen_) {return;}
        BarcodeMsg msg;
        msg.header.stamp = im->header.stamp;
        msg.header.frame_id = base_frame_;
        msg.source = BarcodeMsg::READ;
        msg.read_ok = r.ok;
        msg.text = r.text;
        msg.symbols = static_cast<uint8_t>(std::min<size_t>(r.symbols.size(), 255));
        barcode_pub_->publish(msg);
      });
  }

  void launch_barcode_locate(DepthCamera & c, const Image::ConstSharedPtr & color_msg, const CameraInfo::ConstSharedPtr & info)
  {
    DepthAverage a = std::move(bc_acc_);
    bc_acc_ = DepthAverage();   // 다음 묶음은 새로 모은다
    cv_bridge::CvImageConstPtr color;
    try {
      color = cv_bridge::toCvShare(color_msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 영상 변환 실패: %s", c.name.c_str(), e.what());
      return;
    }
    if (color->image.size() != a.sum.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "바코드: %s 컬러와 깊이 크기가 다름(컬러에 맞춘 깊이여야 함)", c.name.c_str());
      return;
    }
    const std::string frame = c.frame_id.empty() ? color_msg->header.frame_id : c.frame_id;
    const auto T_fc = tf_->fixed(bc_base_frame_, frame), T_bc = tf_->fixed(base_frame_, frame);
    if (!T_fc || !T_bc) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "바코드: TF %s · %s ← %s 없음", bc_base_frame_.c_str(),
        base_frame_.c_str(), frame.c_str());
      return;
    }
    // 평균 깊이: depth_min_valid 장(0 이면 모든 장) 넘게 값이 있는 화소만, 나머지는 0(없음)
    const int need = bc_min_valid_ > 0 && bc_min_valid_ < a.n ? bc_min_valid_ : a.n;
    cv::Mat avg(a.sum.size(), CV_32F);
    for (int y = 0; y < avg.rows; ++y) {
      for (int x = 0; x < avg.cols; ++x) {
        const int n = a.cnt.at<int>(y, x);
        avg.at<float>(y, x) = n >= need ? static_cast<float>(a.sum.at<double>(y, x) / n) : 0.0f;
      }
    }
    const BarcodeK K{info->k[0], info->k[4], info->k[2], info->k[5]};
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {T(r, k) = T_fc->getBasis()[r][k];}
      T(r, 3) = T_fc->getOrigin()[r];
    }
    const tf2::Transform Tb = *T_bc;
    const int frames = a.n;
    const uint64_t gen = bc_gen_;
    bc_locate_job_ = std::async(std::launch::async, [this, color, avg, K, T, Tb, frames, gen]() {
        const auto t0 = std::chrono::steady_clock::now();
        const auto dets = bc_locator_->locate(color->image, avg, K, T);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (gen != bc_gen_) {return;}
        BarcodeMsg msg;
        msg.header.stamp = color->header.stamp;
        msg.header.frame_id = base_frame_;
        msg.source = BarcodeMsg::LOCATE;
        msg.depth_frames = static_cast<uint8_t>(std::min(frames, 255));
        for (const auto & d : dets) {
          const tf2::Vector3 p = Tb * tf2::Vector3(d.center[0], d.center[1], d.center[2]);
          geometry_msgs::msg::Point pt;
          pt.x = p.x();
          pt.y = p.y();
          pt.z = p.z();
          msg.positions.push_back(pt);
          msg.scores.push_back(static_cast<float>(d.score));
          msg.surfaces.push_back(d.surface == BarcodeDetection::TOP ? BarcodeMsg::SURFACE_TOP : BarcodeMsg::SURFACE_WALL);
        }
        barcode_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "바코드 찾기: 라벨 %zu 개 (깊이 %d 장 평균, %.0f ms)", dets.size(), frames, ms);
      });
  }

  // ------------------------------ 신호등 ------------------------------
  // 두 카메라에 쌓인 프레임을 촬영 시각 순서로 처리한다. 이미 본 시각보다 이른 프레임은 버린다
  void process_signal()
  {
    struct Frame
    {
      DepthCamera * cam;
      Image::ConstSharedPtr color, depth;
      CameraInfo::ConstSharedPtr info;
    };
    std::vector<Frame> frames;
    for (auto * c : {&front_, &rear_}) {
      std::lock_guard<std::mutex> lk(c->m);
      for (const auto & m : c->signal_q) {
        Frame f{c, m, nullptr, c->info};
        if (auto d = c->depth.nearest(stamp_ns(m->header.stamp), depth_tol_ns_)) {f.depth = d->second;}
        frames.push_back(f);
      }
      c->signal_q.clear();
    }
    std::sort(frames.begin(), frames.end(), [](const Frame & a, const Frame & b) {
        return stamp_ns(a.color->header.stamp) < stamp_ns(b.color->header.stamp);});
    bool any = false, visible = false;
    rclcpp::Time last_stamp;
    for (const auto & f : frames) {
      const rclcpp::Time stamp(f.color->header.stamp, get_clock()->get_clock_type());
      const double t = stamp.seconds();
      if (t <= signal_last_t_ || !f.info) {continue;}
      signal_last_t_ = t;
      if (!f.depth) {   // 깊이 짝이 없으면 켜진 등을 확인할 수 없음 → 꺼짐이 아니라 모름(빠진 프레임)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 신호등 프레임 깊이 짝 없음 — 모름으로 둠",
          f.cam->name.c_str());
        continue;
      }
      cv_bridge::CvImageConstPtr color;
      try {
        color = cv_bridge::toCvShare(f.color, "bgr8");
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 영상 변환 실패: %s", f.cam->name.c_str(), e.what());
        continue;
      }
      const PinholeK K{f.info->k[0], f.info->k[4], f.info->k[2], f.info->k[5]};
      const std::string frame = f.cam->frame_id.empty() ? f.color->header.frame_id : f.cam->frame_id;
      const std::vector<LightObs> obs = light_observations(color->image, f.depth, K, frame, stamp);
      // 신호등 자리가 이 프레임 화면 안인지: odom → 카메라 광학 좌표로 옮겨 투영
      const auto T_co = lookup_at(frame, odom_frame_, stamp);
      const int w = color->image.cols, h = color->image.rows;
      auto in_view = [&](const Vec3 & p) {
        if (!T_co) {return false;}
        const tf2::Vector3 q = *T_co * tf2::Vector3(p.x, p.y, p.z);
        if (q.z() < depth_min_ || q.z() > depth_max_) {return false;}
        const double u = K.fx * q.x() / q.z() + K.ppx, v = K.fy * q.y() / q.z() + K.ppy;
        return u >= 2 && v >= 2 && u < w - 2 && v < h - 2;
      };
      // 가림: 칸 중심 화소의 깊이가 그 칸까지 거리보다 occlusion_margin_m 넘게 가까우면 앞에 무엇이 있음
      //       (접은 팔 등). 깊이 영상이 없거나 그 자리 깊이 값이 비면 가림 증거 없음
      cv_bridge::CvImageConstPtr dimg;
      const double dscale = f.depth ? depth_scale_for_encoding(f.depth->encoding) : 0.0;
      if (dscale > 0.0 && f.depth->width == static_cast<uint32_t>(w) && f.depth->height == static_cast<uint32_t>(h)) {
        try {
          dimg = cv_bridge::toCvShare(f.depth);
        } catch (const cv_bridge::Exception &) {
          dimg.reset();
        }
      }
      auto occluded = [&](const SignalLightLocator::Segments & segs) {
        if (!T_co || !dimg) {return false;}
        for (const Vec3 & p : segs) {
          const tf2::Vector3 q = *T_co * tf2::Vector3(p.x, p.y, p.z);
          if (q.z() <= 0.0) {continue;}
          const double u = K.fx * q.x() / q.z() + K.ppx, v = K.fy * q.y() / q.z() + K.ppy;
          if (u < 0 || v < 0 || u >= w || v >= h) {continue;}
          const cv::Point c(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
          const double d = median_depth(dimg->image, c, 2, dscale, depth_min_, depth_max_);
          if (std::isfinite(d) && d < q.z() - occl_margin_) {return true;}
        }
        return false;
      };
      const auto state = locator_.update(t, obs, in_view, occluded);
      any = true;
      last_stamp = stamp;
      if (state) {
        sequence_.update(t, *state);
        last_light_ = *state;
        visible = true;
        signal_cam_ = f.cam == &front_ ? Puck::CAMERA_FRONT : Puck::CAMERA_REAR;
      }
    }
    if (!any) {return;}
    const SignalResult & sr = sequence_.result();
    SignalMsg msg;
    msg.header.stamp = last_stamp;
    msg.header.frame_id = base_frame_;
    // 신호등을 고정하기 전은 SEARCHING, 고정 뒤 아직 점멸을 못 셌으면 COUNTING (값은 SignalResult 와 같음)
    msg.status = SignalMsg::SEARCHING;
    if (locator_.locked()) {
      msg.status = sr.status == SignalResult::SEARCHING ? SignalMsg::COUNTING : static_cast<uint8_t>(sr.status);
    }
    msg.sequence = sr.sequence;
    msg.red = static_cast<uint8_t>(sr.red);
    msg.yellow = static_cast<uint8_t>(sr.yellow);
    msg.green = static_cast<uint8_t>(sr.green);
    msg.cycles = static_cast<uint8_t>(sr.cycles);
    msg.light = static_cast<uint8_t>(last_light_);
    msg.light_visible = visible;
    if (locator_.locked()) {
      if (const auto T_bo = lookup_at(base_frame_, odom_frame_, last_stamp)) {
        const Vec3 o = locator_.position();
        const tf2::Vector3 b = *T_bo * tf2::Vector3(o.x, o.y, o.z);
        msg.position.x = b.x();
        msg.position.y = b.y();
        msg.position.z = b.z();
        msg.position_valid = true;
      }
    }
    msg.t_on = static_cast<float>(sr.t_on);
    msg.t_off = static_cast<float>(sr.t_off);
    msg.t_term = static_cast<float>(sr.t_term);
    msg.camera = signal_cam_;
    signal_pub_->publish(msg);
  }

  // 켜진 등 후보 → 깊이로 크기(등 한 칸) · 높이(바닥에서 등까지) 거른 odom 위치. 깊이가 없으면 후보 없음
  std::vector<LightObs> light_observations(const cv::Mat & bgr, const Image::ConstSharedPtr & depth_msg,
    const PinholeK & K, const std::string & frame, const rclcpp::Time & stamp)
  {
    std::vector<LightObs> out;
    if (!depth_msg) {return out;}
    const double scale = depth_scale_for_encoding(depth_msg->encoding);
    if (scale <= 0.0 || depth_msg->width != static_cast<uint32_t>(bgr.cols) ||
      depth_msg->height != static_cast<uint32_t>(bgr.rows))
    {
      return out;
    }
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception &) {
      return out;
    }
    for (const auto & b : detect_light_blobs(bgr, light_cfg_)) {
      const cv::Point ctr(static_cast<int>(std::lround(b.center.x)), static_cast<int>(std::lround(b.center.y)));
      const double z = blob_median_depth(depth->image, b, scale, depth_min_, depth_max_);
      if (!std::isfinite(z)) {continue;}
      const double expect = (seg_w_ * K.fx / z) * (seg_h_ * K.fy / z);
      if (b.area < area_ratio_min_ * expect || b.area > area_ratio_max_ * expect) {continue;}
      LightObs o;
      Vec3 base;
      if (!to_base_odom(deproject(K, ctr.x, ctr.y, z), frame, stamp, base, o.pos)) {continue;}
      if (base.z < light_z_min_ || base.z > light_z_max_) {continue;}
      o.color = b.color;
      o.area = b.area;
      out.push_back(o);
    }
    return out;
  }

  // ------------------------------ 주사위 확인 ------------------------------
  void on_check_dice(const CheckDice::Request & req, CheckDice::Response & res)
  {
    res.observed = false;
    res.passed = false;
    if (!last_pucks_) {res.message = "최근 퍽 목록 없음 (SEARCH_* 또는 READ_TAG 를 켜고 선반을 본 뒤 부름)"; return;}
    const rclcpp::Time stamp(last_pucks_->header.stamp, get_clock()->get_clock_type());
    if ((now() - stamp).seconds() > dice_max_age_) {res.message = "퍽 목록이 오래됨"; return;}
    tf2::Transform T;   // 칸 좌표 → base_link
    T.setIdentity();
    const std::string frame = req.header.frame_id.empty() ? base_frame_ : req.header.frame_id;
    if (frame != base_frame_) {
      const auto t = lookup_at(base_frame_, frame, stamp);
      if (!t) {res.message = "TF " + frame + " → " + base_frame_ + " 없음"; return;}
      T = *t;
    }
    DiceRequest q;
    q.yellow_shape = req.yellow_shape;
    q.tolerance = req.tolerance > 0 ? req.tolerance : dice_tolerance_;
    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9, z0 = 1e9, z1 = -1e9;
    for (int i = 0; i < 9; ++i) {
      const tf2::Vector3 c = T * tf2::Vector3(req.cells[i].x, req.cells[i].y, req.cells[i].z);
      q.cells[i].x = c.x();
      q.cells[i].y = c.y();
      q.cells[i].expect = req.expect[i];
      x0 = std::min(x0, c.x()); x1 = std::max(x1, c.x());
      y0 = std::min(y0, c.y()); y1 = std::max(y1, c.y());
      z0 = std::min(z0, c.z()); z1 = std::max(z1, c.z());
    }
    // 선반 위 퍽만: 칸 범위 + region_margin, 높이는 선반 윗면 아래 2 cm ~ 위 10 cm (바닥 퍽 · 집게 속 퍽 제외)
    std::vector<SeenPuck> seen;
    for (const auto & p : last_pucks_->pucks) {
      const auto & o = p.pose.position;
      if (o.x < x0 - dice_margin_ || o.x > x1 + dice_margin_ || o.y < y0 - dice_margin_ || o.y > y1 + dice_margin_) {continue;}
      if (o.z < z0 - 0.02 || o.z > z1 + 0.10) {continue;}
      seen.push_back({o.x, o.y, p.color, p.shape});
    }
    const DiceResult r = check_dice(q, seen);
    res.observed = true;
    res.passed = r.passed;
    for (int i = 0; i < 9; ++i) {
      res.error[i] = r.cells[i].error;
      res.count[i] = static_cast<uint8_t>(r.cells[i].count);
      res.offset[i] = static_cast<float>(r.cells[i].offset);
      res.color[i] = r.cells[i].color;
      res.shape[i] = r.cells[i].shape;
    }
    res.wrong.assign(r.wrong.begin(), r.wrong.end());
    res.message = r.passed ? "통과" : "틀린 칸 " + std::to_string(r.wrong.size()) + "개";
  }

  void process_arm(const Image::ConstSharedPtr & img, const CameraInfo::ConstSharedPtr & info, uint8_t mode, Result & r)
  {
    cv_bridge::CvImageConstPtr cv;
    try {
      cv = cv_bridge::toCvShare(img, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "arm 영상 변환 실패: %s", e.what());
      return;
    }
    const cv::Mat u = undistort(cv->image, info);
    if (mode & SetMode::Request::ARM_ALIGN) {
      cv::Mat small = u;
      if (detect_w_ > 0 && u.cols > detect_w_) {
        cv::resize(u, small, cv::Size(detect_w_, u.rows * detect_w_ / u.cols), 0, 0, cv::INTER_AREA);
      }
      const auto dets = detector_->detect(small);
      const int b = best_detection(dets, target_class_);
      if (b >= 0) {
        const cv::Vec3d o = image_offset(dets[b].box, small.size());   // 정규화 값이라 해상도와 무관
        r.offset_valid = true;
        r.arm_offset.x = o[0];
        r.arm_offset.y = o[1];
        r.arm_offset.z = o[2];
        if (!r.object_found) {r.confidence = dets[b].score;}
      }
    }
    const std::string frame = arm_.frame_id.empty() ? img->header.frame_id : arm_.frame_id;
    std::optional<tf2::Transform> T;   // 팔은 움직이므로 촬영 시각 변환. 자세가 없으면(camera_info 없음) 찾지 않는다
    const bool want_marker = mode & MARKER_BIT;
    if (!newK_.empty() && ((mode & SetMode::Request::READ_TAG) || want_marker)) {
      T = lookup_at(base_frame_, frame, rclcpp::Time(img->header.stamp, get_clock()->get_clock_type()));
    }
    if (mode & SetMode::Request::READ_TAG) {
      auto tags = tag_->detect(u, newK_, cv::Mat());
      if (!tag_ids_.empty()) {   // 같은 계열 장소 태그는 크기가 달라 퍽 자세가 틀리므로 뺀다
        tags.erase(std::remove_if(tags.begin(), tags.end(), [this](const TagDetection & t) {
            return std::find(tag_ids_.begin(), tag_ids_.end(), t.id) == tag_ids_.end();}), tags.end());
      }
      const TagDetection * best = nullptr;
      for (const auto & t : tags) {if (!best || t.area > best->area) {best = &t;}}
      if (best) {
        r.tag_found = true;
        r.tag_id = best->id;
        r.tag_pose.header.frame_id = frame;
        r.tag_pose.header.stamp = img->header.stamp;
        if (best->pose_valid) {   // 광학 좌표 그대로 (x 오른쪽 · y 아래 · z 앞)
          cv::Matx33d R;
          cv::Rodrigues(best->rvec, R);
          tf2::Quaternion q;
          to_tf(R).getRotation(q);
          r.tag_pose.pose.position.x = best->tvec[0];
          r.tag_pose.pose.position.y = best->tvec[1];
          r.tag_pose.pose.position.z = best->tvec[2];
          r.tag_pose.pose.orientation = tf2::toMsg(q);
        }
      }
      if (T) {arm_.pucks = pucks_from_tags(tags, u, newK_, *T);}
    }
    if (want_marker && T) {
      arm_.marker = best_marker(marker_->detect(u, newK_, cv::Mat()), *T, img->header.stamp, Puck::CAMERA_ARM);
    }
  }

  // camera_info 로 왜곡 보정. camera_info 가 없으면 원본 그대로이고 태그 자세는 내지 않는다(newK_ 비움)
  cv::Mat undistort(const cv::Mat & img, const CameraInfo::ConstSharedPtr & info)
  {
    if (!info || info->k[0] <= 0.0) {newK_ = cv::Mat(); return img;}
    const bool same = map_info_ && map_info_->k == info->k && map_info_->d == info->d && map_size_ == img.size();
    if (!same) {
      cv::Mat K(3, 3, CV_64F);
      for (int i = 0; i < 9; ++i) {K.at<double>(i / 3, i % 3) = info->k[i];}
      // camera_info 해상도와 받은 영상 해상도가 다르면 비율로 맞춘다
      const double sx = info->width ? static_cast<double>(img.cols) / info->width : 1.0;
      const double sy = info->height ? static_cast<double>(img.rows) / info->height : 1.0;
      K.at<double>(0, 0) *= sx; K.at<double>(0, 2) *= sx; K.at<double>(1, 1) *= sy; K.at<double>(1, 2) *= sy;
      cv::Mat D(1, static_cast<int>(info->d.size()), CV_64F);
      bool distorted = false;
      for (size_t i = 0; i < info->d.size(); ++i) {D.at<double>(0, static_cast<int>(i)) = info->d[i]; distorted |= info->d[i] != 0.0;}
      map_info_ = info;
      map_size_ = img.size();
      if (!distorted) {
        newK_ = K;
        map1_ = cv::Mat();
      } else {
        newK_ = cv::getOptimalNewCameraMatrix(K, D, img.size(), 0.0);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK_, img.size(), CV_16SC2, map1_, map2_);
      }
    }
    if (map1_.empty()) {return img;}
    cv::Mat out;
    cv::remap(img, out, map1_, map2_, cv::INTER_LINEAR);
    return out;
  }

  // 파라미터
  std::string base_frame_, odom_frame_;
  double depth_min_, depth_max_, frame_max_age_;
  int depth_win_, detect_w_, target_class_;
  int64_t depth_tol_ns_;
  double size_min_m_ = 0.015, size_max_m_ = 0.09;
  rclcpp::QoS image_qos_{rclcpp::KeepLast(2)};

  std::unique_ptr<Detector> detector_;
  std::unique_ptr<TagDetector> tag_, marker_;
  std::vector<int> marker_ids_, tag_ids_;
  std::vector<HsvRange> hsv_;
  int blur_ksize_ = 3, morph_kernel_ = 3;
  float score_threshold_ = 0.3f;
  bool split_on_ = true;
  DepthSplitConfig split_;
  PuckGeometry puck_geom_;
  double merge_dist_ = 0.04, min_confidence_ = 0.5;
  double upright_gap_px_ = 1.0;
  TargetTracker tracker_;
  uint8_t mode_ = SetMode::Request::OFF;   // ctl_cbg_ 에서만 읽고 쓴다
  bool restart_seen_ = false;
  DepthCamera front_, rear_;
  MonoCamera arm_;
  CameraInfo::ConstSharedPtr map_info_;
  cv::Mat map1_, map2_, newK_;
  cv::Size map_size_;

  // 신호등 · 주사위
  LightBlobConfig light_cfg_;
  double seg_w_ = 0.0396, seg_h_ = 0.0349, area_ratio_min_ = 0.15, area_ratio_max_ = 2.5;
  double occl_margin_ = 0.10;   // [m] 칸 앞을 가렸다고 보는 깊이 차
  double light_z_min_ = 0.05, light_z_max_ = 0.20;
  size_t signal_q_max_ = 30;
  SignalLightLocator locator_;
  SignalSequenceTracker sequence_;
  double signal_last_t_ = -1;
  Light last_light_ = Light::Off;
  uint8_t signal_cam_ = Puck::CAMERA_FRONT;
  std::atomic<bool> signal_on_{false};   // 영상 콜백(cam_cbg_)이 읽는다
  std::optional<PuckArray> last_pucks_;
  double dice_tolerance_ = 0.015, dice_margin_ = 0.04, dice_max_age_ = 1.0;

  std::unique_ptr<TfHelper> tf_;
  rclcpp::CallbackGroup::SharedPtr cam_cbg_, ctl_cbg_;
  rclcpp::Publisher<Result>::SharedPtr pub_;
  rclcpp::Publisher<PuckArray>::SharedPtr puck_pub_;
  rclcpp::Publisher<TaskMarker>::SharedPtr marker_pub_;
  rclcpp::Publisher<SignalMsg>::SharedPtr signal_pub_;
  rclcpp::Service<SetMode>::SharedPtr srv_;
  rclcpp::Service<CheckDice>::SharedPtr dice_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 바코드 (READ_BARCODE). 일꾼 future 는 맨 뒤에 두어 소멸 때 먼저 끝을 기다린다(일꾼이 위 멤버를 씀)
  struct DepthAverage
  {
    cv::Mat sum, cnt;              // 화소별 깊이 합 [m] (CV_64F) · 값이 있던 장 수 (CV_32S)
    int n = 0;
    int64_t first_ns = -1, last_ns = -1;
    std::optional<tf2::Transform> odom0;   // 첫 장 촬영 때 odom ← base_link
  };
  std::unique_ptr<BarcodeLocator> bc_locator_;
  std::unique_ptr<BarcodeReader> bc_reader_;   // bc_read_job_ 일꾼에서만 쓴다
  rclcpp::Publisher<BarcodeMsg>::SharedPtr barcode_pub_;
  MonoCamera bc_arm_;                          // 바코드 읽기용 팔 카메라 구독 (영상 · 처리 기록이 arm_ 과 따로)
  DepthCamera * bc_cam_ = nullptr;
  int bc_frames_ = 5, bc_min_valid_ = 0;
  std::string bc_base_frame_ = "base_footprint";
  double bc_max_move_ = 0.01, bc_max_rot_ = 1.0;   // [m] · [°] (생성자에서 rad)
  DepthAverage bc_acc_;                        // ctl_cbg_ 에서만
  std::atomic<bool> barcode_on_{false};        // 깊이 콜백(cam_cbg_)이 읽는다
  std::atomic<uint64_t> bc_gen_{0};            // 모드를 켜고 끌 때마다 늘림: 그 전에 시작한 일꾼 결과는 내지 않는다
  std::future<void> bc_locate_job_, bc_read_job_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::PerceptionNode)
