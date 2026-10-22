// PerceptionNode (perception_container)
//   카메라 입력은 모두 ROS 영상 토픽이다. 시뮬레이션은 Isaac Sim, 실물은 realsense2_camera(D435 × 2)와
//   UVC 드라이버(Arducam)가 같은 토픽 이름으로 발행한다. 그래서 실물과 시뮬레이션이 같은 코드로 돈다.
//   드라이버를 같은 컨테이너에 넣으면 intra-process 로 영상을 복사 없이 받는다.
//
//   /perception/set_mode (서비스): 비트 SEARCH_FRONT 1 · ARM_ALIGN 2 · READ_TAG 4 · SEARCH_REAR 8, 0 = 전부 정지
//      켜진 비트의 카메라 토픽만 구독한다. OFF 를 받으면 대상 추적을 초기화하고 mode=0 result 를 한 번 발행한다
//      (받는 쪽 초기화 신호).
//   /perception/result (토픽): publish_rate 마다 확인해, 새 영상이 하나라도 들어왔을 때만 발행한다
//      stamp = 결과에 쓴 영상 중 가장 오래된 촬영 시각 (받는 쪽 300 ms 규칙의 기준)
//      쓸 수 있는 영상이 하나도 없으면 발행하지 않는다 (카메라 정지 = 침묵, 받는 쪽은 300 ms 뒤 "모름")
//   영상 시각 규칙: 같은·과거 stamp(재전송)는 버린다. 1 s 넘게 뒤로 가면 시각 재시작으로 보고 추적 상태를 초기화한다
//   SEARCH_* : 검출 → 정렬 깊이 중앙값 → 컬러 광학 좌표 3D → TF 로 base_link · odom → TargetTracker → object_pose
//   ARM_ALIGN: camera_info 로 왜곡 보정한 영상을 detect_width 로 줄여 검출 → 화면 중심 대비 arm_offset
//   READ_TAG : 왜곡 보정 원본에서 AprilTag → tag_id · tag_pose (Arducam 광학 프레임 기준, x 오른쪽 · y 아래 · z 앞)
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/names.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "perception/detector.hpp"
#include "perception/frame_gate.hpp"
#include "perception/geometry.hpp"
#include "perception/tag_detector.hpp"
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
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

namespace
{
int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1'000'000'000LL + t.nanosec;
}
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

    TrackerConfig tc;
    auto ex_min = declare_parameter("target.exclude_min", std::vector<double>{tc.exclude_min.x, tc.exclude_min.y, tc.exclude_min.z});
    auto ex_max = declare_parameter("target.exclude_max", std::vector<double>{tc.exclude_max.x, tc.exclude_max.y, tc.exclude_max.z});
    if (ex_min.size() == 3) {tc.exclude_min = {ex_min[0], ex_min[1], ex_min[2]};}
    if (ex_max.size() == 3) {tc.exclude_max = {ex_max[0], ex_max[1], ex_max[2]};}
    tc.min_confidence = declare_parameter("target.min_confidence", tc.min_confidence);
    tc.gate = declare_parameter("target.gate", tc.gate);
    tc.lost_timeout = declare_parameter("target.lost_timeout", tc.lost_timeout);
    tracker_ = TargetTracker(tc);
    size_min_m_ = declare_parameter("target.size_min_m", 0.015);
    size_max_m_ = declare_parameter("target.size_max_m", 0.09);

    tag_ = std::make_unique<TagDetector>(declare_parameter("tag.family", std::string("tag36h11")),
        declare_parameter("tag.size", 0.10));

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
    RCLCPP_INFO(get_logger(), "검출기: %s %s · 태그 %s", detector_->name().c_str(), info.c_str(),
      tag_->ok() ? "사용 가능" : "패밀리 오류");

    tf_ = std::make_unique<TfHelper>(*this);

    // 영상 콜백은 받아 두기만 한다(병렬). 모드 변경과 처리는 한 그룹에서 차례로 돌아 구독 생성·삭제와 겹치지 않는다
    cam_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ctl_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pub_ = create_publisher<Result>(names::PERCEPTION_RESULT, qos::result());
    srv_ = create_service<SetMode>(names::PERCEPTION_SET_MODE,
        [this](SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res) {on_set_mode(*req, *res);},
        rclcpp::ServicesQoS(), ctl_cbg_);
    timer_ = create_timer(std::chrono::duration<double>(1.0 / rate), [this]() {process();}, ctl_cbg_);
  }

private:
  static constexpr uint8_t SEARCH_BITS = SetMode::Request::SEARCH_FRONT | SetMode::Request::SEARCH_REAR;
  static constexpr uint8_t ARM_BITS = SetMode::Request::ARM_ALIGN | SetMode::Request::READ_TAG;

  // ------------------------------ set_mode ------------------------------
  void on_set_mode(const SetMode::Request & req, SetMode::Response & res)
  {
    if (req.mode & ~(SEARCH_BITS | ARM_BITS)) {res.success = false; res.message = "모르는 모드 비트"; return;}
    if ((req.mode & SetMode::Request::SEARCH_FRONT) && front_.color_topic.empty()) {
      res.success = false; res.message = "전방 카메라 토픽 미설정"; return;
    }
    if ((req.mode & SetMode::Request::SEARCH_REAR) && rear_.color_topic.empty()) {
      res.success = false; res.message = "후방 카메라 토픽 미설정"; return;
    }
    if ((req.mode & ARM_BITS) && arm_.image_topic.empty()) {res.success = false; res.message = "팔 카메라 토픽 미설정"; return;}
    if ((req.mode & SetMode::Request::READ_TAG) && !tag_->ok()) {res.success = false; res.message = "태그 패밀리 오류"; return;}

    const uint8_t old = mode_;
    mode_ = req.mode;
    update_subscriptions();
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
    set_depth_subscribed(front_, mode_ & front_.bit);
    set_depth_subscribed(rear_, mode_ & rear_.bit);
    set_mono_subscribed(arm_, mode_ & ARM_BITS);
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
          [&c](Image::ConstSharedPtr m) {std::lock_guard<std::mutex> lk(c.m); c.depth.push(stamp_ns(m->header.stamp), m);}, o);
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
    if (g == GateResult::Restart) {c.depth.clear();}   // 재시작 전 시각의 깊이와 짝짓지 않게
    c.color = m;
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

  void collect(DepthCamera & c, const DepthSnapshot & s, int cam_idx, std::vector<Candidate> & out)
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
    cv_bridge::CvImageConstPtr color, depth;
    try {
      color = cv_bridge::toCvShare(s.color, "bgr8");
      depth = cv_bridge::toCvShare(s.depth);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 영상 변환 실패: %s", c.name.c_str(), e.what());
      return;
    }
    const PinholeK K{s.info->k[0], s.info->k[4], s.info->k[2], s.info->k[5]};
    const std::string frame = c.frame_id.empty() ? s.color->header.frame_id : c.frame_id;
    const rclcpp::Time stamp(s.color->header.stamp, get_clock()->get_clock_type());
    for (const auto & d : detector_->detect(color->image)) {
      if (target_class_ >= 0 && d.class_id != target_class_) {continue;}
      const cv::Point ctr(static_cast<int>(d.box.x + d.box.width / 2), static_cast<int>(d.box.y + d.box.height / 2));
      const double z = median_depth(depth->image, ctr, depth_win_, scale, depth_min_, depth_max_);
      if (!std::isfinite(z)) {continue;}
      if (size_min_m_ > 0 && !size_plausible(d.box, z, K, size_min_m_, size_max_m_)) {continue;}
      Candidate k;
      k.confidence = d.score;
      k.camera = cam_idx;
      if (to_base_odom(deproject(K, ctr.x, ctr.y, z), frame, stamp, k.base, k.odom)) {out.push_back(k);}
    }
  }

  void process()
  {
    const uint8_t mode = mode_;
    if (mode == SetMode::Request::OFF) {return;}
    const rclcpp::Time now_t = now();
    Result r;
    r.mode = mode;
    std::optional<rclcpp::Time> oldest;
    bool any_fresh = false;
    auto note = [&](const builtin_interfaces::msg::Time & st, bool fresh) {
      const rclcpp::Time t(st, now_t.get_clock_type());
      if (!oldest || t < *oldest) {oldest = t;}
      any_fresh = any_fresh || fresh;
    };

    if (mode & SEARCH_BITS) {
      std::vector<Candidate> cands;
      bool have = false;
      for (auto * c : {&front_, &rear_}) {
        if (!(mode & c->bit)) {continue;}
        if (auto s = take(*c, now_t)) {
          note(s->color->header.stamp, s->fresh);
          have = true;
          if (s->fresh) {collect(*c, *s, c == &front_ ? 0 : 1, cands);}
        }
      }
      if (restart_seen_) {tracker_.reset(); restart_seen_ = false;}
      if (have && any_fresh) {
        if (auto t = tracker_.update(cands, oldest->seconds())) {
          r.object_found = true;
          r.object_pose.header.frame_id = base_frame_;
          r.object_pose.header.stamp = *oldest;
          r.object_pose.pose.position.x = t->base.x;
          r.object_pose.pose.position.y = t->base.y;
          r.object_pose.pose.position.z = t->base.z;
          r.object_pose.pose.orientation.w = 1.0;
          r.confidence = static_cast<float>(t->confidence);
        }
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
        note(img->header.stamp, fresh);
        if (fresh) {process_arm(img, info, mode, r);}
      }
    }

    if (!oldest) {   // 쓸 수 있는 영상이 하나도 없음 → 발행하지 않음 (받는 쪽은 300 ms 규칙으로 "모름")
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAMERA_STALL: 모드 0x%02x 에 쓸 수 있는 영상 없음", mode);
      return;
    }
    if (!any_fresh) {return;}   // 새 영상이 없으면 같은 결과를 다시 내지 않는다
    r.header.stamp = *oldest;
    r.header.frame_id = base_frame_;
    pub_->publish(r);
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
    if (mode & SetMode::Request::READ_TAG) {
      const auto tags = tag_->detect(u, newK_, cv::Mat());
      const TagDetection * best = nullptr;
      for (const auto & t : tags) {if (!best || t.area > best->area) {best = &t;}}
      if (!best) {return;}
      r.tag_found = true;
      r.tag_id = best->id;
      r.tag_pose.header.frame_id = arm_.frame_id.empty() ? img->header.frame_id : arm_.frame_id;
      r.tag_pose.header.stamp = img->header.stamp;
      if (best->pose_valid) {   // 광학 좌표 그대로 (x 오른쪽 · y 아래 · z 앞)
        cv::Matx33d R;
        cv::Rodrigues(best->rvec, R);
        tf2::Matrix3x3 m(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
        tf2::Quaternion q;
        m.getRotation(q);
        r.tag_pose.pose.position.x = best->tvec[0];
        r.tag_pose.pose.position.y = best->tvec[1];
        r.tag_pose.pose.position.z = best->tvec[2];
        r.tag_pose.pose.orientation = tf2::toMsg(q);
      }
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
  std::unique_ptr<TagDetector> tag_;
  TargetTracker tracker_;
  uint8_t mode_ = SetMode::Request::OFF;   // ctl_cbg_ 에서만 읽고 쓴다
  bool restart_seen_ = false;
  DepthCamera front_, rear_;
  MonoCamera arm_;
  CameraInfo::ConstSharedPtr map_info_;
  cv::Mat map1_, map2_, newK_;
  cv::Size map_size_;

  std::unique_ptr<TfHelper> tf_;
  rclcpp::CallbackGroup::SharedPtr cam_cbg_, ctl_cbg_;
  rclcpp::Publisher<Result>::SharedPtr pub_;
  rclcpp::Service<SetMode>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::PerceptionNode)
