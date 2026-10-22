// PerceptionNode (Perception 팀 · ⑦ perception_container)
//   /perception/set_mode (서비스): 비트 플래그 SEARCH_FRONT 1 · ARM_ALIGN 2 · READ_TAG 4 · SEARCH_REAR 8, 0 = 전부 정지
//      켜진 비트의 카메라만 연다. OFF 를 받으면 대상 추적을 초기화하고 mode=0 result 를 한 번 발행 (받는 쪽 초기화 신호)
//   /perception/result (토픽, 15 Hz 이상, 활성 모드에서만) — stamp = 촬영 시각(ROS 시계), 받는 쪽 300 ms 규칙 기준
//   카메라는 토픽 없이 직접 연다 (영상 직렬화 · 복사 없음):
//     D435 × 2 : rs2::pipeline + rs2::align(color), global time → ROS 시계 stamp — 카메라마다 캡처 스레드
//     Arducam  : cv::VideoCapture(/dev/arm_cam, V4L2, MJPG, BUFFERSIZE=1) — 캡처 스레드
//   SEARCH_* : 검출 → 깊이 중앙값 → 3D → base_link · odom 변환 → TargetTracker(물체 하나) → object_pose(base_link)
//   ARM_ALIGN: 왜곡 보정 영상을 detect_width 로 줄여 검출 → 화면 중심 대비 arm_offset (부하 절감)
//   READ_TAG : 왜곡 보정 원본 해상도에서 AprilTag(tag36h11) → tag_id · tag_pose(arm_camera_link)
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef USE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "perception/detector.hpp"
#include "perception/geometry.hpp"
#include "perception/tag_detector.hpp"
#include "perception/target_tracker.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;
using SetMode = robot_interfaces::srv::SetPerceptionMode;
using Result = robot_interfaces::msg::PerceptionResult;

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception_node", options)
  {
    front_serial_ = declare_parameter("d435.front_serial", std::string(""));
    rear_serial_ = declare_parameter("d435.rear_serial", std::string(""));
    rs_w_ = declare_parameter("d435.width", 640);
    rs_h_ = declare_parameter("d435.height", 480);
    rs_fps_ = declare_parameter("d435.fps", 15);
    depth_min_ = declare_parameter("d435.depth_min", 0.15);
    depth_max_ = declare_parameter("d435.depth_max", 4.0);
    depth_win_ = declare_parameter("d435.depth_window", 3);

    arm_dev_ = declare_parameter("arm_cam.device", std::string("/dev/arm_cam"));
    arm_w_ = declare_parameter("arm_cam.width", 1280);
    arm_h_ = declare_parameter("arm_cam.height", 720);
    arm_fps_ = declare_parameter("arm_cam.fps", 30);
    detect_w_ = declare_parameter("arm_cam.detect_width", 640);
    arm_K_ = declare_parameter("arm_cam.camera_matrix", std::vector<double>(9, 0.0));
    arm_D_ = declare_parameter("arm_cam.dist_coeffs", std::vector<double>(5, 0.0));

    DetectorConfig dc;
    dc.type = declare_parameter("detector.type", dc.type);
    dc.engine_path = declare_parameter("detector.engine_path", dc.engine_path);
    dc.score_threshold = static_cast<float>(declare_parameter("detector.score_threshold", 0.3));
    dc.nms_threshold = static_cast<float>(declare_parameter("detector.nms_threshold", 0.45));
    dc.target_class = declare_parameter("detector.target_class", -1);
    dc.target_color = declare_parameter("detector.target_color", dc.target_color);
    dc.min_area_ratio = declare_parameter("detector.min_area_ratio", dc.min_area_ratio);
    dc.blur_ksize = declare_parameter("detector.blur_ksize", dc.blur_ksize);
    dc.morph_kernel = declare_parameter("detector.morph_kernel", dc.morph_kernel);
    dc.min_fill_ratio = declare_parameter("detector.min_fill_ratio", dc.min_fill_ratio);
    {   // HSV 범위: detector.hsv.<key>.lower/upper = [H, S, V] — 조명 바뀌면 yaml 만 고침
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
    size_min_m_ = declare_parameter("target.size_min_m", 0.015);   // 퍽 30×30×60 mm — 0 이면 크기 검증 끔
    size_max_m_ = declare_parameter("target.size_max_m", 0.09);
    rs_white_balance_ = declare_parameter("d435.white_balance", 4600.0);   // K, ≤0 = 자동
    rs_exposure_ = declare_parameter("d435.exposure", 0.0);                // µs, ≤0 = 자동

    tag_ = std::make_unique<TagDetector>(declare_parameter("tag.family", std::string("tag36h11")),
        declare_parameter("tag.size", 0.10));   // ★ 실제 태그 한 변 (m)

    rate_ = declare_parameter("publish_rate", 15.0);
    frame_max_age_ = declare_parameter("frame_max_age_s", 0.3);
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    odom_frame_ = declare_parameter("frames.odom", std::string("odom"));
    front_frame_ = declare_parameter("frames.front", std::string("camera_front_link"));
    rear_frame_ = declare_parameter("frames.rear", std::string("camera_rear_link"));
    arm_frame_ = declare_parameter("frames.arm", std::string("arm_camera_link"));

    std::string info;
    detector_ = make_detector(dc, info);
    if (!detector_) {
      RCLCPP_FATAL(get_logger(), "검출기 생성 실패: %s", info.c_str());
      throw std::runtime_error("detector");
    }
    RCLCPP_INFO(get_logger(), "검출기: %s %s · 태그 %s", detector_->name().c_str(), info.c_str(),
      tag_->ok() ? "tag36h11 계열" : "패밀리 오류");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<Result>(names::PERCEPTION_RESULT, qos::result());
    srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = create_service<SetMode>(names::PERCEPTION_SET_MODE,
        [this](SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res) {on_set_mode(*req, *res);},
        rclcpp::ServicesQoS(), srv_cbg_);

#ifdef USE_REALSENSE
    threads_.emplace_back([this]() {rs_loop(front_serial_, SetMode::Request::SEARCH_FRONT, front_, "전방");});
    if (!rear_serial_.empty() && !front_serial_.empty()) {
      threads_.emplace_back([this]() {rs_loop(rear_serial_, SetMode::Request::SEARCH_REAR, rear_, "후방");});
    } else {
      RCLCPP_WARN(get_logger(), "D435 두 대를 쓰려면 d435.front_serial · rear_serial 을 모두 지정 — SEARCH_REAR 비활성");
    }
#else
    RCLCPP_WARN(get_logger(), "librealsense2 없이 빌드됨 — SEARCH_FRONT · SEARCH_REAR 를 쓸 수 없음");
#endif
    threads_.emplace_back([this]() {arm_loop();});
    threads_.emplace_back([this]() {worker_loop();});
  }

  ~PerceptionNode() override
  {
    running_ = false;
    for (auto & t : threads_) {if (t.joinable()) {t.join();}}
  }

private:
  struct DepthSlot
  {
    std::mutex m;
    cv::Mat color, depth;
    double depth_scale = 0.001;
    PinholeK K;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };
  struct ImageSlot
  {
    std::mutex m;
    cv::Mat image;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };
  static constexpr uint8_t SEARCH_BITS = SetMode::Request::SEARCH_FRONT | SetMode::Request::SEARCH_REAR;
  static constexpr uint8_t ARM_BITS = SetMode::Request::ARM_ALIGN | SetMode::Request::READ_TAG;

  // ------------------------------ set_mode ------------------------------
  void on_set_mode(const SetMode::Request & req, SetMode::Response & res)
  {
    const uint8_t all = SEARCH_BITS | ARM_BITS;
    if (req.mode & ~all) {res.success = false; res.message = "모르는 모드 비트"; return;}
#ifndef USE_REALSENSE
    if (req.mode & SEARCH_BITS) {res.success = false; res.message = "librealsense2 없이 빌드되어 SEARCH 불가"; return;}
#endif
    if ((req.mode & SetMode::Request::SEARCH_REAR) && (rear_serial_.empty() || front_serial_.empty())) {
      res.success = false;
      res.message = "후방 D435 미설정 (d435.rear_serial)";
      return;
    }
    if ((req.mode & SetMode::Request::READ_TAG) && !tag_->ok()) {res.success = false; res.message = "태그 패밀리 오류"; return;}
    const uint8_t old = mode_.exchange(req.mode);
    if (req.mode == SetMode::Request::OFF) {
      reset_tracker_ = true;
      Result r;   // 받는 쪽 초기화 신호 (manipulator 기억 삭제)
      r.header.stamp = now();
      r.mode = 0;
      pub_->publish(r);
    }
    res.success = true;
    res.message = "모드 " + std::to_string(old) + " → " + std::to_string(req.mode);
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  // ------------------------------ D435 ------------------------------
#ifdef USE_REALSENSE
  // 색 검출은 화이트밸런스가 흔들리면 HSV 범위를 벗어난다 → 고정 (Xmen REALSENSE.md 와 같은 방침)
  void apply_color_options(rs2::color_sensor s, const std::string & name)
  {
    try {
      if (s.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
        s.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, rs_white_balance_ > 0 ? 0.f : 1.f);
        if (rs_white_balance_ > 0) {s.set_option(RS2_OPTION_WHITE_BALANCE, static_cast<float>(rs_white_balance_));}
      }
      if (s.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
        s.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, rs_exposure_ > 0 ? 0.f : 1.f);
        if (rs_exposure_ > 0) {s.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(rs_exposure_));}
      }
    } catch (const rs2::error & e) {
      RCLCPP_WARN(get_logger(), "D435 %s 색 옵션 설정 실패: %s", name.c_str(), e.what());
    }
  }

  void rs_loop(const std::string & serial, uint8_t bit, DepthSlot & slot, const std::string & name)
  {
    rs2::pipeline pipe;
    rs2::align align(RS2_STREAM_COLOR);
    bool started = false;
    while (running_) {
      if (!(mode_.load() & bit)) {
        if (started) {pipe.stop(); started = false; RCLCPP_INFO(get_logger(), "D435 %s 정지", name.c_str());}
        std::this_thread::sleep_for(50ms);
        continue;
      }
      try {
        if (!started) {
          rs2::config cfg;
          if (!serial.empty()) {cfg.enable_device(serial);}
          cfg.enable_stream(RS2_STREAM_COLOR, rs_w_, rs_h_, RS2_FORMAT_BGR8, rs_fps_);
          cfg.enable_stream(RS2_STREAM_DEPTH, rs_w_, rs_h_, RS2_FORMAT_Z16, rs_fps_);
          auto prof = pipe.start(cfg);
          for (auto && s : prof.get_device().query_sensors()) {   // 장치 시계 → 호스트(ROS) 시계
            if (s.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {s.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);}
          }
          apply_color_options(prof.get_device().first<rs2::color_sensor>(), name);
          const auto intr = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
          std::lock_guard<std::mutex> lk(slot.m);
          slot.depth_scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
          slot.K = {intr.fx, intr.fy, intr.ppx, intr.ppy};
          started = true;
          RCLCPP_INFO(get_logger(), "D435 %s 시작 (%s)", name.c_str(), serial.empty() ? "첫 장치" : serial.c_str());
        }
        rs2::frameset fs;
        if (!pipe.try_wait_for_frames(&fs, 1000)) {continue;}
        fs = align.process(fs);
        const auto c = fs.get_color_frame();
        const auto d = fs.get_depth_frame();
        if (!c || !d) {continue;}
        rclcpp::Time stamp = now();
        if (c.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
          stamp = rclcpp::Time(static_cast<int64_t>(c.get_timestamp() * 1e6), RCL_ROS_TIME);
        }
        cv::Mat color(cv::Size(c.get_width(), c.get_height()), CV_8UC3, const_cast<void *>(c.get_data()));
        cv::Mat depth(cv::Size(d.get_width(), d.get_height()), CV_16U, const_cast<void *>(d.get_data()));
        std::lock_guard<std::mutex> lk(slot.m);
        slot.color = color.clone();
        slot.depth = depth.clone();
        slot.stamp = stamp;
      } catch (const rs2::error & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "D435 %s 오류: %s", name.c_str(), e.what());
        if (started) {try {pipe.stop();} catch (...) {}}
        started = false;
        std::this_thread::sleep_for(1s);
      }
    }
    if (started) {try {pipe.stop();} catch (...) {}}
  }
#endif

  // ------------------------------ Arducam ------------------------------
  void arm_loop()
  {
    cv::VideoCapture cap;
    while (running_) {
      if (!(mode_.load() & ARM_BITS)) {
        if (cap.isOpened()) {cap.release(); RCLCPP_INFO(get_logger(), "Arducam 정지");}
        std::this_thread::sleep_for(50ms);
        continue;
      }
      if (!cap.isOpened()) {
        if (!cap.open(arm_dev_, cv::CAP_V4L2)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 열기 실패", arm_dev_.c_str());
          std::this_thread::sleep_for(1s);
          continue;
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, arm_w_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, arm_h_);
        cap.set(cv::CAP_PROP_FPS, arm_fps_);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);   // 지연 방지
        RCLCPP_INFO(get_logger(), "Arducam 시작 %s", arm_dev_.c_str());
      }
      cv::Mat f;
      if (!cap.grab()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Arducam 프레임 없음 — 다시 연결");
        cap.release();
        continue;
      }
      const rclcpp::Time stamp = now();   // grab 직후 = 촬영 시각에 가장 가까움 (BUFFERSIZE=1)
      if (!cap.retrieve(f) || f.empty()) {continue;}
      std::lock_guard<std::mutex> lk(arm_.m);
      arm_.image = f;
      arm_.stamp = stamp;
    }
  }

  // ------------------------------ 처리 (15 Hz) ------------------------------
  void worker_loop()
  {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / rate_));
    auto next = std::chrono::steady_clock::now();
    while (running_) {
      next += period;
      if (reset_tracker_.exchange(false)) {tracker_.reset();}
      const uint8_t mode = mode_.load();
      if (mode != SetMode::Request::OFF) {process(mode);}
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() > next + 1s) {next = std::chrono::steady_clock::now();}
    }
  }

  // 카메라 링크 점 → base_link · odom (정적 TF 는 최신값, odom 은 촬영 시각)
  bool to_base_odom(const cv::Point3d & p, const std::string & cam, const rclcpp::Time & stamp, Vec3 & base, Vec3 & odom)
  {
    geometry_msgs::msg::PointStamped in, b, o;
    in.header.frame_id = cam;
    in.point.x = p.x;
    in.point.y = p.y;
    in.point.z = p.z;
    try {
      tf_buffer_->transform(in, b, base_frame_, tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s→%s 없음: %s", cam.c_str(), base_frame_.c_str(), e.what());
      return false;
    }
    base = {b.point.x, b.point.y, b.point.z};
    odom = base;
    try {
      b.header.stamp = stamp;
      if (!tf_buffer_->canTransform(odom_frame_, base_frame_, tf2_ros::fromRclcpp(stamp))) {b.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);}
      tf_buffer_->transform(b, o, odom_frame_, tf2::durationFromSec(0.0));
      odom = {o.point.x, o.point.y, o.point.z};
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "TF %s→%s 없음 — base_link 로 추적", odom_frame_.c_str(), base_frame_.c_str());
    }
    return true;
  }

  void collect(DepthSlot & slot, const std::string & cam, int cam_idx, std::vector<Candidate> & out, std::optional<rclcpp::Time> & oldest)
  {
    cv::Mat color, depth;
    PinholeK K;
    double scale;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lk(slot.m);
      if (slot.color.empty() || (now() - slot.stamp).seconds() > frame_max_age_) {return;}
      color = slot.color;
      depth = slot.depth;
      K = slot.K;
      scale = slot.depth_scale;
      stamp = slot.stamp;
    }
    if (!oldest || stamp < *oldest) {oldest = stamp;}
    for (const auto & d : detector_->detect(color)) {
      if (target_class_ >= 0 && d.class_id != target_class_) {continue;}
      const cv::Point c(static_cast<int>(d.box.x + d.box.width / 2), static_cast<int>(d.box.y + d.box.height / 2));
      const double z = median_depth(depth, c, depth_win_, scale, depth_min_, depth_max_);
      if (!std::isfinite(z)) {continue;}
      if (size_min_m_ > 0 && !size_plausible(d.box, z, K, size_min_m_, size_max_m_)) {continue;}
      Candidate k;
      k.confidence = d.score;
      k.camera = cam_idx;
      if (to_base_odom(optical_to_link(deproject(K, c.x, c.y, z)), cam, stamp, k.base, k.odom)) {out.push_back(k);}
    }
  }

  void process(uint8_t mode)
  {
    Result r;
    r.mode = mode;
    std::optional<rclcpp::Time> oldest;

    if (mode & SEARCH_BITS) {
      std::vector<Candidate> cands;
      if (mode & SetMode::Request::SEARCH_FRONT) {collect(front_, front_frame_, 0, cands, oldest);}
      if (mode & SetMode::Request::SEARCH_REAR) {collect(rear_, rear_frame_, 1, cands, oldest);}
      if (oldest) {
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
      cv::Mat img;
      rclcpp::Time stamp;
      {
        std::lock_guard<std::mutex> lk(arm_.m);
        if (!arm_.image.empty() && (now() - arm_.stamp).seconds() <= frame_max_age_) {img = arm_.image; stamp = arm_.stamp;}
      }
      if (!img.empty()) {
        if (!oldest || stamp < *oldest) {oldest = stamp;}
        const cv::Mat u = undistort(img);
        if (mode & SetMode::Request::ARM_ALIGN) {
          cv::Mat small = u;
          if (detect_w_ > 0 && u.cols > detect_w_) {
            cv::resize(u, small, cv::Size(detect_w_, u.rows * detect_w_ / u.cols), 0, 0, cv::INTER_AREA);
          }
          const auto dets = detector_->detect(small);
          const int b = best_detection(dets, target_class_);
          if (b >= 0) {
            const cv::Vec3d o = image_offset(dets[b].box, small.size());   // 정규화 → 해상도와 무관
            r.offset_valid = true;
            r.arm_offset.x = o[0];
            r.arm_offset.y = o[1];
            r.arm_offset.z = o[2];
            if (!(mode & SEARCH_BITS) || !r.object_found) {r.confidence = dets[b].score;}
          }
        }
        if (mode & SetMode::Request::READ_TAG) {read_tag(u, stamp, r);}
      }
    }
    if (!oldest) {   // 쓸 수 있는 프레임이 하나도 없음 → 발행하지 않음 (Xmen: 멈추면 안 보냄)
      // 받는 쪽은 결과가 300 ms 넘게 오래되면 "모름"으로 처리 — "안 보임" 과 구별된다
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAMERA_STALL: 모드 0x%02x 에 쓸 수 있는 프레임 없음", mode);
      return;
    }
    r.header.stamp = *oldest;
    r.header.frame_id = base_frame_;
    if (mode_.load() == mode) {pub_->publish(r);}   // 그새 모드가 바뀌었으면 버림
  }

  void read_tag(const cv::Mat & u, const rclcpp::Time & stamp, Result & r)
  {
    const auto tags = tag_->detect(u, newK_, cv::Mat());
    const TagDetection * best = nullptr;
    for (const auto & t : tags) {if (!best || t.area > best->area) {best = &t;}}
    if (!best) {return;}
    r.tag_found = true;
    r.tag_id = best->id;
    r.tag_pose.header.frame_id = arm_frame_;
    r.tag_pose.header.stamp = stamp;
    if (best->pose_valid) {
      // 광학 → 링크 (x 앞 · y 왼 · z 위): link = C · opt
      const cv::Matx33d C(0, 0, 1, -1, 0, 0, 0, -1, 0);
      cv::Matx33d Ro;
      cv::Rodrigues(best->rvec, Ro);
      const cv::Matx33d Rl = C * Ro;
      const cv::Vec3d tl = C * best->tvec;
      tf2::Matrix3x3 m(Rl(0, 0), Rl(0, 1), Rl(0, 2), Rl(1, 0), Rl(1, 1), Rl(1, 2), Rl(2, 0), Rl(2, 1), Rl(2, 2));
      tf2::Quaternion q;
      m.getRotation(q);
      r.tag_pose.pose.position.x = tl[0];
      r.tag_pose.pose.position.y = tl[1];
      r.tag_pose.pose.position.z = tl[2];
      r.tag_pose.pose.orientation = tf2::toMsg(q);
    }
  }

  cv::Mat undistort(const cv::Mat & img)
  {
    if (arm_K_.size() != 9 || arm_K_[0] <= 0.0) {return img;}   // 보정값 없음 → 그대로 (태그 자세는 생략)
    if (map1_.empty() || map_size_ != img.size()) {
      cv::Mat K(3, 3, CV_64F, arm_K_.data()), D(1, static_cast<int>(arm_D_.size()), CV_64F, arm_D_.data());
      cv::Mat Ks = K.clone();
      const double sx = static_cast<double>(img.cols) / arm_w_, sy = static_cast<double>(img.rows) / arm_h_;
      Ks.at<double>(0, 0) *= sx; Ks.at<double>(0, 2) *= sx; Ks.at<double>(1, 1) *= sy; Ks.at<double>(1, 2) *= sy;
      newK_ = cv::getOptimalNewCameraMatrix(Ks, D, img.size(), 0.0);
      cv::initUndistortRectifyMap(Ks, D, cv::Mat(), newK_, img.size(), CV_16SC2, map1_, map2_);
      map_size_ = img.size();
    }
    cv::Mat out;
    cv::remap(img, out, map1_, map2_, cv::INTER_LINEAR);
    return out;
  }

  // 파라미터
  std::string front_serial_, rear_serial_, arm_dev_, base_frame_, odom_frame_, front_frame_, rear_frame_, arm_frame_;
  int rs_w_, rs_h_, rs_fps_, depth_win_, arm_w_, arm_h_, arm_fps_, detect_w_, target_class_;
  double depth_min_, depth_max_, rate_, frame_max_age_;
  std::vector<double> arm_K_, arm_D_;

  std::unique_ptr<Detector> detector_;
  std::unique_ptr<TagDetector> tag_;
  TargetTracker tracker_;
  std::atomic<uint8_t> mode_{SetMode::Request::OFF};
  std::atomic<bool> reset_tracker_{false};
  std::atomic<bool> running_{true};
  DepthSlot front_, rear_;
  ImageSlot arm_;
  cv::Mat map1_, map2_, newK_;
  double size_min_m_ = 0.015, size_max_m_ = 0.09;
  double rs_white_balance_ = 4600.0, rs_exposure_ = 0.0;
  cv::Size map_size_;
  std::vector<std::thread> threads_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  rclcpp::Publisher<Result>::SharedPtr pub_;
  rclcpp::Service<SetMode>::SharedPtr srv_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::PerceptionNode)
