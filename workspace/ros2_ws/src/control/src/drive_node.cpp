// DriveNode (Controller 팀) — 바퀴 명령 주체는 이 노드 하나 (/cmd_vel_drive → twist_mux priority 50)
//   /drive/navigate      목표가 현재 위치와 xy_goal_tolerance 이내면 yaw 차이만큼 nav2 spin, 아니면 navigate_to_pose
//   /drive/align         goal 수신 뒤 첫 새 result 의 ARM_ALIGN 비트로 모드를 정하고 끝까지 유지
//                        정밀: arm_offset → 회전, /arm_range(TFmini) → 전후 (20 Hz)
//                        먼 거리: object_pose(D435, base_link) → 회전 + 전진/후진, 범퍼 기준 거리, /scan 장애물 감시
//   /drive/execute_skill 강화학습 정책 (robot_policy, CPU) → 속도
//   result · /arm_range · /scan · odom 은 header.stamp 300 ms 지나면 버림. 바퀴 goal 은 한 번에 하나.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control/align_controller.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "robot_interfaces/action/align_base.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_policy/policy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace control
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;
using Nav = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;
using Align = robot_interfaces::action::AlignBase;
using Skill = robot_interfaces::action::ExecuteSkill;
using NavGH = rclcpp_action::ServerGoalHandle<Nav>;
using AlignGH = rclcpp_action::ServerGoalHandle<Align>;
using SkillGH = rclcpp_action::ServerGoalHandle<Skill>;
using PerceptionResult = robot_interfaces::msg::PerceptionResult;
using PerceptionMode = robot_interfaces::srv::SetPerceptionMode::Request;   // 모드 비트

class DriveNode : public rclcpp::Node
{
public:
  explicit DriveNode(const rclcpp::NodeOptions & options)
  : Node("drive_node", options)
  {
    AlignGains g;
    g.kp_yaw = declare_parameter("align.kp_yaw", g.kp_yaw);
    g.kp_lin = declare_parameter("align.kp_lin", g.kp_lin);
    g.max_ang = declare_parameter("align.max_angular", g.max_ang);
    g.max_lin = declare_parameter("align.max_linear", g.max_lin);
    g.lateral_tol = declare_parameter("align.lateral_tolerance", g.lateral_tol);
    g.settle_cycles = declare_parameter("align.settle_cycles", g.settle_cycles);
    fine_ = AlignController(g);
    CoarseGains cg;
    cg.kp_yaw = declare_parameter("align.coarse_kp_yaw", cg.kp_yaw);
    cg.kp_lin = declare_parameter("align.coarse_kp_lin", cg.kp_lin);
    cg.max_lin = declare_parameter("align.coarse_max_speed", cg.max_lin);
    cg.max_ang = declare_parameter("align.coarse_max_angular", cg.max_ang);
    cg.yaw_tol = declare_parameter("align.coarse_yaw_tolerance", cg.yaw_tol);
    coarse_ = CoarseAlign(cg);
    rate_hz_ = declare_parameter("align.rate_hz", 20.0);
    lost_s_ = declare_parameter("align.lost_timeout_s", 0.5);
    mode_wait_s_ = declare_parameter("align.mode_wait_s", 0.5);
    align_timeout_s_ = declare_parameter("align.timeout_s", 30.0);
    min_fine_target_ = declare_parameter("align.min_target_distance", 0.10);
    coarse_min_ = declare_parameter("align.coarse_min", 0.3);
    default_tol_ = declare_parameter("align.default_tolerance", 0.01);
    front_offset_ = declare_parameter("base.front_offset", 0.20);   // base_link → 범퍼 앞면 ★
    scan_half_ = declare_parameter("scan.half_angle", 0.349);        // ±20°
    scan_front_margin_ = declare_parameter("scan.front_margin", 0.10);
    scan_rear_stop_ = declare_parameter("scan.rear_stop", 0.30);
    require_scan_ = declare_parameter("scan.required", true);
    stale_s_ = declare_parameter("stale_s", 0.3);
    xy_tol_ = declare_parameter("navigate.xy_goal_tolerance", 0.25);
    spin_time_ = declare_parameter("navigate.spin_time_allowance", 15.0);
    map_frame_ = declare_parameter("frames.map", std::string("map"));
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    nav_action_name_ = declare_parameter("nav2_action", std::string("navigate_to_pose"));
    spin_action_name_ = declare_parameter("spin_action", std::string("spin"));
    skill_enabled_ = declare_parameter("skill.enabled", false);
    const auto policies_file = declare_parameter("skill.policies_file", std::string(""));
    max_v_ = declare_parameter("skill.max_linear", 0.3);
    max_w_ = declare_parameter("skill.max_angular", 1.5);
    scan_bins_ = declare_parameter("skill.scan_bins", 360);
    skill_front_margin_ = declare_parameter("skill.front_margin", 0.10);   // 범퍼 앞면에서 이 거리 안에 점 → 전진 0

    if (skill_enabled_) {
      std::vector<std::string> errs;
      policies_.load(policies_file, errs);
      for (const auto & e : errs) {RCLCPP_WARN(get_logger(), "정책 로드 실패: %s", e.c_str());}
      RCLCPP_INFO(get_logger(), "정책 %zu개 로드", policies_.names().size());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cbg_;
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(names::CMD_VEL_DRIVE, 10);
    perc_sub_ = create_subscription<PerceptionResult>(names::PERCEPTION_RESULT, qos::result(),
        [this](PerceptionResult::ConstSharedPtr m) {result_ = m; result_rx_ = now();}, so);
    range_sub_ = create_subscription<sensor_msgs::msg::Range>(names::ARM_RANGE, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr m) {range_ = m;}, so);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(names::SCAN, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {scan_ = m;}, so);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(names::ODOM_FILTERED, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = m;}, so);

    nav_client_ = rclcpp_action::create_client<Nav>(this, nav_action_name_, cbg_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_, cbg_);

    nav_server_ = rclcpp_action::create_server<Nav>(this, names::DRIVE_NAVIGATE,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<NavGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<NavGH> gh) {start_navigate(gh);},
        rcl_action_server_get_default_options(), cbg_);
    align_server_ = rclcpp_action::create_server<Align>(this, names::DRIVE_ALIGN,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Align::Goal> g) {
          if (g->target_distance < min_fine_target_) {
            RCLCPP_WARN(get_logger(), "target_distance %.3f < %.2f m — 거부", g->target_distance, min_fine_target_);
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<AlignGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<AlignGH> gh) {start_align(gh);},
        rcl_action_server_get_default_options(), cbg_);
    skill_server_ = rclcpp_action::create_server<Skill>(this, names::DRIVE_SKILL,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Skill::Goal> g) {
          std::string why;
          if (!skill_enabled_) {why = "skill.enabled=false";}
          else if (!policies_.find(g->policy)) {why = "정책 '" + g->policy + "' 없음";}
          else if (policies_.find(g->policy)->spec.action_kind != "base_twist") {why = "주행 정책이 아님";}
          if (!why.empty()) {
            RCLCPP_WARN(get_logger(), "execute_skill 거부: %s", why.c_str());
            return rclcpp_action::GoalResponse::REJECT;   // mission 이 fallback 실행
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<SkillGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<SkillGH> gh) {start_skill(gh);},
        rcl_action_server_get_default_options(), cbg_);

    // 노드 시계 기준 (use_sim_time 이면 /clock) — wall timer 는 시뮬레이션 실시간 비율이 1 이 아니면 주기 · timeout 이 어긋남
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz_),
        [this]() {tick();}, cbg_);
    RCLCPP_INFO(get_logger(), "drive_node 준비 (align %.0f Hz, 정책 %s)", rate_hz_,
      skill_enabled_ ? "사용" : "사용 안 함");
  }

private:
  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return age <= stale_s_ && age >= -0.5;
  }

  void publish_cmd(double v, double w)
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = base_frame_;
    t.twist.linear.x = v;
    t.twist.angular.z = w;
    cmd_pub_->publish(t);
  }

  // ---------- 바퀴 goal 하나만 ----------
  template<typename GH, typename R>
  void end(std::shared_ptr<GH> & gh, bool ok, std::shared_ptr<R> r, bool canceled = false)
  {
    if (gh && gh->is_active()) {
      if (canceled) {gh->canceled(r);} else if (ok) {gh->succeed(r);} else {gh->abort(r);}
    }
    gh.reset();
  }

  void preempt_all(const std::string & why)
  {
    if (align_gh_) {
      publish_cmd(0, 0);
      auto r = std::make_shared<Align::Result>();
      r->message = why;
      end(align_gh_, false, r);
    }
    if (skill_gh_) {
      publish_cmd(0, 0);
      auto r = std::make_shared<Skill::Result>();
      r->message = why;
      end(skill_gh_, false, r);
      stop_skill_timer();
    }
    if (nav_gh_) {
      cancel_nav2();
      end(nav_gh_, false, std::make_shared<Nav::Result>());
    }
  }

  // ---------- /drive/navigate ----------
  std::optional<geometry_msgs::msg::PoseStamped> current_pose(const std::string & frame)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped p;
      p.header = t.header;
      p.pose.position.x = t.transform.translation.x;
      p.pose.position.y = t.transform.translation.y;
      p.pose.orientation = t.transform.rotation;
      return p;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void start_navigate(std::shared_ptr<NavGH> gh)
  {
    preempt_all("새 /drive/navigate goal 로 대체");
    nav_gh_ = gh;
    const uint64_t gen = ++nav_gen_;
    const auto & goal = *gh->get_goal();
    const std::string frame = goal.pose.header.frame_id.empty() ? map_frame_ : goal.pose.header.frame_id;
    auto cur = current_pose(frame);
    if (cur) {
      const double dx = goal.pose.pose.position.x - cur->pose.position.x;
      const double dy = goal.pose.pose.position.y - cur->pose.position.y;
      if (std::hypot(dx, dy) <= xy_tol_) {
        // 시작 = 목표 위치인 navigate_to_pose 는 경로가 안 나오거나 회전 없이 성공 → spin 으로 처리
        double dyaw = tf2::getYaw(goal.pose.pose.orientation) - tf2::getYaw(cur->pose.orientation);
        dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
        Spin::Goal s;
        s.target_yaw = static_cast<float>(dyaw);
        s.time_allowance = rclcpp::Duration::from_seconds(spin_time_);
        RCLCPP_INFO(get_logger(), "navigate → spin %.1f°", dyaw * 180 / M_PI);
        if (!spin_client_->action_server_is_ready()) {finish_nav(false, "nav2 spin 서버 없음"); return;}
        auto opts = rclcpp_action::Client<Spin>::SendGoalOptions();
        opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<Spin>::SharedPtr h) {
            if (gen != nav_gen_) {if (h) {spin_client_->async_cancel_goal(h);} return;}
            if (!h) {finish_nav(false, "spin 거부"); return;}
            spin_handle_ = h;
          };
        opts.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult & res) {
            if (gen != nav_gen_) {return;}
            spin_handle_.reset();
            finish_nav(res.code == rclcpp_action::ResultCode::SUCCEEDED, "spin");
          };
        spin_client_->async_send_goal(s, opts);
        return;
      }
    }
    if (!nav_client_->action_server_is_ready()) {finish_nav(false, "nav2 navigate_to_pose 서버 없음"); return;}
    auto opts = rclcpp_action::Client<Nav>::SendGoalOptions();
    opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr h) {
        if (gen != nav_gen_) {if (h) {nav_client_->async_cancel_goal(h);} return;}
        if (!h) {finish_nav(false, "nav2 가 goal 거부"); return;}
        nav2_handle_ = h;
      };
    opts.feedback_callback = [this, gen](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr,
      const std::shared_ptr<const Nav::Feedback> fb) {
        if (gen == nav_gen_ && nav_gh_ && nav_gh_->is_active()) {nav_gh_->publish_feedback(std::make_shared<Nav::Feedback>(*fb));}
      };
    opts.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & res) {
        if (gen != nav_gen_) {return;}
        nav2_handle_.reset();
        finish_nav(res.code == rclcpp_action::ResultCode::SUCCEEDED, "navigate_to_pose");
      };
    nav_client_->async_send_goal(goal, opts);
  }

  void cancel_nav2()
  {
    ++nav_gen_;
    if (nav2_handle_) {nav_client_->async_cancel_goal(nav2_handle_); nav2_handle_.reset();}
    if (spin_handle_) {spin_client_->async_cancel_goal(spin_handle_); spin_handle_.reset();}
  }

  void finish_nav(bool ok, const std::string & what)
  {
    if (!ok) {RCLCPP_WARN(get_logger(), "navigate 실패 (%s)", what.c_str());}
    end(nav_gh_, ok, std::make_shared<Nav::Result>());
  }

  // ---------- /drive/align ----------
  enum class AlignMode { WAIT, COARSE, FINE };

  void start_align(std::shared_ptr<AlignGH> gh)
  {
    preempt_all("새 /drive/align goal 로 대체");
    align_gh_ = gh;
    fine_.reset();
    coarse_.reset();
    align_mode_ = AlignMode::WAIT;
    align_start_ = now();
    last_valid_ = align_start_;
    align_target_ = gh->get_goal()->target_distance;
    align_tol_ = gh->get_goal()->tolerance > 0.0 ? gh->get_goal()->tolerance : default_tol_;
  }

  void finish_align(bool ok, const std::string & msg)
  {
    publish_cmd(0, 0);
    auto r = std::make_shared<Align::Result>();
    r->success = ok;
    r->message = msg;
    RCLCPP_INFO(get_logger(), "align(%s) %s: %s", align_mode_ == AlignMode::FINE ? "정밀" : "먼 거리",
      ok ? "성공" : "실패", msg.c_str());
    end(align_gh_, ok, r);
  }

  void tick_align()
  {
    if (align_gh_->is_canceling()) {
      publish_cmd(0, 0);
      auto r = std::make_shared<Align::Result>();
      r->message = "취소됨";
      end(align_gh_, false, r, true);
      return;
    }
    const auto t = now();
    if ((t - align_start_).seconds() > align_timeout_s_) {finish_align(false, "시간 초과"); return;}

    if (align_mode_ == AlignMode::WAIT) {
      // goal 수신 뒤 stamp 인 첫 result 의 모드로 결정 — 이후 바꾸지 않음
      if (result_ && rclcpp::Time(result_->header.stamp, RCL_ROS_TIME) >= align_start_ && result_->mode != 0) {
        align_mode_ = (result_->mode & PerceptionMode::ARM_ALIGN) ? AlignMode::FINE : AlignMode::COARSE;
        if (align_mode_ == AlignMode::COARSE && align_target_ < coarse_min_) {
          finish_align(false, "먼 거리 모드인데 target_distance < align.coarse_min (팔을 편 채 접근 방지)");
          return;
        }
        last_valid_ = t;
        RCLCPP_INFO(get_logger(), "align 모드: %s, 목표 %.3f m", align_mode_ == AlignMode::FINE ? "정밀(TFmini)" : "먼 거리(범퍼)",
          align_target_);
      } else if ((t - align_start_).seconds() > mode_wait_s_) {
        finish_align(false, "0.5 s 안에 새 perception result 없음");
      }
      return;
    }

    auto fb = std::make_shared<Align::Feedback>();
    AlignCmd c;
    if (align_mode_ == AlignMode::FINE) {
      const bool off_ok = result_ && fresh(result_->header.stamp) && (result_->mode & PerceptionMode::ARM_ALIGN) && result_->offset_valid;
      const bool rng_ok = range_ && fresh(range_->header.stamp) && std::isfinite(range_->range) &&
        range_->range >= range_->min_range && range_->range <= range_->max_range;
      if (!off_ok || !rng_ok) {
        publish_cmd(0, 0);   // 정밀 중 값이 끊겨도 먼 거리로 바꾸지 않음 (팔을 편 채 범퍼 기준 접근 = 충돌)
        if ((t - last_valid_).seconds() > lost_s_) {finish_align(false, !off_ok ? "물체 놓침 (arm_offset)" : "거리 무효 (/arm_range)");}
        return;
      }
      last_valid_ = t;
      c = fine_.step(result_->arm_offset.x, range_->range, align_target_, align_tol_);
      fb->lateral_error = static_cast<float>(result_->arm_offset.x);
      fb->distance = range_->range;
    } else {
      const bool obj_ok = result_ && fresh(result_->header.stamp) && result_->object_found;
      if (!obj_ok) {
        publish_cmd(0, 0);
        if ((t - last_valid_).seconds() > lost_s_) {finish_align(false, "물체 놓침 (object_pose)");}
        return;
      }
      last_valid_ = t;
      const auto & p = result_->object_pose.pose.position;
      c = coarse_.step(p.x, p.y, front_offset_, align_target_, std::max(align_tol_, 0.02));
      fb->lateral_error = static_cast<float>(std::atan2(p.y, p.x));
      fb->distance = static_cast<float>(p.x - front_offset_);
      // 직접 P 제어라 nav2 회피가 없음 → /scan 확인
      if (c.v != 0.0) {
        const bool scan_ok = scan_ && fresh(scan_->header.stamp);
        if (!scan_ok && require_scan_) {finish_align(false, "/scan 없음 — 먼 거리 접근 중지"); return;}
        if (scan_ok) {
          const bool fwd = c.v > 0;
          const double limit = fwd ? std::hypot(p.x, p.y) - scan_front_margin_ : scan_rear_stop_;
          if (scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, fwd ? 0.0 : M_PI, scan_half_, limit)) {
            finish_align(false, fwd ? "전방 장애물 (물체보다 가까운 점)" : "후방 장애물 0.3 m");
            return;
          }
        }
      }
    }
    align_gh_->publish_feedback(fb);
    if (c.done) {finish_align(true, "허용오차 안에서 정지"); return;}
    publish_cmd(c.v, c.w);
  }

  // ---------- /drive/execute_skill ----------
  void start_skill(std::shared_ptr<SkillGH> gh)
  {
    preempt_all("새 execute_skill goal 로 대체");
    skill_gh_ = gh;
    skill_ = policies_.find(gh->get_goal()->policy);
    skill_start_ = now();
    skill_timeout_ = gh->get_goal()->timeout > 0 ? gh->get_goal()->timeout : 30.0;
    skill_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / skill_->spec.rate_hz),
        [this]() {tick_skill();}, cbg_);
    RCLCPP_INFO(get_logger(), "skill %s 시작 (%.0f Hz, %.1f s)", skill_->spec.name.c_str(), skill_->spec.rate_hz, skill_timeout_);
  }

  // 타이머 콜백(tick_skill) 안에서 끝날 수 있으므로 파괴하지 않고 멈추기만 한다 (다음 start_skill 에서 교체)
  void stop_skill_timer()
  {
    if (skill_timer_) {skill_timer_->cancel();}
  }

  void finish_skill(bool ok, const std::string & msg, bool canceled = false)
  {
    publish_cmd(0, 0);
    stop_skill_timer();
    auto r = std::make_shared<Skill::Result>();
    r->success = ok;
    r->message = msg;
    RCLCPP_INFO(get_logger(), "skill %s: %s", ok ? "성공" : "실패", msg.c_str());
    end(skill_gh_, ok, r, canceled);
  }

  std::optional<std::vector<float>> scan_bins()
  {
    if (!scan_ || !fresh(scan_->header.stamp)) {return std::nullopt;}
    std::vector<float> b(scan_bins_, scan_->range_max);
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const float r = scan_->ranges[i];
      if (!std::isfinite(r) || r < scan_->range_min) {continue;}
      double a = scan_->angle_min + scan_->angle_increment * i;
      a = std::atan2(std::sin(a), std::cos(a));   // -π ~ π
      const int k = std::clamp(static_cast<int>((a + M_PI) / (2 * M_PI) * scan_bins_), 0, scan_bins_ - 1);
      b[k] = std::min(b[k], std::min(r, scan_->range_max));
    }
    return b;
  }

  void tick_skill()
  {
    if (!skill_gh_) {stop_skill_timer(); return;}
    if (skill_gh_->is_canceling()) {finish_skill(false, "취소됨", true); return;}
    const double el = (now() - skill_start_).seconds();
    if (el >= skill_timeout_) {
      if (skill_->spec.finish_on_timeout) {finish_skill(true, "정해진 시간 완료");} else {finish_skill(false, "시간 초과");}
      return;
    }
    robot_policy::ObsValues v;
    v["scan"] = scan_bins();
    if (odom_ && fresh(odom_->header.stamp)) {
      v["odom_twist"] = std::vector<float>{static_cast<float>(odom_->twist.twist.linear.x),
        static_cast<float>(odom_->twist.twist.linear.y), static_cast<float>(odom_->twist.twist.angular.z)};
    } else {
      v["odom_twist"] = std::nullopt;
    }
    if (result_ && fresh(result_->header.stamp) && result_->object_found) {
      const auto & p = result_->object_pose.pose.position;
      v["object_pose_base"] = std::vector<float>{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
    } else {
      v["object_pose_base"] = std::nullopt;
    }
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(skill_->spec).build(v, obs, err)) {finish_skill(false, "필수 관측 무효: " + err); return;}
    if (!skill_->runner->run(obs, raw)) {finish_skill(false, "추론 실패"); return;}
    const auto a = robot_policy::ActionMapper(skill_->spec).map(raw);
    double cmd_v = std::clamp<double>(a[0], -max_v_, max_v_);
    const double cmd_w = std::clamp<double>(a[1], -max_w_, max_w_);
    // 정책 출력은 nav2 · collision_monitor 를 거치지 않음 → 진행 방향 /scan 확인 (align 과 같은 규칙)
    //   막힌 방향의 전후 속도만 0, 회전은 그대로 (정책이 돌아서 빠져나갈 수 있게)
    if (std::fabs(cmd_v) > 1e-3) {
      const bool scan_ok = scan_ && fresh(scan_->header.stamp);
      if (!scan_ok && require_scan_) {finish_skill(false, "/scan 없음 — 정책 주행 중지"); return;}
      if (scan_ok) {
        const bool fwd = cmd_v > 0;
        const double limit = fwd ? front_offset_ + skill_front_margin_ : scan_rear_stop_;
        if (scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, fwd ? 0.0 : M_PI, scan_half_, limit)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "skill: %s 장애물 — 전후 속도 0", fwd ? "전방" : "후방");
          cmd_v = 0.0;
        }
      }
    }
    publish_cmd(cmd_v, cmd_w);
    auto fb = std::make_shared<Skill::Feedback>();
    fb->progress = static_cast<float>(el / skill_timeout_);
    skill_gh_->publish_feedback(fb);
  }

  void tick()
  {
    if (nav_gh_ && nav_gh_->is_canceling()) {
      cancel_nav2();
      end(nav_gh_, false, std::make_shared<Nav::Result>(), true);
    }
    if (align_gh_) {tick_align();}
  }

  // 파라미터
  double rate_hz_, lost_s_, mode_wait_s_, align_timeout_s_, min_fine_target_, coarse_min_, default_tol_, front_offset_;
  double scan_half_, scan_front_margin_, scan_rear_stop_, stale_s_, xy_tol_, spin_time_, max_v_, max_w_, skill_front_margin_;
  bool require_scan_, skill_enabled_;
  int scan_bins_;
  std::string map_frame_, base_frame_, nav_action_name_, spin_action_name_;
  AlignController fine_;
  CoarseAlign coarse_;
  robot_policy::PolicyLibrary policies_;

  // 입력 (최신값, 300 ms 규칙은 쓸 때 확인)
  PerceptionResult::ConstSharedPtr result_;
  rclcpp::Time result_rx_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Range::ConstSharedPtr range_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

  // goal
  std::shared_ptr<NavGH> nav_gh_;
  std::shared_ptr<AlignGH> align_gh_;
  std::shared_ptr<SkillGH> skill_gh_;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr nav2_handle_;
  rclcpp_action::ClientGoalHandle<Spin>::SharedPtr spin_handle_;
  uint64_t nav_gen_ = 0;
  AlignMode align_mode_ = AlignMode::WAIT;
  rclcpp::Time align_start_{0, 0, RCL_ROS_TIME}, last_valid_{0, 0, RCL_ROS_TIME}, skill_start_{0, 0, RCL_ROS_TIME};
  double align_target_ = 0.15, align_tol_ = 0.01, skill_timeout_ = 30.0;
  robot_policy::LoadedPolicy * skill_ = nullptr;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr perc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<Nav>::SharedPtr nav_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<Nav>::SharedPtr nav_server_;
  rclcpp_action::Server<Align>::SharedPtr align_server_;
  rclcpp_action::Server<Skill>::SharedPtr skill_server_;
  rclcpp::TimerBase::SharedPtr timer_, skill_timer_;
};

}  // namespace control

RCLCPP_COMPONENTS_REGISTER_NODE(control::DriveNode)
