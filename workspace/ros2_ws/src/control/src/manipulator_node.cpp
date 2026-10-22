// ManipulatorNode (Controller 팀) — 팔 · 그리퍼 명령 주체는 이 노드 하나.
//   /manipulator/pose          이름(STOW · READY · PLACE · TAG_VIEW) → 저장된 관절각 → 궤적
//   /manipulator/pick          목표 우선순위 ① 정밀 정렬 결과(TFmini 축 위 /arm_range 지점) ② 새 object_pose ③ 기억한 위치(odom)
//                              → arm_base_link → ArmIK → 열기 → 접근 → 미세 보정 → 마지막 유효 거리만큼 직선 전진
//                              → 닫기(stalled = 잡음 → grasped) → 들기
//   /manipulator/execute_skill 강화학습 정책 (robot_policy, CPU) → /arm_controller/joint_trajectory 1포인트 스트리밍
//   스킬 실행 FSM(PRECHECK · RUNNING · VERIFY · HOLD · DONE · FAILED · REJECTED, /arm/skill_state — 모니터링)
//   안전: HOLD(관절 · 속도 한계, 공구 높이) 는 이 노드, E-stop 은 OpenCRHardware 한 곳(잠금식 /e_stop)을 따른다.
//   result · /arm_range 는 header.stamp 300 ms 지나면 버림 (기억한 위치만 예외). 대기는 20 Hz 타이머 + 비동기 콜백.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control/arm_ik.hpp"
#include "control/fsm.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "robot_interfaces/action/arm_pose.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/action/pick.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces/msg/skill_state.hpp"
#include "robot_policy/policy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace control
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;
using FJT = control_msgs::action::FollowJointTrajectory;
using Grip = control_msgs::action::ParallelGripperCommand;
using ArmPose = robot_interfaces::action::ArmPose;
using Pick = robot_interfaces::action::Pick;
using Skill = robot_interfaces::action::ExecuteSkill;
using PoseGH = rclcpp_action::ServerGoalHandle<ArmPose>;
using PickGH = rclcpp_action::ServerGoalHandle<Pick>;
using SkillGH = rclcpp_action::ServerGoalHandle<Skill>;
using PerceptionResult = robot_interfaces::msg::PerceptionResult;
using PerceptionMode = robot_interfaces::srv::SetPerceptionMode::Request;   // 모드 비트

class ManipulatorNode : public rclcpp::Node
{
public:
  explicit ManipulatorNode(const rclcpp::NodeOptions & options)
  : Node("manipulator_node", options), ik_(ArmGeometry{}, JointLimits{})
  {
    load_params();
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cbg_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    skill_pub_ = create_publisher<robot_interfaces::msg::SkillState>(names::ARM_SKILL_STATE, qos::latched_state());
    safety_pub_ = create_publisher<robot_interfaces::msg::SafetyState>(names::ARM_SAFETY_STATE, qos::latched_state());
    stream_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(stream_topic_, 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(names::JOINT_STATES, 20,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {on_joint_states(*m);}, so);
    perc_sub_ = create_subscription<PerceptionResult>(names::PERCEPTION_RESULT, qos::result(),
        [this](PerceptionResult::ConstSharedPtr m) {on_result(m);}, so);
    range_sub_ = create_subscription<sensor_msgs::msg::Range>(names::ARM_RANGE, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr m) {range_ = m;}, so);
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(names::E_STOP, qos::e_stop(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {estop_input_ = m->data;}, so);

    traj_client_ = rclcpp_action::create_client<FJT>(this, arm_action_, cbg_);
    grip_client_ = rclcpp_action::create_client<Grip>(this, gripper_action_, cbg_);

    pose_server_ = rclcpp_action::create_server<ArmPose>(this, names::MANIP_POSE,
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmPose::Goal>) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
        [](std::shared_ptr<PoseGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<PoseGH> gh) {start_pose(gh);},
        rcl_action_server_get_default_options(), cbg_);
    pick_server_ = rclcpp_action::create_server<Pick>(this, names::MANIP_PICK,
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal>) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
        [](std::shared_ptr<PickGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<PickGH> gh) {start_pick(gh);},
        rcl_action_server_get_default_options(), cbg_);
    skill_server_ = rclcpp_action::create_server<Skill>(this, names::MANIP_SKILL,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Skill::Goal> g) {
          std::string why;
          if (!skill_enabled_) {why = "skill.enabled=false";}
          else if (!policies_.find(g->policy)) {why = "정책 '" + g->policy + "' 없음";}
          else if (policies_.find(g->policy)->spec.action_kind != "joint_target" || policies_.find(g->policy)->spec.action_dim != 6) {
            why = "팔 정책이 아님 (joint_target(6))";
          }
          if (!why.empty()) {
            RCLCPP_WARN(get_logger(), "execute_skill 거부: %s", why.c_str());
            return rclcpp_action::GoalResponse::REJECT;   // mission 이 IK 경로(fallback)로 대체
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<SkillGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<SkillGH> gh) {start_skill(gh);},
        rcl_action_server_get_default_options(), cbg_);

    latch_.fsm().on_change([this](SafetyState from, SafetyState to, const std::string & why) {
        RCLCPP_WARN(get_logger(), "안전 %s → %s : %s", to_string(from), to_string(to), why.c_str());
        if (to != SafetyState::NORMAL) {hold_motion();}
        publish_safety();
      });

    timer_ = create_wall_timer(50ms, [this]() {tick();}, cbg_);
    publish_safety();
    publish_skill();
    RCLCPP_INFO(get_logger(), "manipulator_node 준비 (그리퍼 %s, 정책 %s)", use_gripper_ ? gripper_action_.c_str() : "사용 안 함",
      skill_enabled_ ? "사용" : "사용 안 함");
  }

private:
  // =============================== 파라미터 ===============================
  void load_params()
  {
    joint_names_ = declare_parameter("joint_names",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"});
    ArmGeometry g;
    g.d1 = declare_parameter("arm.d1", g.d1);
    g.l2x = declare_parameter("arm.l2x", g.l2x);
    g.l2z = declare_parameter("arm.l2z", g.l2z);
    g.l3 = declare_parameter("arm.l3", g.l3);
    g.l_tool = declare_parameter("arm.l_tool", g.l_tool);
    JointLimits lim;
    auto lo = declare_parameter("joint_limits.lower", std::vector<double>(lim.lower.begin(), lim.lower.end()));
    auto hi = declare_parameter("joint_limits.upper", std::vector<double>(lim.upper.begin(), lim.upper.end()));
    for (int i = 0; i < 6 && i < static_cast<int>(lo.size()) && i < static_cast<int>(hi.size()); ++i) {
      lim.lower[i] = lo[i];
      lim.upper[i] = hi[i];
    }
    ik_ = ArmIK(g, lim);
    for (const std::string name : {"STOW", "READY", "PLACE", "TAG_VIEW"}) {
      auto v = declare_parameter("poses." + name, std::vector<double>(6, 0.0));
      if (v.size() == 6) {Joints q; std::copy(v.begin(), v.end(), q.begin()); poses_[name] = q;}
    }
    max_joint_vel_ = declare_parameter("motion.max_joint_velocity", 1.0);
    approach_vel_scale_ = declare_parameter("motion.approach_velocity_scale", 0.4);
    min_move_time_ = declare_parameter("motion.min_move_time", 0.6);
    verify_tol_ = declare_parameter("motion.verify_tolerance", 0.05);
    pose_timeout_ = declare_parameter("motion.pose_timeout_s", 15.0);
    pick_timeout_ = declare_parameter("motion.pick_timeout_s", 45.0);
    line_steps_ = declare_parameter("motion.final_approach_points", 4);

    arm_base_frame_ = declare_parameter("frames.arm_base", std::string("arm_base_link"));
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    odom_frame_ = declare_parameter("frames.odom", std::string("odom"));
    range_frame_ = declare_parameter("frames.range", std::string("tfmini_link"));
    tool_pitch_ = declare_parameter("pick.tool_pitch", -1.5708);
    approach_dist_ = declare_parameter("pick.approach_distance", 0.05);
    grasp_depth_ = declare_parameter("pick.grasp_depth", 0.01);
    lift_height_ = declare_parameter("pick.lift_height", 0.06);
    corr_iters_ = declare_parameter("pick.correction_iterations", 3);
    corr_tol_ = declare_parameter("pick.correction_tolerance", 0.03);
    corr_gain_ = declare_parameter("pick.correction_gain", 0.8);
    corr_settle_ = declare_parameter("pick.correction_settle_s", 0.3);
    cam_hfov_ = declare_parameter("pick.cam_hfov", 1.20);
    cam_vfov_ = declare_parameter("pick.cam_vfov", 0.75);
    img_x_sign_ = declare_parameter("pick.image_x_to_tangential", -1.0);
    img_y_sign_ = declare_parameter("pick.image_y_to_radial", -1.0);

    use_gripper_ = declare_parameter("gripper.enabled", false);
    gripper_joint_ = declare_parameter("gripper.joint", std::string("gripper_joint"));
    grip_open_ = declare_parameter("gripper.open_position", 0.02);
    grip_close_ = declare_parameter("gripper.close_position", 0.0);
    assume_grasped_ = declare_parameter("gripper.assume_grasped_without_gripper", true);

    js_timeout_ = declare_parameter("safety.joint_state_timeout_s", 0.5);
    limit_margin_ = declare_parameter("safety.limit_margin", 0.05);
    vel_violation_ = declare_parameter("safety.max_joint_velocity", 3.0);
    min_tool_z_ = declare_parameter("safety.min_tool_z", -0.115);
    stale_s_ = declare_parameter("stale_s", 0.3);

    arm_action_ = declare_parameter("arm_controller_action", std::string(names::ARM_TRAJ_ACTION));
    gripper_action_ = declare_parameter("gripper_action", std::string(names::GRIPPER_ACTION));
    stream_topic_ = declare_parameter("skill.stream_topic", std::string(names::ARM_STREAM));
    stream_horizon_ = declare_parameter("skill.stream_horizon_s", 0.1);   // 떨리면 늘림 (미결: 실물 시험)
    skill_enabled_ = declare_parameter("skill.enabled", false);
    const auto policies_file = declare_parameter("skill.policies_file", std::string(""));
    if (skill_enabled_) {
      std::vector<std::string> errs;
      policies_.load(policies_file, errs);
      for (const auto & e : errs) {RCLCPP_WARN(get_logger(), "정책 로드 실패: %s", e.c_str());}
    }
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return age <= stale_s_ && age >= -0.5;
  }
  bool range_valid() const
  {
    return range_ && fresh(range_->header.stamp) && std::isfinite(range_->range) &&
           range_->range >= range_->min_range && range_->range <= range_->max_range;
  }
  bool offset_valid() const
  {
    return result_ && fresh(result_->header.stamp) && (result_->mode & PerceptionMode::ARM_ALIGN) && result_->offset_valid;
  }

  // =============================== 입력 ===============================
  void on_joint_states(const sensor_msgs::msg::JointState & m)
  {
    for (size_t i = 0; i < m.name.size(); ++i) {
      for (int k = 0; k < 6; ++k) {
        if (m.name[i] == joint_names_[k]) {
          if (i < m.position.size()) {q_[k] = m.position[i]; got_[k] = true;}
          if (i < m.velocity.size()) {qd_[k] = m.velocity[i];}
        }
      }
    }
    if (std::all_of(got_.begin(), got_.end(), [](bool b) {return b;})) {js_rx_ = now(); have_js_ = true;}
  }
  bool joints_fresh() {return have_js_ && (now() - js_rx_).seconds() <= js_timeout_;}

  // 물체 위치 기억: SEARCH_FRONT 가 켜진 동안 유효한 object_pose 를 odom 기준으로 저장, OFF · SEARCH_FRONT 꺼짐이면 지움
  void on_result(PerceptionResult::ConstSharedPtr m)
  {
    result_ = m;
    if (m->mode == 0 || !(m->mode & PerceptionMode::SEARCH_FRONT)) {
      if (memory_) {RCLCPP_INFO(get_logger(), "물체 위치 기억 삭제 (mode %u)", m->mode);}
      memory_.reset();
      return;
    }
    if (!m->object_found || !fresh(m->header.stamp)) {return;}
    try {
      geometry_msgs::msg::PoseStamped in = m->object_pose, out;
      if (!tf_buffer_->canTransform(odom_frame_, in.header.frame_id, tf2_ros::fromRclcpp(rclcpp::Time(in.header.stamp, RCL_ROS_TIME)))) {
        in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      tf_buffer_->transform(in, out, odom_frame_, tf2::durationFromSec(0.0));
      memory_ = out;
    } catch (const tf2::TransformException &) {
      // odom 이 없으면 기억 불가 (③ 단계 생략)
    }
  }

  // =============================== 안전 ===============================
  void update_safety()
  {
    // E-stop 은 OpenCRHardware 의 잠금식 /e_stop 을 그대로 따른다 (해제도 그쪽 리셋 버튼)
    if (estop_input_) {latch_.estop(names::E_STOP); return;}
    if (have_js_ && !joints_fresh()) {latch_.estop("관절 상태 끊김"); return;}
    if (latch_.state() == SafetyState::ESTOP) {
      std::string why;
      latch_.reset_estop(false, joints_fresh(), why);
      return;
    }
    if (!have_js_) {return;}
    const std::string lv = ik_.limits().violation(q_, limit_margin_);
    if (!lv.empty()) {latch_.hold("관절 한계 위반: " + lv); return;}
    for (int k = 0; k < 6; ++k) {
      if (std::fabs(qd_[k]) > vel_violation_) {latch_.hold("속도 한계 위반: " + joint_names_[k]); return;}
    }
    if (ik_.fk_position(q_).z() < min_tool_z_) {latch_.hold("공구 끝이 너무 낮음 (바닥 · 차체 충돌 위험)");}
  }

  void publish_safety()
  {
    robot_interfaces::msg::SafetyState s;
    s.header.stamp = now();
    s.state = static_cast<uint8_t>(latch_.state());
    s.reason = latch_.reason();
    safety_pub_->publish(s);
  }

  void hold_motion()
  {
    ++traj_gen_;
    if (traj_handle_) {traj_client_->async_cancel_goal(traj_handle_); traj_handle_.reset();}
    if (!have_js_ || !traj_client_->action_server_is_ready()) {return;}
    FJT::Goal g;
    g.trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.assign(q_.begin(), q_.end());
    p.velocities.assign(6, 0.0);
    p.time_from_start = rclcpp::Duration::from_seconds(0.1);
    g.trajectory.points.push_back(p);
    traj_client_->async_send_goal(g);
  }

  // =============================== 스킬 FSM ===============================
  enum class Kind { POSE, PICK, SKILL };
  struct Job
  {
    Kind kind = Kind::POSE;
    std::shared_ptr<PoseGH> pose_gh;
    std::shared_ptr<PickGH> pick_gh;
    std::shared_ptr<SkillGH> skill_gh;
    std::string skill;
    enum class Phase { MOVE, OPEN, PREGRASP, CORRECT, GRASP, CLOSE, LIFT } phase = Phase::MOVE;
    bool waiting = false;
    bool step_ok = true;
    std::string step_msg;
    rclcpp::Time start;
    double timeout = 10.0;
    Joints final_target{};
    double move_duration = 1.0;
    rclcpp::Time move_start;
    Eigen::Vector3d grasp{0, 0, 0};
    double obj_yaw = 0.0;
    int corr_done = 0;
    rclcpp::Time settle_until;
    uint8_t stage = 0;
    bool grasped = false;
    std::string source;          // 목표 출처 (로그)
    robot_policy::LoadedPolicy * policy = nullptr;
    rclcpp::Time last_stream;
  };

  void set_skill(SkillState to, const std::string & why)
  {
    if (!skill_fsm_.transition(to, why)) {
      RCLCPP_ERROR(get_logger(), "스킬 전이 거부: %s → %s (%s)", to_string(skill_fsm_.state()), to_string(to), why.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "[%s] %s %s", job_ ? job_->skill.c_str() : "-", to_string(to), why.c_str());
    publish_skill();
  }

  void publish_skill()
  {
    robot_interfaces::msg::SkillState s;
    s.header.stamp = now();
    s.skill = job_ ? job_->skill : "";
    s.state = static_cast<uint8_t>(skill_fsm_.state());
    s.reason = skill_fsm_.reason();
    skill_pub_->publish(s);
  }

  template<typename GH, typename R>
  static void reply(const std::shared_ptr<GH> & gh, std::shared_ptr<R> r, bool ok, bool canceled)
  {
    if (!gh || !gh->is_active()) {return;}
    if (canceled) {gh->canceled(r);} else if (ok) {gh->succeed(r);} else {gh->abort(r);}
  }

  void finish(SkillState end, const std::string & msg, bool canceled = false)
  {
    if (!job_) {return;}
    if (skill_fsm_.state() != end) {set_skill(end, msg);}
    const bool ok = end == SkillState::DONE;
    if (job_->kind == Kind::PICK) {
      auto r = std::make_shared<Pick::Result>();
      r->success = ok;
      r->grasped = ok && job_->grasped;
      r->message = msg;
      reply(job_->pick_gh, r, ok, canceled);
    } else if (job_->kind == Kind::POSE) {
      auto r = std::make_shared<ArmPose::Result>();
      r->success = ok;
      r->message = msg;
      reply(job_->pose_gh, r, ok, canceled);
      if (ok) {last_pose_ = job_->skill;}
    } else {
      auto r = std::make_shared<Skill::Result>();
      r->success = ok;
      r->message = msg;
      reply(job_->skill_gh, r, ok, canceled);
    }
    ++traj_gen_;
    traj_handle_.reset();
    set_skill(SkillState::IDLE, msg);
    job_.reset();
  }

  void fail_running(const std::string & why, bool canceled = false)
  {
    if (skill_fsm_.state() == SkillState::RUNNING) {
      set_skill(SkillState::HOLD, why);
      hold_motion();
    }
    if (skill_fsm_.state() == SkillState::HOLD || skill_fsm_.state() == SkillState::VERIFY) {finish(SkillState::FAILED, why, canceled);}
  }

  void preempt(const std::string & why)
  {
    if (!job_) {return;}
    if (skill_fsm_.state() == SkillState::RUNNING) {fail_running(why);} else {finish(SkillState::FAILED, why);}
  }

  bool precheck_common(std::string & why)
  {
    if (latch_.state() == SafetyState::HOLD) {latch_.release_hold(have_js_ && ik_.limits().within(q_, limit_margin_));}
    if (latch_.state() != SafetyState::NORMAL) {why = std::string("안전 상태 ") + to_string(latch_.state()) + " (" + latch_.reason() + ")"; return false;}
    if (!joints_fresh()) {why = "관절 상태 없음"; return false;}
    if (!traj_client_->action_server_is_ready()) {why = "arm_controller 액션 서버 없음"; return false;}
    return true;
  }

  bool target_ok(const Joints & q, std::string & why)
  {
    const std::string v = ik_.limits().violation(q);
    if (!v.empty()) {why = "목표 관절 한계 초과: " + v; return false;}
    if (ik_.fk_position(q).z() < min_tool_z_) {why = "목표 공구 높이가 최소 높이보다 낮음"; return false;}
    return true;
  }

  std::unique_ptr<Job> new_job(Kind k, const std::string & name, double timeout)
  {
    auto j = std::make_unique<Job>();
    j->kind = k;
    j->skill = name;
    j->start = now();
    j->timeout = timeout;
    return j;
  }

  // ------------------------------- ArmPose -------------------------------
  void start_pose(std::shared_ptr<PoseGH> gh)
  {
    preempt("새 /manipulator/pose goal 로 대체");
    job_ = new_job(Kind::POSE, gh->get_goal()->name, pose_timeout_);
    job_->pose_gh = gh;
    set_skill(SkillState::PRECHECK, "");
    std::string why;
    auto it = poses_.find(gh->get_goal()->name);
    if (it == poses_.end()) {why = "모르는 자세 이름: " + gh->get_goal()->name;}
    if (why.empty()) {precheck_common(why);}
    if (why.empty()) {target_ok(it->second, why);}
    if (!why.empty()) {finish(SkillState::REJECTED, why); return;}
    job_->final_target = it->second;
    set_skill(SkillState::RUNNING, "");
    send_move({it->second}, 1.0);
  }

  // ------------------------------- Pick -------------------------------
  bool to_arm_base(geometry_msgs::msg::PoseStamped in, bool latest, Eigen::Vector3d & p, double & yaw, std::string & why)
  {
    try {
      if (latest) {in.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);}
      geometry_msgs::msg::PoseStamped out;
      tf_buffer_->transform(in, out, arm_base_frame_, tf2::durationFromSec(0.0));
      p = {out.pose.position.x, out.pose.position.y, out.pose.position.z};
      const auto & o = out.pose.orientation;
      if (std::fabs(o.w) < 0.9999) {
        double r, pi, y;
        tf2::Matrix3x3(tf2::Quaternion(o.x, o.y, o.z, o.w)).getRPY(r, pi, y);
        yaw = y;
      }
      return true;
    } catch (const tf2::TransformException & e) {
      why = std::string("TF 변환 실패: ") + e.what();
      return false;
    }
  }

  // TFmini 축 위 /arm_range 지점 (arm_base_link)
  bool range_point(Eigen::Vector3d & p, std::string & why)
  {
    if (!range_valid()) {why = "/arm_range 무효"; return false;}
    geometry_msgs::msg::PoseStamped s;
    s.header.frame_id = range_->header.frame_id.empty() ? range_frame_ : range_->header.frame_id;
    s.pose.position.x = range_->range;
    s.pose.orientation.w = 1.0;
    double yaw = 0;
    return to_arm_base(s, true, p, yaw, why);
  }

  bool resolve_pick_target(const Pick::Goal & goal, Eigen::Vector3d & p, double & yaw, std::string & why)
  {
    yaw = std::nan("");
    if (!goal.target.header.frame_id.empty()) {
      job_->source = "goal.target";
      return to_arm_base(goal.target, true, p, yaw, why);
    }
    // ① 정밀 정렬 결과: offset 유효 · 오차 ≤ 허용오차 · /arm_range 유효, 모두 300 ms 안
    if (offset_valid() && std::hypot(result_->arm_offset.x, result_->arm_offset.y) <= corr_tol_ && range_valid()) {
      job_->source = "정밀 정렬 (TFmini)";
      return range_point(p, why);
    }
    // ② 새 object_pose (base_link)
    if (result_ && fresh(result_->header.stamp) && result_->object_found) {
      job_->source = "object_pose (D435)";
      return to_arm_base(result_->object_pose, true, p, yaw, why);
    }
    // ③ SEARCH_FRONT 가 켜진 동안 odom 기준으로 기억한 위치 (300 ms 규칙 예외)
    if (memory_) {
      job_->source = "기억한 위치 (odom)";
      return to_arm_base(*memory_, true, p, yaw, why);
    }
    why = "목표 없음 (정밀 정렬 · object_pose · 기억한 위치 모두 없음)";
    return false;
  }

  Eigen::Vector3d tool_dir(double yaw_base) const
  {
    return {std::cos(tool_pitch_) * std::cos(yaw_base), std::cos(tool_pitch_) * std::sin(yaw_base), std::sin(tool_pitch_)};
  }

  std::optional<Joints> solve(const Eigen::Vector3d & p, double yaw, std::string & why)
  {
    IkRequest r;
    r.position = p;
    r.tool_pitch = tool_pitch_;
    r.object_yaw = std::isfinite(yaw) ? yaw : std::atan2(p.y(), p.x());
    auto q = ik_.solve(r, why);
    if (q && !target_ok(*q, why)) {return std::nullopt;}
    return q;
  }

  Eigen::Vector3d pregrasp_point() const
  {
    return job_->grasp - approach_dist_ * tool_dir(std::atan2(job_->grasp.y(), job_->grasp.x()));
  }

  void start_pick(std::shared_ptr<PickGH> gh)
  {
    preempt("새 /manipulator/pick goal 로 대체");
    job_ = new_job(Kind::PICK, "pick", pick_timeout_);
    job_->pick_gh = gh;
    job_->stage = Pick::Feedback::PLAN;
    set_skill(SkillState::PRECHECK, "");
    pick_feedback();
    std::string why;
    precheck_common(why);
    if (why.empty() && use_gripper_ && !grip_client_->action_server_is_ready()) {why = "gripper_controller 액션 서버 없음";}
    Eigen::Vector3d surf;
    double yaw = 0.0;
    if (why.empty()) {resolve_pick_target(*gh->get_goal(), surf, yaw, why);}
    if (why.empty()) {
      job_->grasp = surf - grasp_depth_ * Eigen::Vector3d::UnitZ();
      job_->obj_yaw = yaw;
      if (!solve(pregrasp_point(), yaw, why)) {why = "접근 위치 " + why;}
      else if (!solve(job_->grasp, yaw, why)) {why = "집기 위치 " + why;}
      else if (!solve(job_->grasp + lift_height_ * Eigen::Vector3d::UnitZ(), yaw, why)) {why = "들기 위치 " + why;}
    }
    if (!why.empty()) {finish(SkillState::REJECTED, why); return;}   // IK 해 없음 → abort, 재시도는 mission
    RCLCPP_INFO(get_logger(), "pick 목표 [%s] (arm_base) %.3f %.3f %.3f", job_->source.c_str(),
      job_->grasp.x(), job_->grasp.y(), job_->grasp.z());
    set_skill(SkillState::RUNNING, "");
    job_->stage = Pick::Feedback::APPROACH;
    if (use_gripper_) {job_->phase = Job::Phase::OPEN; send_gripper(grip_open_, false);} else {go_pregrasp();}
  }

  void go_pregrasp()
  {
    std::string why;
    auto q = solve(pregrasp_point(), job_->obj_yaw, why);
    if (!q) {fail_running("접근 위치 " + why); return;}
    job_->phase = Job::Phase::PREGRASP;
    send_move({*q}, 1.0);
  }

  void pick_feedback()
  {
    if (!job_ || job_->kind != Kind::PICK) {return;}
    auto fb = std::make_shared<Pick::Feedback>();
    fb->stage = job_->stage;
    job_->pick_gh->publish_feedback(fb);
  }

  // 최종 접근: TFmini 는 10 cm 미만을 못 재므로 보정 끝의 유효 거리로 집을 점을 다시 구하고 IK 직선 전진
  void final_approach()
  {
    std::string why;
    Eigen::Vector3d p;
    if (range_point(p, why)) {
      const Eigen::Vector3d g = p - grasp_depth_ * Eigen::Vector3d::UnitZ();
      if ((g - job_->grasp).norm() < 0.05) {job_->grasp = g;}   // 5 cm 넘게 다르면 잘못 잰 값으로 보고 무시
    }
    const Eigen::Vector3d a = ik_.fk_position(q_), b = job_->grasp;
    std::vector<Joints> path;
    for (int i = 1; i <= std::max(1, line_steps_); ++i) {
      auto q = solve(a + (b - a) * (static_cast<double>(i) / std::max(1, line_steps_)), job_->obj_yaw, why);
      if (!q) {fail_running("집기 위치 (직선 전진) " + why); return;}
      path.push_back(*q);
    }
    job_->stage = Pick::Feedback::GRASP;
    pick_feedback();
    job_->phase = Job::Phase::GRASP;
    send_move(path, approach_vel_scale_);
  }

  void advance_pick()
  {
    using P = Job::Phase;
    std::string why;
    switch (job_->phase) {
      case P::OPEN:
        go_pregrasp();
        break;
      case P::PREGRASP:
        job_->phase = P::CORRECT;
        job_->settle_until = now() + rclcpp::Duration::from_seconds(corr_settle_);
        break;
      case P::CORRECT: {
          if (now() < job_->settle_until) {return;}
          // 이동이 끝나고 settle 이 지난 뒤 촬영한 offset 만
          const bool fresh_off = offset_valid() && rclcpp::Time(result_->header.stamp, RCL_ROS_TIME) > job_->settle_until;
          if (fresh_off) {
            const auto & o = result_->arm_offset;
            const bool in_tol = std::hypot(o.x, o.y) <= corr_tol_;
            if (!in_tol && job_->corr_done < corr_iters_) {
              const double dist = range_valid() ? range_->range : approach_dist_;
              const double mx = o.x * dist * std::tan(cam_hfov_ / 2.0) * corr_gain_;
              const double my = o.y * dist * std::tan(cam_vfov_ / 2.0) * corr_gain_;
              const double yb = std::atan2(job_->grasp.y(), job_->grasp.x());
              const Eigen::Vector3d radial(std::cos(yb), std::sin(yb), 0), tang(-std::sin(yb), std::cos(yb), 0);
              const Eigen::Vector3d old = job_->grasp;
              job_->grasp += img_y_sign_ * my * radial + img_x_sign_ * mx * tang;
              auto q = solve(pregrasp_point(), job_->obj_yaw, why);
              if (!q || !solve(job_->grasp, job_->obj_yaw, why)) {
                RCLCPP_WARN(get_logger(), "미세 보정 %d: IK 해 없음 — 보정 없이 진행 (%s)", job_->corr_done + 1, why.c_str());
                job_->grasp = old;
                job_->corr_done = corr_iters_;
                return;
              }
              ++job_->corr_done;
              RCLCPP_INFO(get_logger(), "미세 보정 %d/%d: 오차 (%.3f, %.3f) → %.1f / %.1f mm", job_->corr_done, corr_iters_,
                o.x, o.y, 1000 * mx, 1000 * my);
              job_->phase = P::PREGRASP;
              send_move({*q}, 1.0);
              return;
            }
          } else if ((now() - job_->settle_until).seconds() < 1.0) {
            return;   // 새 offset 을 1 s 까지 기다림
          }
          final_approach();
          break;
        }
      case P::GRASP:
        if (use_gripper_) {job_->phase = P::CLOSE; send_gripper(grip_close_, true);}
        else {job_->grasped = assume_grasped_; start_lift();}
        break;
      case P::CLOSE:
        start_lift();
        break;
      case P::LIFT:
        set_skill(SkillState::VERIFY, "");
        verify();
        break;
      case P::MOVE:
        break;
    }
  }

  void start_lift()
  {
    std::string why;
    auto q = solve(job_->grasp + lift_height_ * Eigen::Vector3d::UnitZ(), job_->obj_yaw, why);
    if (!q) {fail_running("들기 위치 " + why); return;}
    job_->stage = Pick::Feedback::LIFT;
    pick_feedback();
    job_->phase = Job::Phase::LIFT;
    send_move({*q}, approach_vel_scale_);
  }

  void verify(bool timed_out = false)
  {
    if (job_->kind == Kind::PICK) {
      if (timed_out && !(job_->phase == Job::Phase::LIFT && !job_->waiting && job_->step_ok)) {
        finish(SkillState::FAILED, "시간 초과 (들기까지 끝나지 않음)");
        return;
      }
      // 동작은 끝까지 했으면 success. 빈손이면 grasped=false → mission 이 직전 search 부터 재시도
      finish(SkillState::DONE, job_->grasped ? "집기 완료" : (use_gripper_ ? "빈손 (그리퍼 끝까지 닫힘)" : "집기 동작 완료 (그리퍼 미사용)"));
      return;
    }
    double err = 0.0;
    for (int k = 0; k < 6; ++k) {err = std::max(err, std::fabs(q_[k] - job_->final_target[k]));}
    if (err <= verify_tol_) {finish(SkillState::DONE, "자세 도달");} else {finish(SkillState::FAILED, "자세 오차 " + std::to_string(err) + " rad");}
  }

  // ------------------------------- ExecuteSkill -------------------------------
  void start_skill(std::shared_ptr<SkillGH> gh)
  {
    preempt("새 execute_skill goal 로 대체");
    job_ = new_job(Kind::SKILL, "skill:" + gh->get_goal()->policy, gh->get_goal()->timeout > 0 ? gh->get_goal()->timeout : 30.0);
    job_->skill_gh = gh;
    job_->policy = policies_.find(gh->get_goal()->policy);
    set_skill(SkillState::PRECHECK, "");
    std::string why;
    precheck_common(why);
    if (!why.empty()) {finish(SkillState::REJECTED, why); return;}
    ++traj_gen_;
    if (traj_handle_) {traj_client_->async_cancel_goal(traj_handle_); traj_handle_.reset();}
    job_->last_stream = now() - rclcpp::Duration::from_seconds(1.0);
    set_skill(SkillState::RUNNING, "");
  }

  void tick_skill()
  {
    const auto & spec = job_->policy->spec;
    const double el = (now() - job_->start).seconds();
    if (el >= job_->timeout) {
      hold_motion();
      if (spec.finish_on_timeout) {set_skill(SkillState::VERIFY, ""); finish(SkillState::DONE, "정해진 시간 완료");}
      else {fail_running("시간 초과");}
      return;
    }
    if ((now() - job_->last_stream).seconds() < 1.0 / spec.rate_hz) {return;}
    job_->last_stream = now();
    robot_policy::ObsValues v;
    if (joints_fresh()) {v["joint_pos"] = std::vector<float>(q_.begin(), q_.end());} else {v["joint_pos"] = std::nullopt;}
    if (offset_valid()) {
      v["arm_offset"] = std::vector<float>{static_cast<float>(result_->arm_offset.x), static_cast<float>(result_->arm_offset.y),
        static_cast<float>(result_->arm_offset.z)};
    } else {
      v["arm_offset"] = std::nullopt;
    }
    if (range_valid()) {v["arm_range"] = std::vector<float>{range_->range};} else {v["arm_range"] = std::nullopt;}
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(spec).build(v, obs, err)) {fail_running("필수 관측 무효: " + err); return;}
    if (!job_->policy->runner->run(obs, raw)) {fail_running("추론 실패"); return;}
    const auto a = robot_policy::ActionMapper(spec).map(raw);
    trajectory_msgs::msg::JointTrajectory t;
    t.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    for (int k = 0; k < 6; ++k) {   // 관절 한계 clamp
      p.positions.push_back(std::clamp<double>(a[k], ik_.limits().lower[k], ik_.limits().upper[k]));
    }
    p.time_from_start = rclcpp::Duration::from_seconds(stream_horizon_);
    t.points.push_back(p);
    stream_pub_->publish(t);
    auto fb = std::make_shared<Skill::Feedback>();
    fb->progress = static_cast<float>(el / job_->timeout);
    job_->skill_gh->publish_feedback(fb);
  }

  // =============================== 궤적 · 그리퍼 ===============================
  // 여러 점이면 직선 경로 (점 사이 시간은 관절 변화에 비례)
  void send_move(const std::vector<Joints> & path, double speed_scale)
  {
    FJT::Goal g;
    g.trajectory.joint_names = joint_names_;
    Joints prev = q_;
    double t = 0.0;
    for (const auto & target : path) {
      double dq = 0.0;
      for (int k = 0; k < 6; ++k) {dq = std::max(dq, std::fabs(target[k] - prev[k]));}
      t += std::max(min_move_time_ / static_cast<double>(path.size()), dq / std::max(1e-3, max_joint_vel_ * speed_scale));
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(target.begin(), target.end());
      if (&target == &path.back()) {p.velocities.assign(6, 0.0);}
      p.time_from_start = rclcpp::Duration::from_seconds(t);
      g.trajectory.points.push_back(p);
      prev = target;
    }
    g.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    if (job_->kind == Kind::POSE) {job_->final_target = path.back();}
    job_->move_duration = t;
    job_->move_start = now();
    job_->waiting = true;
    job_->step_ok = true;

    const uint64_t gen = ++traj_gen_;
    auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<FJT>::SharedPtr h) {
        if (gen != traj_gen_ || !job_) {return;}
        if (!h) {job_->waiting = false; job_->step_ok = false; job_->step_msg = "arm_controller 가 궤적 거부";} else {traj_handle_ = h;}
      };
    opts.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & res) {
        if (gen != traj_gen_ || !job_) {return;}
        traj_handle_.reset();
        job_->waiting = false;
        job_->step_ok = res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result && res.result->error_code == 0;
        if (!job_->step_ok) {
          job_->step_msg = "궤적 실패 (code " + std::to_string(static_cast<int>(res.code)) +
            (res.result ? ", " + res.result->error_string : std::string()) + ")";
        }
      };
    traj_client_->async_send_goal(g, opts);
  }

  // closing: 닫기면 결과로 grasped 판정 (stalled = 물체 잡음, reached_goal = 끝까지 닫힘 = 빈손)
  void send_gripper(double position, bool closing)
  {
    job_->waiting = true;
    job_->step_ok = true;
    Grip::Goal g;
    g.command.name = {gripper_joint_};
    g.command.position = {position};
    const uint64_t gen = ++traj_gen_;
    auto opts = rclcpp_action::Client<Grip>::SendGoalOptions();
    opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<Grip>::SharedPtr h) {
        if (gen != traj_gen_ || !job_) {return;}
        if (!h) {job_->waiting = false; job_->step_ok = false; job_->step_msg = "gripper_controller 가 명령 거부";}
      };
    opts.result_callback = [this, gen, closing](const rclcpp_action::ClientGoalHandle<Grip>::WrappedResult & res) {
        if (gen != traj_gen_ || !job_) {return;}
        job_->waiting = false;
        const bool stalled = res.result && res.result->stalled;
        const bool reached = res.result && res.result->reached_goal;
        job_->step_ok = res.code == rclcpp_action::ResultCode::SUCCEEDED || stalled || reached;
        if (closing) {job_->grasped = stalled && !reached;}
        if (!job_->step_ok) {job_->step_msg = "그리퍼 명령 실패";}
      };
    grip_client_->async_send_goal(g, opts);
  }

  // =============================== 20 Hz ===============================
  bool canceling() const
  {
    switch (job_->kind) {
      case Kind::PICK: return job_->pick_gh->is_canceling();
      case Kind::POSE: return job_->pose_gh->is_canceling();
      default: return job_->skill_gh->is_canceling();
    }
  }

  void tick()
  {
    update_safety();
    if ((now() - last_safety_pub_).seconds() > 1.0) {publish_safety(); last_safety_pub_ = now();}
    if (!job_) {return;}
    if (canceling()) {fail_running("취소됨", true); if (job_) {finish(SkillState::FAILED, "취소됨", true);} return;}
    if (skill_fsm_.state() != SkillState::RUNNING) {return;}
    if (latch_.state() != SafetyState::NORMAL) {fail_running(std::string("안전 ") + to_string(latch_.state()) + ": " + latch_.reason()); return;}
    if (job_->kind == Kind::SKILL) {tick_skill(); return;}
    if ((now() - job_->start).seconds() > job_->timeout) {
      set_skill(SkillState::VERIFY, "시간 초과");
      hold_motion();
      verify(true);
      return;
    }
    if (job_->kind == Kind::POSE && job_->waiting) {
      auto fb = std::make_shared<ArmPose::Feedback>();
      fb->progress = static_cast<float>(std::clamp((now() - job_->move_start).seconds() / job_->move_duration, 0.0, 1.0));
      job_->pose_gh->publish_feedback(fb);
    }
    if (job_->waiting) {return;}
    if (!job_->step_ok) {fail_running(job_->step_msg); return;}
    if (job_->kind == Kind::POSE) {set_skill(SkillState::VERIFY, ""); verify();} else {advance_pick();}
  }

  // =============================== 멤버 ===============================
  ArmIK ik_;
  std::vector<std::string> joint_names_;
  std::map<std::string, Joints> poses_;
  double max_joint_vel_, approach_vel_scale_, min_move_time_, verify_tol_, pose_timeout_, pick_timeout_;
  int line_steps_;
  std::string arm_base_frame_, base_frame_, odom_frame_, range_frame_;
  double tool_pitch_, approach_dist_, grasp_depth_, lift_height_;
  int corr_iters_;
  double corr_tol_, corr_gain_, corr_settle_, cam_hfov_, cam_vfov_, img_x_sign_, img_y_sign_;
  bool use_gripper_, assume_grasped_, skill_enabled_;
  std::string gripper_joint_;
  double grip_open_, grip_close_;
  double js_timeout_, limit_margin_, vel_violation_, min_tool_z_, stale_s_, stream_horizon_;
  std::string arm_action_, gripper_action_, stream_topic_;
  robot_policy::PolicyLibrary policies_;

  Joints q_{}, qd_{};
  std::array<bool, 6> got_{};
  bool have_js_ = false;
  rclcpp::Time js_rx_{0, 0, RCL_ROS_TIME};
  PerceptionResult::ConstSharedPtr result_;
  sensor_msgs::msg::Range::ConstSharedPtr range_;
  std::optional<geometry_msgs::msg::PoseStamped> memory_;
  bool estop_input_ = false;
  std::string last_pose_;
  rclcpp::Time last_safety_pub_{0, 0, RCL_ROS_TIME};

  SafetyLatch latch_;
  Fsm<SkillState> skill_fsm_ = make_skill_fsm();
  std::unique_ptr<Job> job_;
  uint64_t traj_gen_ = 0;
  rclcpp_action::ClientGoalHandle<FJT>::SharedPtr traj_handle_;

  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<robot_interfaces::msg::SkillState>::SharedPtr skill_pub_;
  rclcpp::Publisher<robot_interfaces::msg::SafetyState>::SharedPtr safety_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr stream_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr perc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp_action::Client<FJT>::SharedPtr traj_client_;
  rclcpp_action::Client<Grip>::SharedPtr grip_client_;
  rclcpp_action::Server<ArmPose>::SharedPtr pose_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Skill>::SharedPtr skill_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control

RCLCPP_COMPONENTS_REGISTER_NODE(control::ManipulatorNode)
