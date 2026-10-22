// MissionNode (Controller 팀 · center 패키지 = 판단) — 순서만 결정, 직접 움직이지 않음.
//   공통 시작: pose(TAG_VIEW) → READ_TAG, 2 s 안에 없으면 제자리 30° 회전 (최대 11번) → tag_id → 미션표(missions.tag_<id>)
//   단계: navigate · search · approach · pick · pose · skill (mission_table.hpp 문법, 노드 시작 시 전부 검증)
//   /mission/start: IDLE · DONE · FAILED 면 새로, PAUSED 면 현재 단계부터 재개 (E-stop · 진단 오류가 남아 있으면 거절)
//   /mission/stop : goal 전부 cancel → IDLE, set_mode(OFF)
//   /e_stop=true · /diagnostics(opencr/*) ERROR → goal 전부 cancel → PAUSED (자동 재개 없음)
//   /mission/status 2 Hz + 변경 시. result 는 header.stamp 300 ms 지나면 버림.
//   단계 안 진행은 phase 번호로: phase 를 바꾼 뒤(이전 응답 무시) 요청을 보낸다.
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "center/mission_table.hpp"
#include "control/fsm.hpp"
#include "robot_interfaces/action/align_base.hpp"
#include "robot_interfaces/action/arm_pose.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/action/pick.hpp"
#include "robot_interfaces/msg/mission_status.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace center
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace ::control;   // Fsm · MissionState (control/fsm.hpp)
using namespace std::chrono_literals;
using Nav = nav2_msgs::action::NavigateToPose;
using Align = robot_interfaces::action::AlignBase;
using ArmPose = robot_interfaces::action::ArmPose;
using Pick = robot_interfaces::action::Pick;
using Skill = robot_interfaces::action::ExecuteSkill;
using SetMode = robot_interfaces::srv::SetPerceptionMode;
using PerceptionResult = robot_interfaces::msg::PerceptionResult;
using MissionStatus = robot_interfaces::msg::MissionStatus;
using S = MissionState;

class MissionNode : public rclcpp::Node
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options)
  : Node("mission_node", options)
  {
    load_table();
    tag_wait_ = declare_parameter("tag.wait_s", 2.0);
    tag_step_ = declare_parameter("tag.rotate_step_deg", 30.0) * M_PI / 180;
    tag_max_rot_ = declare_parameter("tag.max_rotations", 11);
    search_wait_ = declare_parameter("search.wait_s", 2.0);
    search_step_ = declare_parameter("search.rotate_step_deg", 45.0) * M_PI / 180;
    search_max_rot_ = declare_parameter("search.max_rotations", 4);
    face_tol_ = declare_parameter("search.face_tolerance_deg", 10.0) * M_PI / 180;
    confirm_s_ = declare_parameter("search.confirm_s", 2.0);
    ready_reach_ = declare_parameter("approach.ready_reach", 0.15);   // READY 팔 끝이 범퍼보다 앞으로 나온 거리 ★ 실측
    approach_margin_ = declare_parameter("approach.margin", 0.10);
    coarse_min_ = declare_parameter("approach.coarse_min", 0.30);
    fine_tol_ = declare_parameter("approach.tolerance", 0.01);
    coarse_tol_ = declare_parameter("approach.coarse_tolerance", 0.02);
    max_retries_ = declare_parameter("max_retries", 2);
    retry_delay_ = declare_parameter("retry_delay_s", 1.0);
    stale_s_ = declare_parameter("stale_s", 0.3);
    diag_prefix_ = declare_parameter("diagnostics.name_prefix", std::string("opencr/"));
    diag_timeout_ = declare_parameter("diagnostics.timeout", 0.0);   // 발행 확인 전 0 (끔), 확인 뒤 3.0
    map_frame_ = declare_parameter("frames.map", std::string("map"));
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    timeouts_[StepType::NAVIGATE] = declare_parameter("timeout.navigate_s", 120.0);
    timeouts_[StepType::SEARCH] = declare_parameter("timeout.search_s", 60.0);
    timeouts_[StepType::APPROACH] = declare_parameter("timeout.approach_s", 90.0);
    timeouts_[StepType::PICK] = declare_parameter("timeout.pick_s", 60.0);
    timeouts_[StepType::POSE] = declare_parameter("timeout.pose_s", 20.0);
    timeouts_[StepType::SKILL] = 0.0;   // goal timeout + 10 s
    tag_timeout_ = declare_parameter("timeout.tag_s", 90.0);

    fsm_.on_change([this](S from, S to, const std::string & why) {
        RCLCPP_INFO(get_logger(), "미션 %s → %s %s", to_string(from), to_string(to), why.c_str());
      });
    fsm_.on_reject([this](S from, S to, const std::string & why) {
        RCLCPP_ERROR(get_logger(), "허용되지 않은 전이 거부: %s → %s (%s)", to_string(from), to_string(to), why.c_str());
      });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cbg_;
    status_pub_ = create_publisher<MissionStatus>(names::MISSION_STATUS, 10);
    perc_sub_ = create_subscription<PerceptionResult>(names::PERCEPTION_RESULT, qos::result(),
        [this](PerceptionResult::ConstSharedPtr m) {result_ = m;}, so);
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(names::E_STOP, qos::e_stop(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {estop_ = m->data;}, so);
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(names::DIAGNOSTICS, 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {on_diag(*m);}, so);

    mode_client_ = create_client<SetMode>(names::PERCEPTION_SET_MODE, rclcpp::ServicesQoS(), cbg_);
    nav_client_ = rclcpp_action::create_client<Nav>(this, names::DRIVE_NAVIGATE, cbg_);
    align_client_ = rclcpp_action::create_client<Align>(this, names::DRIVE_ALIGN, cbg_);
    pose_client_ = rclcpp_action::create_client<ArmPose>(this, names::MANIP_POSE, cbg_);
    pick_client_ = rclcpp_action::create_client<Pick>(this, names::MANIP_PICK, cbg_);
    skill_clients_["drive"] = rclcpp_action::create_client<Skill>(this, names::DRIVE_SKILL, cbg_);
    skill_clients_["manipulator"] = rclcpp_action::create_client<Skill>(this, names::MANIP_SKILL, cbg_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(names::MISSION_START,
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {on_start(*r);},
        rclcpp::ServicesQoS(), cbg_);
    stop_srv_ = create_service<std_srvs::srv::Trigger>(names::MISSION_STOP,
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
          if (fsm_.state() == S::IDLE) {r->success = false; r->message = "이미 IDLE"; return;}
          cancel_all();
          retry_pending_ = false;
          act_.reset();
          send_mode(mode_bits::OFF);
          go(S::IDLE, "운용자 정지");
          r->success = true;
          r->message = "정지";
        }, rclcpp::ServicesQoS(), cbg_);

    timer_ = create_wall_timer(100ms, [this]() {tick();}, cbg_);
    status_timer_ = create_wall_timer(500ms, [this]() {publish_status();}, cbg_);
    RCLCPP_INFO(get_logger(), "mission_node 준비 — 미션표 tag %zu개, 장소 %zu개", missions_.size(), ctx_.locations.size());
  }

private:
  // ---------------- 미션표 (시작 시 전부 검증 → 오타는 실행 전에 실패) ----------------
  void load_table()
  {
    const auto locs = declare_parameter("locations.names", std::vector<std::string>{"home"});
    for (const auto & n : locs) {
      locations_[n] = declare_parameter("locations." + n, std::vector<double>{0.0, 0.0, 0.0});
      ctx_.locations.insert(n);
    }
    const auto ids = declare_parameter("missions.tag_ids", std::vector<int64_t>{});
    for (int64_t id : ids) {
      const auto texts = declare_parameter("missions.tag_" + std::to_string(id), std::vector<std::string>{});
      std::vector<Step> steps;
      std::string err;
      if (!parse_steps(texts, ctx_, steps, err)) {
        RCLCPP_FATAL(get_logger(), "미션표 tag_%ld 오류: %s", static_cast<long>(id), err.c_str());
        throw std::runtime_error("mission table: " + err);
      }
      missions_[static_cast<int>(id)] = steps;
    }
  }

  // ---------------- 진단 · E-stop ----------------
  void on_diag(const diagnostic_msgs::msg::DiagnosticArray & m)
  {
    bool mine = false;
    uint8_t worst = 0;
    std::string what;
    for (const auto & s : m.status) {
      if (s.name.rfind(diag_prefix_, 0) != 0) {continue;}   // opencr/ 항목만 (다른 패키지 진단 무시)
      mine = true;
      if (s.level > worst && s.level != diagnostic_msgs::msg::DiagnosticStatus::STALE) {worst = s.level; what = s.name + ": " + s.message;}
    }
    if (!mine) {return;}
    diag_rx_ = now();
    have_diag_ = true;
    diag_error_ = worst >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_what_ = what;
  }

  std::string blocking_reason()
  {
    if (estop_) {return "E-stop";}
    if (diag_error_) {return "진단 오류 (" + diag_what_ + ")";}
    if (diag_timeout_ > 0 && (!have_diag_ || (now() - diag_rx_).seconds() > diag_timeout_)) {return "opencr 진단 무수신";}
    return "";
  }

  // ---------------- 미션 상태 ----------------
  bool go(S to, const std::string & why)
  {
    if (!fsm_.transition(to, why)) {return false;}
    reason_ = why;
    publish_status();
    return true;
  }

  void publish_status()
  {
    MissionStatus m;
    m.stamp = now();
    m.tag_id = tag_id_;
    m.step_index = static_cast<uint16_t>(in_tag_phase_ ? 0 : step_i_ + 1);
    m.step_name = in_tag_phase_ ? "read_tag" : (act_ ? act_->step.text : (step_i_ < plan_.size() ? plan_[step_i_].text : ""));
    m.state = static_cast<uint8_t>(fsm_.state());
    m.reason = reason_;
    status_pub_->publish(m);
  }

  void on_start(std_srvs::srv::Trigger::Response & r)
  {
    const std::string block = blocking_reason();
    if (!block.empty()) {r.success = false; r.message = "시작 불가: " + block; return;}
    if (fsm_.state() == S::RUNNING) {r.success = false; r.message = "이미 실행 중"; return;}
    if (fsm_.state() == S::PAUSED && act_) {
      go(S::RUNNING, "재개 — 현재 단계부터");
      restart_activity();
      r.success = true;
      r.message = "재개: " + act_->step.text;
      return;
    }
    tag_id_ = -1;
    plan_.clear();
    step_i_ = 0;
    retries_ = 0;
    retry_origin_ = 0;
    in_tag_phase_ = true;
    last_pose_.clear();
    go(S::RUNNING, "미션 시작");
    begin_tag();
    r.success = true;
    r.message = "미션 시작";
  }

  // ---------------- 단계 진행 ----------------
  struct Op
  {
    bool used = false, done = false, ok = false;
    std::string msg;
    void start() {used = true; done = ok = false; msg.clear();}
    void complete(bool s, const std::string & m) {done = true; ok = s; msg = m;}
    bool success() const {return used && done && ok;}
    bool failed() const {return used && done && !ok;}
    bool idle_or_success() const {return !used || success();}
  };
  struct Activity
  {
    Step step;
    bool is_tag = false;
    int phase = 0;
    rclcpp::Time phase_start;
    rclcpp::Time start;
    int rotations = 0;
    bool fallback_used = false;
  };

  std::vector<Op *> ops() {return {&nav_op_, &align_op_, &pose_op_, &pick_op_, &skill_op_, &mode_op_};}

  // phase 를 바꾸고 이전 요청의 응답을 무시하도록 세대 증가 → 그 다음에 새 요청을 보낸다
  void phase(int p)
  {
    act_->phase = p;
    act_->phase_start = now();
    for (Op * o : ops()) {*o = Op{};}
    ++gen_;
  }

  void begin_tag()
  {
    act_ = std::make_unique<Activity>();
    act_->is_tag = true;
    act_->step.text = "read_tag";
    act_->start = now();
    phase(0);
    publish_status();
  }

  void begin_step(size_t i)
  {
    step_i_ = i;
    act_ = std::make_unique<Activity>();
    act_->step = plan_[i];
    act_->start = now();
    phase(0);
    publish_status();
  }

  void restart_activity()
  {
    act_->rotations = 0;
    act_->start = now();
    phase(0);
  }

  void step_done()
  {
    // 재시도로 앞 단계(직전 search)로 돌아간 경우, 실패했던 단계를 넘어선 뒤에만 횟수 초기화 (무한 반복 방지)
    if (step_i_ >= retry_origin_) {retries_ = 0; retry_origin_ = 0;}
    if (step_i_ + 1 >= plan_.size()) {
      send_mode(mode_bits::OFF);
      act_.reset();
      go(S::DONE, "미션표 완료");
      return;
    }
    begin_step(step_i_ + 1);
  }

  void fail_step(const std::string & why)
  {
    cancel_all();
    // skill 실패 → fallback 단계로 한 번 대체
    if (act_ && act_->step.type == StepType::SKILL && act_->step.fallback && !act_->fallback_used) {
      RCLCPP_WARN(get_logger(), "skill 실패 (%s) → fallback '%s'", why.c_str(), act_->step.fallback->text.c_str());
      act_->step = Step(*act_->step.fallback);
      act_->fallback_used = true;
      act_->start = now();
      phase(0);
      return;
    }
    const std::string name = act_ ? act_->step.text : "?";
    if (retries_ < max_retries_) {
      ++retries_;
      retry_index_ = step_i_;
      if (!in_tag_phase_ && act_ && act_->step.type == StepType::PICK) {retry_index_ = retry_index_for_pick(plan_, step_i_);}
      retry_origin_ = std::max(retry_origin_, step_i_);
      RCLCPP_WARN(get_logger(), "'%s' 실패 (%s) — 재시도 %d/%d%s", name.c_str(), why.c_str(), retries_, max_retries_,
        retry_index_ != step_i_ ? (" — 직전 '" + plan_[retry_index_].text + "' 부터").c_str() : "");
      reason_ = "재시도: " + why;
      retry_at_ = now() + rclcpp::Duration::from_seconds(retry_delay_);
      retry_pending_ = true;
      act_.reset();
      publish_status();
      return;
    }
    send_mode(mode_bits::OFF);
    act_.reset();
    go(S::FAILED, name + " 실패: " + why);
  }

  // ---------------- 10 Hz ----------------
  void tick()
  {
    if (fsm_.state() != S::RUNNING) {return;}
    const std::string block = blocking_reason();
    if (!block.empty()) {
      cancel_all();
      if (retry_pending_) {   // 재시도 대기 중이었으면 재개 때 그 단계부터
        retry_pending_ = false;
        if (in_tag_phase_) {begin_tag();} else {begin_step(retry_index_);}
      }
      go(S::PAUSED, block + " — 원인 해결 · 리셋 후 시작 버튼으로 재개");
      return;
    }
    if (retry_pending_) {
      if (now() < retry_at_) {return;}
      retry_pending_ = false;
      if (in_tag_phase_) {begin_tag();} else {begin_step(retry_index_);}
      return;
    }
    if (!act_) {return;}
    const double limit = act_->is_tag ? tag_timeout_ :
      act_->step.type == StepType::SKILL ? act_->step.value + 10.0 : timeouts_[act_->step.type];
    if ((now() - act_->start).seconds() > limit) {fail_step("단계 시간 초과"); return;}
    for (Op * o : ops()) {if (o->failed()) {fail_step(o->msg); return;}}
    if (act_->is_tag) {run_tag(); return;}
    run_step();
  }

  void run_step()
  {
    switch (act_->step.type) {
      case StepType::NAVIGATE: run_navigate(); break;
      case StepType::SEARCH: run_search(); break;
      case StepType::APPROACH: run_approach(); break;
      case StepType::PICK: run_pick(); break;
      case StepType::POSE: run_pose(); break;
      case StepType::SKILL: run_skill(); break;
    }
  }

  double phase_elapsed() const {return (now() - act_->phase_start).seconds();}
  bool fresh_result_since(const rclcpp::Time & t) const
  {
    if (!result_) {return false;}
    const rclcpp::Time st(result_->header.stamp, RCL_ROS_TIME);
    return st >= t && (now() - st).seconds() <= stale_s_;
  }

  // 공통 시작: TAG_VIEW → READ_TAG → (30° 회전 반복) → 미션표
  void run_tag()
  {
    switch (act_->phase) {
      case 0: phase(1); send_pose("TAG_VIEW"); break;
      case 1: if (pose_op_.success()) {phase(2); send_mode(mode_bits::READ_TAG);} break;
      case 2: if (mode_op_.success()) {phase(3);} break;
      case 3:
        if (fresh_result_since(act_->phase_start) && result_->tag_found) {
          tag_id_ = result_->tag_id;
          auto it = missions_.find(tag_id_);
          send_mode(mode_bits::OFF);
          if (it == missions_.end()) {
            act_.reset();
            go(S::FAILED, "미션표에 tag " + std::to_string(tag_id_) + " 없음");
            return;
          }
          RCLCPP_INFO(get_logger(), "tag %d → 단계 %zu개", tag_id_, it->second.size());
          plan_ = it->second;
          in_tag_phase_ = false;
          retries_ = 0;
          retry_origin_ = 0;
          begin_step(0);
        } else if (phase_elapsed() > tag_wait_) {
          if (act_->rotations >= tag_max_rot_) {fail_step("태그를 찾지 못함 (" + std::to_string(tag_max_rot_) + "번 회전)"); return;}
          ++act_->rotations;
          phase(4);
          rotate(tag_step_);
        }
        break;
      case 4: if (nav_op_.success()) {phase(3);} break;
    }
  }

  void run_navigate()
  {
    if (act_->phase == 0) {
      phase(1);
      send_nav(locations_[act_->step.arg]);
      ensure_pose("STOW");   // 주행 + 팔 STOW 병렬 (이미 STOW 면 생략)
      return;
    }
    if (nav_op_.success() && pose_op_.idle_or_success()) {step_done();}
  }

  // search: STOW → OFF(대상 · 기억 초기화) → SEARCH_FRONT|SEARCH_REAR → (45° 회전) → 물체 쪽 spin → SEARCH_FRONT 확인
  void run_search()
  {
    switch (act_->phase) {
      case 0: phase(1); ensure_pose("STOW"); break;
      case 1: if (pose_op_.idle_or_success()) {phase(2); send_mode(mode_bits::OFF);} break;
      case 2: if (mode_op_.success()) {phase(3); send_mode(mode_bits::SEARCH_FRONT | mode_bits::SEARCH_REAR);} break;
      case 3:
        if (mode_op_.done && !mode_op_.ok) {   // 후방 D435 가 없으면 전방만 (실패로 처리하지 않음)
          RCLCPP_WARN(get_logger(), "SEARCH_REAR 불가 (%s) — SEARCH_FRONT 만", mode_op_.msg.c_str());
          phase(4);
          send_mode(mode_bits::SEARCH_FRONT);
        } else if (mode_op_.success()) {
          phase(5);
        }
        break;
      case 4: if (mode_op_.success()) {phase(5);} break;
      case 5:
        if (fresh_result_since(act_->phase_start) && result_->object_found) {
          const auto & p = result_->object_pose.pose.position;
          const double ang = std::atan2(p.y, p.x);
          if (std::fabs(ang) > face_tol_) {phase(6); rotate(ang);}   // 후방에서 찾았어도 전방이 향하게
          else {phase(7); send_mode(mode_bits::SEARCH_FRONT);}
        } else if (phase_elapsed() > search_wait_) {
          if (act_->rotations >= search_max_rot_) {fail_step("물체를 찾지 못함"); return;}
          ++act_->rotations;
          phase(8);
          rotate(search_step_);
        }
        break;
      case 6: if (nav_op_.success()) {phase(7); send_mode(mode_bits::SEARCH_FRONT);} break;
      case 7: if (mode_op_.success()) {phase(9);} break;
      case 8: if (nav_op_.success()) {phase(5);} break;
      case 9:
        if (fresh_result_since(act_->phase_start) && result_->object_found && (result_->mode & mode_bits::SEARCH_FRONT)) {step_done();}
        else if (phase_elapsed() > confirm_s_) {fail_step("전방 D435 로 다시 확인 못함");}
        break;
    }
  }

  // approach d: ① STOW · SEARCH_FRONT → 먼 거리(범퍼) ② READY ③ SEARCH_FRONT|ARM_ALIGN → 정밀 d (pick 까지 유지)
  void run_approach()
  {
    const double d = act_->step.value;
    switch (act_->phase) {
      case 0: phase(1); ensure_pose("STOW"); break;
      case 1: if (pose_op_.idle_or_success()) {phase(2); send_mode(mode_bits::SEARCH_FRONT);} break;
      case 2:
        if (mode_op_.success()) {
          phase(3);
          send_align(std::max(ready_reach_ + d + approach_margin_, coarse_min_), coarse_tol_);
        }
        break;
      case 3: if (align_op_.success()) {phase(4); send_pose("READY");} break;
      case 4: if (pose_op_.success()) {phase(5); send_mode(mode_bits::SEARCH_FRONT | mode_bits::ARM_ALIGN);} break;
      case 5: if (mode_op_.success()) {phase(6); send_align(d, fine_tol_);} break;
      case 6: if (align_op_.success()) {step_done();} break;
    }
  }

  void run_pick()
  {
    if (act_->phase == 0) {phase(1); send_pick(); return;}
    if (pick_op_.success()) {
      last_pose_.clear();
      if (!pick_grasped_) {fail_step("grasped=false (빈손)"); return;}
      step_done();
    }
  }

  void run_pose()
  {
    if (act_->phase == 0) {phase(1); send_pose(act_->step.arg); return;}
    if (pose_op_.success()) {step_done();}
  }

  void run_skill()
  {
    const Step & st = act_->step;
    if (st.type != StepType::SKILL) {run_step(); return;}   // fallback 으로 바뀐 경우 (run_step 이 처리)
    switch (act_->phase) {
      case 0: phase(1); send_mode(st.has_mode ? st.mode : (mode_bits::SEARCH_FRONT | mode_bits::ARM_ALIGN)); break;
      case 1: if (mode_op_.success()) {phase(2); send_skill(st);} break;
      case 2:
        if (skill_op_.success()) {
          if (st.target == "manipulator") {last_pose_.clear();}
          step_done();
        }
        break;
    }
  }

  // ---------------- 제자리 회전: 같은 위치 + yaw (drive 가 nav2 spin 으로 처리) ----------------
  void rotate(double dyaw)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const double yaw = tf2::getYaw(t.transform.rotation) + dyaw;
      RCLCPP_INFO(get_logger(), "제자리 회전 %.0f°", dyaw * 180 / M_PI);
      send_nav({t.transform.translation.x, t.transform.translation.y, yaw});
    } catch (const tf2::TransformException & e) {
      nav_op_.start();
      nav_op_.complete(false, std::string("현재 위치 TF 없음 (") + map_frame_ + "→" + base_frame_ + ")");
    }
  }

  // ---------------- 비동기 요청 ----------------
  void send_mode(uint8_t mode)
  {
    const uint64_t g = gen_;
    mode_op_.start();
    if (!mode_client_->service_is_ready()) {mode_op_.complete(false, "/perception/set_mode 서비스 없음"); return;}
    auto req = std::make_shared<SetMode::Request>();
    req->mode = mode;
    mode_client_->async_send_request(req, [this, g](rclcpp::Client<SetMode>::SharedFuture f) {
        if (g != gen_) {return;}
        const auto r = f.get();
        mode_op_.complete(r->success, r->success ? "" : "모드 전환 실패: " + r->message);
      });
  }

  template<typename ActionT>
  void send(typename rclcpp_action::Client<ActionT>::SharedPtr client, const typename ActionT::Goal & goal, Op & op,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & handle, const std::string & name,
    std::function<bool(const typename ActionT::Result &)> ok_fn,
    std::function<std::string(const typename ActionT::Result &)> why_fn = nullptr)
  {
    const uint64_t g = gen_;
    op.start();
    if (!client->action_server_is_ready()) {op.complete(false, name + " 서버 없음"); return;}
    auto opts = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    opts.goal_response_callback = [this, g, &op, &handle, name](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr h) {
        if (g != gen_) {return;}
        if (!h) {op.complete(false, name + " goal 거부"); return;}
        handle = h;
      };
    opts.result_callback = [this, g, &op, &handle, name, ok_fn, why_fn](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & res) {
        if (g != gen_) {return;}
        handle.reset();
        const bool ok = res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result && ok_fn(*res.result);
        std::string why;
        if (!ok && res.result && why_fn) {why = why_fn(*res.result);}
        op.complete(ok, ok ? "" : name + " 실패" + (why.empty() ? "" : ": " + why));
      };
    client->async_send_goal(goal, opts);
  }

  void send_nav(const std::vector<double> & xyyaw)
  {
    Nav::Goal g;
    g.pose.header.frame_id = map_frame_;
    g.pose.header.stamp = now();
    g.pose.pose.position.x = xyyaw.size() > 0 ? xyyaw[0] : 0.0;
    g.pose.pose.position.y = xyyaw.size() > 1 ? xyyaw[1] : 0.0;
    const double yaw = xyyaw.size() > 2 ? xyyaw[2] : 0.0;
    g.pose.pose.orientation.z = std::sin(yaw / 2);
    g.pose.pose.orientation.w = std::cos(yaw / 2);
    send<Nav>(nav_client_, g, nav_op_, nav_h_, names::DRIVE_NAVIGATE, [](const Nav::Result &) {return true;});
  }
  void send_align(double dist, double tol)
  {
    Align::Goal g;
    g.target_distance = static_cast<float>(dist);
    g.tolerance = static_cast<float>(tol);
    send<Align>(align_client_, g, align_op_, align_h_, names::DRIVE_ALIGN, [](const Align::Result & r) {return r.success;},
      [](const Align::Result & r) {return r.message;});
  }
  void send_pose(const std::string & name)
  {
    ArmPose::Goal g;
    g.name = name;
    send<ArmPose>(pose_client_, g, pose_op_, pose_h_, "pose " + name, [this, name](const ArmPose::Result & r) {
        if (r.success) {last_pose_ = name;}
        return r.success;
      }, [](const ArmPose::Result & r) {return r.message;});
  }
  void ensure_pose(const std::string & name)
  {
    if (last_pose_ != name) {send_pose(name);}   // 이미 그 자세면 pose_op_ 를 쓰지 않음
  }
  void send_pick()
  {
    Pick::Goal g;   // target 비움 → 정밀 정렬 결과 → object_pose → 기억한 위치
    pick_grasped_ = false;
    send<Pick>(pick_client_, g, pick_op_, pick_h_, names::MANIP_PICK, [this](const Pick::Result & r) {
        pick_grasped_ = r.grasped;
        return r.success;
      }, [](const Pick::Result & r) {return r.message;});
  }
  void send_skill(const Step & st)
  {
    Skill::Goal g;
    g.policy = st.arg;
    g.timeout = static_cast<float>(st.value);
    skill_target_ = st.target;
    send<Skill>(skill_clients_[st.target], g, skill_op_, skill_h_, st.target + " skill " + st.arg,
      [](const Skill::Result & r) {return r.success;}, [](const Skill::Result & r) {return r.message;});
  }

  void cancel_all()
  {
    ++gen_;
    if (nav_h_) {nav_client_->async_cancel_goal(nav_h_); nav_h_.reset();}
    if (align_h_) {align_client_->async_cancel_goal(align_h_); align_h_.reset();}
    if (pose_h_) {pose_client_->async_cancel_goal(pose_h_); pose_h_.reset();}
    if (pick_h_) {pick_client_->async_cancel_goal(pick_h_); pick_h_.reset();}
    if (skill_h_) {skill_clients_[skill_target_]->async_cancel_goal(skill_h_); skill_h_.reset();}
    last_pose_.clear();   // 취소된 팔 자세는 알 수 없음
  }

  // 미션표
  TableContext ctx_;
  std::map<std::string, std::vector<double>> locations_;
  std::map<int, std::vector<Step>> missions_;
  // 파라미터
  double tag_wait_, tag_step_, search_wait_, search_step_, face_tol_, confirm_s_;
  double ready_reach_, approach_margin_, coarse_min_, fine_tol_, coarse_tol_, retry_delay_, stale_s_, diag_timeout_, tag_timeout_;
  int tag_max_rot_, search_max_rot_, max_retries_;
  std::string diag_prefix_, map_frame_, base_frame_;
  std::map<StepType, double> timeouts_;

  // 상태
  Fsm<S> fsm_ = make_mission_fsm();
  std::string reason_;
  int tag_id_ = -1;
  bool in_tag_phase_ = true;
  std::vector<Step> plan_;
  size_t step_i_ = 0;
  std::unique_ptr<Activity> act_;
  int retries_ = 0;
  bool retry_pending_ = false;
  size_t retry_index_ = 0;
  size_t retry_origin_ = 0;
  rclcpp::Time retry_at_{0, 0, RCL_ROS_TIME};
  uint64_t gen_ = 0;
  std::string last_pose_, skill_target_ = "drive";
  bool pick_grasped_ = false;
  Op nav_op_, align_op_, pose_op_, pick_op_, skill_op_, mode_op_;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr nav_h_;
  rclcpp_action::ClientGoalHandle<Align>::SharedPtr align_h_;
  rclcpp_action::ClientGoalHandle<ArmPose>::SharedPtr pose_h_;
  rclcpp_action::ClientGoalHandle<Pick>::SharedPtr pick_h_;
  rclcpp_action::ClientGoalHandle<Skill>::SharedPtr skill_h_;

  PerceptionResult::ConstSharedPtr result_;
  bool estop_ = false;
  bool have_diag_ = false, diag_error_ = false;
  std::string diag_what_;
  rclcpp::Time diag_rx_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr perc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  rclcpp_action::Client<Nav>::SharedPtr nav_client_;
  rclcpp_action::Client<Align>::SharedPtr align_client_;
  rclcpp_action::Client<ArmPose>::SharedPtr pose_client_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  std::map<std::string, rclcpp_action::Client<Skill>::SharedPtr> skill_clients_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};

}  // namespace center

RCLCPP_COMPONENTS_REGISTER_NODE(center::MissionNode)
