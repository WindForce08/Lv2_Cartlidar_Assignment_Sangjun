// MissionNode — 순서만 결정, 직접 움직이지 않음 (Mission 팀)
// 명세: docs/nodes/mission_node.md · docs/interfaces.md
//
// 흐름: /mission/start → pose(TAG_VIEW) → read_tag → tag_id로 미션표(missions.tag_<id>) 조회 → 단계 목록 실행
// - 바퀴는 drive_node, 팔은 manipulator_node에 Action으로 맡긴다. /cmd_vel · 관절 명령은 내지 않는다
// - 자세 값은 manipulator가 정한다. 정렬 거리는 미션 파라미터로 drive에 넘긴다 (align.coarse_distance · 미션표 approach <거리>)
// - robot_interfaces는 팀 공용 정의를 그대로 쓴다 (이 노드 때문에 바꾸지 않는다)
// - 병렬은 센서와 무관한 동작만 (navigate 중 팔 STOW). 정렬은 팔 동작이 끝난 뒤
// - 실패 · 타임아웃: 진행 중 goal 전부 cancel → 단계별 max_retries회 재시도 후 FAILED
//   pick 실패 · 빈손은 직전 search부터, skill 실패는 fallback 단계로 대체 (한 번)
// - /e_stop=true 또는 opencr 진단 ERROR: goal 전부 cancel → PAUSED. 자동 재개 없음, /mission/start로 재개
// - 결과를 콜백 안에서 기다리지 않는다. 각 결과는 result 콜백에서 처리한다
//
// 단계 문법 (mission_table.yaml의 missions.tag_<id> 문자열 배열)
//   navigate <location>          locations.<name> = [x, y, yaw]로 주행 + 팔 STOW (병렬)
//   search                        STOW → set_mode(OFF) → set_mode(SEARCH_FRONT | SEARCH_REAR),
//                                 object_found 연속 확인, 없으면 제자리 회전
//   approach <distance>           STOW → SEARCH_FRONT → align COARSE → READY → SEARCH_FRONT | ARM_ALIGN | READ_TAG → align FINE
//                                 <distance> = 1층 TFmini 렌즈 → 퍽이 놓인 박스 앞면 [m] (>= 0.13: 최소 측정 0.10 + 정지 여유)
//   pick                          SEARCH_FRONT | ARM_ALIGN | READ_TAG 유지, /manipulator/pick (grasped 확인)
//                                 READ_TAG: perception 이 Arducam 태그로 /arm_range (팔 끝 → 퍽) 를 정밀하게 낸다
//   pose <name>                   /manipulator/pose
//   skill <drive|manipulator> <policy> [timeout] [mode=이름+이름] [fallback=<단계>]
//                                 예: mode=SEARCH_FRONT+ARM_ALIGN, fallback=navigate:home (':' = 공백)
//
// plan 모드 (mission_source: plan, 주사위 기억 로봇 발제 v5) — 시작 단계가 tag 대신 observe → plan
//   observe                       팔 observe.arm_pose → READ_SIGNAL | READ_BARCODE → /perception/signal CONFIRMED +
//                                 /perception/barcode READ(read_ok) 둘 다 받으면 성공. MISMATCH 면 안전 정지 (다시 보여주기 없음)
//   plan                          /mission_planner/plan (std_srvs/Trigger) → message 의 단계 문장 (줄바꿈) 을 뒤에 붙임.
//                                 실패 (풀 수 없음 · 미등록 바코드 등) 면 재시도 없이 FAILED (발제: 오류 기록 후 안전 정지)
//   goto <x> <y> <yaw>            navigate 와 같고 위치 이름 대신 지도 좌표 (계획기 접근 자세표)
//   pick_target <x> <y> <z>       pick 과 같고 /manipulator/pick target = 지도 좌표 퍽 중심
//   verify <L> <cx> <cy> <z> <expect9> <yellow_shape>
//                                 팔 verify.arm_pose → verify.mode → verify.settle_s 기다림 → /perception/check_dice.
//                                 칸 = (cx, cy) ± 0.06 격자 (행 우선, 위 줄 +Y 부터, 줄 안은 -X 부터), expect9 = 칸마다 0 빈칸 · 1 R · 2 G · 3 Y.
//                                 결과는 로그 (log_dir/mission_log.jsonl) 에 남기고, 틀려도 verify.stop_on_fail=false 면 계속 (복귀 점수)
//   wait <s>                      제자리 대기 (복귀 후 정지 등)
//   퍽 측량 (survey.enabled, 이슈 #39 #53) — observe 와 plan 사이에 붙는다:
//   survey_start                  /perception/survey/start (std_srvs/Trigger) → survey.mode 켜기
//   survey_view <box> <arm_pose>  팔 자세 → survey.mode → survey.settle_s 기다림 (perception_survey 가 /perception/pucks 를 쌓음)
//                                 박스마다 goto (survey.sides 면의 approach_poses.yaml stage) [+ approach survey.approach_distance] 뒤에 붙는다
//   survey_finish                 /perception/survey/finish. 성공 = 12칸 확정 → 완료 파일 (message = 경로) 을 계획기 퍽 등록표
//                                 (survey.pucks_out, mission_planner pucks_file 과 같은 경로) 로 바꿔 씀.
//                                 실패면 /perception/survey/revisit → 문제 칸이 있는 박스만 survey.revisit_arm_pose ·
//                                 survey.revisit_approach_distance (각도 · 거리를 바꿈) 로 다시 보고 finish (#53).
//                                 다시 볼 칸이 없고 미확인 칸만 남으면 survey.on_unconfirmed: fail (안전 정지) | registered (등록표 그대로 계획)
//   장소 태그 (/perception/task_marker, READ_MARKER, #39) — L1 · L2 · L3 = marker.ids (12 · 13 · 14):
//     verify 에서 verify.check_marker 면 READ_MARKER 를 같이 켜고, 새로 본 태그 ID · 위치 (base_link · map) 를
//     check_dice 결과와 같은 줄에 기록 (발제 채점: 새 관측 기반 장소별 ID · 위치). ID 가 다르면 verify.stop_on_fail 에 따름
//   check_marker <L>              marker.check_before_place 면 그 장소 첫 place_<L>_* skill 앞에 자동으로 붙음:
//                                 READ_MARKER → 기대 ID 를 보면 성공, 다른 ID 면 실패 (엉뚱한 장소에 놓지 않게)
//
// callback group
// - mission_cb_group_: Action · Service 클라이언트 · 서버, 타이머. 상태는 이 그룹에서만 바뀐다
// - sensor_cb_group_: /perception/result · /e_stop · /diagnostics 구독. sensor_mutex_ · atomic으로 보호

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "action_msgs/srv/cancel_goal.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_interfaces/action/align_base.hpp"
#include "robot_interfaces/action/arm_pose.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/action/pick.hpp"
#include "robot_interfaces/msg/mission_status.hpp"
#include "robot_interfaces/msg/barcode.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/msg/signal_sequence.hpp"
#include "robot_interfaces/msg/task_marker.hpp"
#include "robot_interfaces/srv/check_dice.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.hpp"
#include "yaml-cpp/yaml.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mission
{

class MissionNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using AlignBase = robot_interfaces::action::AlignBase;
  using ArmPose = robot_interfaces::action::ArmPose;
  using Pick = robot_interfaces::action::Pick;
  using ExecuteSkill = robot_interfaces::action::ExecuteSkill;
  using PerceptionResult = robot_interfaces::msg::PerceptionResult;
  using MissionStatus = robot_interfaces::msg::MissionStatus;
  using SetPerceptionMode = robot_interfaces::srv::SetPerceptionMode;
  using Mode = SetPerceptionMode::Request;
  using Trigger = std_srvs::srv::Trigger;
  using SignalSequence = robot_interfaces::msg::SignalSequence;
  using Barcode = robot_interfaces::msg::Barcode;
  using TaskMarker = robot_interfaces::msg::TaskMarker;
  using CheckDice = robot_interfaces::srv::CheckDice;
  using Report = std::function<void (bool ok, const std::string & message)>;

  explicit MissionNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mission_node", options)
  {
    loadParameters();
    validation();

    mission_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- 구독 ---
    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = sensor_cb_group_;
    // 계약: Reliable, depth 1 (오래된 값은 쌓지 않는다)
    result_sub_ = create_subscription<PerceptionResult>(
      "/perception/result", rclcpp::QoS(1).reliable(),
      [this](PerceptionResult::ConstSharedPtr msg) {onPerceptionResult(msg);}, sub_opt);
    // 계약: Reliable + TransientLocal (늦게 떠도 마지막 E-stop 상태를 받는다)
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/e_stop", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {estop_active_ = msg->data;}, sub_opt);
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {onDiagnostics(msg);},
      sub_opt);
    if (plan_mode_) {
      // plan 모드: 신호 · 바코드 (계약: Reliable, depth 1)
      signal_sub_ = create_subscription<SignalSequence>(
        "/perception/signal", rclcpp::QoS(1).reliable(),
        [this](SignalSequence::ConstSharedPtr msg) {onSignal(msg);}, sub_opt);
      barcode_sub_ = create_subscription<Barcode>(
        "/perception/barcode", rclcpp::QoS(1).reliable(),
        [this](Barcode::ConstSharedPtr msg) {onBarcode(msg);}, sub_opt);
      marker_sub_ = create_subscription<TaskMarker>(
        "/perception/task_marker", rclcpp::QoS(1).reliable(),
        [this](TaskMarker::ConstSharedPtr msg) {onMarker(msg);}, sub_opt);
    }

    // --- 클라이언트 ---
    mode_client_ = create_client<SetPerceptionMode>(
      "/perception/set_mode", rclcpp::ServicesQoS(), mission_cb_group_);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/drive/navigate", mission_cb_group_);
    align_client_ = rclcpp_action::create_client<AlignBase>(this, "/drive/align", mission_cb_group_);
    drive_skill_client_ =
      rclcpp_action::create_client<ExecuteSkill>(this, "/drive/execute_skill", mission_cb_group_);
    pose_client_ = rclcpp_action::create_client<ArmPose>(this, "/manipulator/pose", mission_cb_group_);
    pick_client_ = rclcpp_action::create_client<Pick>(this, "/manipulator/pick", mission_cb_group_);
    arm_skill_client_ =
      rclcpp_action::create_client<ExecuteSkill>(this, "/manipulator/execute_skill", mission_cb_group_);
    if (plan_mode_) {
      plan_client_ = create_client<Trigger>(plan_service_, rclcpp::ServicesQoS(), mission_cb_group_);
      dice_client_ = create_client<CheckDice>("/perception/check_dice", rclcpp::ServicesQoS(), mission_cb_group_);
      if (survey_enabled_) {
        survey_start_client_ = create_client<Trigger>("/perception/survey/start", rclcpp::ServicesQoS(), mission_cb_group_);
        survey_finish_client_ = create_client<Trigger>("/perception/survey/finish", rclcpp::ServicesQoS(), mission_cb_group_);
        survey_revisit_client_ =
          create_client<Trigger>("/perception/survey/revisit", rclcpp::ServicesQoS(), mission_cb_group_);
      }
    }

    // --- 서버: 시작 · 정지 (joy_teleop 버튼 또는 CLI에서 호출) ---
    start_srv_ = create_service<Trigger>(
      "/mission/start",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {onStart(res);},
      rclcpp::ServicesQoS(), mission_cb_group_);
    stop_srv_ = create_service<Trigger>(
      "/mission/stop",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {onStop(res);},
      rclcpp::ServicesQoS(), mission_cb_group_);

    status_pub_ = create_publisher<MissionStatus>("/mission/status", rclcpp::QoS(10));

    start_requested_ = get_parameter("auto_start").as_bool();
    tick_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {onTick();},
      mission_cb_group_);
    status_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.5), [this]() {publishStatus();},
      mission_cb_group_);

    RCLCPP_INFO(get_logger(), "mission_node ready (source: %s, missions: %zu, auto_start: %s)",
      plan_mode_ ? "plan" : "tag", missions_.size(), start_requested_ ? "true" : "false");
  }

private:
  enum class Run { IDLE, RUNNING, PAUSED, DONE, FAILED };
  enum class AlignStage { COARSE, FINE };   // 로그 · 순서용 (AlignBase 필드 아님)
  // 정밀 정렬 · pick 모드. READ_TAG 는 perception /arm_range(팔 끝 → 퍽)를 태그로 정밀하게 하려고 켠다
  static constexpr uint8_t kFineMode = Mode::SEARCH_FRONT | Mode::ARM_ALIGN | Mode::READ_TAG;

  struct Step
  {
    std::string type;                 // pose · read_tag · navigate · search · approach · pick · skill
    std::vector<std::string> args;
    std::string mode_text;            // skill 전용: mode=이름+이름 (없으면 SEARCH_FRONT | ARM_ALIGN)
    std::string fallback;             // skill 전용: 실패 시 대신 실행할 단계 (문법 동일)
    std::string text;                 // 원문 (로그 · status)
  };

  // read_tag · search: 결과 확인 횟수와 못 찾았을 때의 제자리 회전
  struct Rotate
  {
    int confirm_count{2};
    double view_wait{2.0};
    double rotate_step{0.0};   // rad
    int max_rotations{0};
  };

  static uint8_t runToMsg(Run r)
  {
    switch (r) {
      case Run::IDLE: return MissionStatus::IDLE;
      case Run::RUNNING: return MissionStatus::RUNNING;
      case Run::PAUSED: return MissionStatus::PAUSED;
      case Run::DONE: return MissionStatus::DONE;
      case Run::FAILED: return MissionStatus::FAILED;
    }
    return MissionStatus::IDLE;
  }

  // ------------------------------------------------------------------ 파라미터
  void loadParameters()
  {
    declare_parameter<bool>("auto_start", false);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    behavior_tree_ = declare_parameter<std::string>("behavior_tree", "");

    // 위치표: locations.names = [home, L1, ...], locations.<name> = [x, y, yaw(rad)] (map = 발제 좌표, base_link 자세)
    //   값은 mission_table.yaml (#30: 인지 파트 Nav2 모의로 확인한 목표, 배치가 바뀌면 다시 계산)
    const auto loc_names = declare_parameter<std::vector<std::string>>("locations.names", {"home"});
    for (const auto & n : loc_names) {
      const auto v = declare_parameter<std::vector<double>>("locations." + n, {0.0, 0.0, 0.0});
      if (v.size() != 3) {
        throw std::invalid_argument("locations." + n + " must be [x, y, yaw]");
      }
      locations_[n] = v;
    }

    // 미션표: missions.tag_ids = [7, 12], missions.tag_<id> = ["navigate start_box_1", "search", ...]
    const auto tag_ids = declare_parameter<std::vector<int64_t>>("missions.tag_ids", std::vector<int64_t>{});
    for (int64_t id : tag_ids) {
      const auto lines = declare_parameter<std::vector<std::string>>(
        "missions.tag_" + std::to_string(id), std::vector<std::string>{});
      std::vector<Step> steps;
      for (const auto & line : lines) {
        steps.push_back(parseStep(line));
      }
      if (steps.empty()) {
        throw std::invalid_argument("missions.tag_" + std::to_string(id) + " has no steps");
      }
      missions_[static_cast<int>(id)] = steps;
    }

    // 단계별 타임아웃 (s) · 재시도
    timeout_["pose"] = declare_parameter<double>("timeout.pose", 20.0);
    timeout_["read_tag"] = declare_parameter<double>("timeout.read_tag", 90.0);  // 회전 탐색 포함
    timeout_["navigate"] = declare_parameter<double>("timeout.navigate", 120.0);
    timeout_["search"] = declare_parameter<double>("timeout.search", 60.0);      // 회전 탐색 포함
    timeout_["approach"] = declare_parameter<double>("timeout.approach", 90.0);  // COARSE + READY + FINE
    timeout_["pick"] = declare_parameter<double>("timeout.pick", 60.0);
    skill_default_timeout_ = declare_parameter<double>("skill.default_timeout", 30.0);
    skill_timeout_margin_ = declare_parameter<double>("skill.timeout_margin", 5.0);
    max_retries_ = declare_parameter<int>("max_retries", 2);
    retry_delay_ = declare_parameter<double>("retry_delay", 1.0);

    // 300ms 규칙 (계약): header.stamp 기준 이보다 오래된 인식 결과는 버린다
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.3);

    // read_tag: 태그가 view_wait 안에 안 보이면 제자리에서 rotate_step만큼 회전, 최대 max_rotations
    tag_.confirm_count = declare_parameter<int>("tag.confirm_count", 2);
    tag_.view_wait = declare_parameter<double>("tag.view_wait", 2.0);
    tag_.rotate_step = declare_parameter<double>("tag.rotate_step_deg", 30.0) * M_PI / 180.0;
    tag_.max_rotations = declare_parameter<int>("tag.max_rotations", 11);

    // search: object_found가 연속 N번이면 찾음. view_wait 안에 없으면 rotate_step 회전, 최대 max_rotations
    search_.confirm_count = declare_parameter<int>("search.confirm_count", 3);
    search_.view_wait = declare_parameter<double>("search.view_wait", 2.0);
    search_.rotate_step = declare_parameter<double>("search.rotate_step_deg", 45.0) * M_PI / 180.0;
    search_.max_rotations = declare_parameter<int>("search.max_rotations", 4);

    // align tolerance: 0 = drive 기본값 (drive_node align.default_tolerance)
    align_tolerance_ = declare_parameter<double>("align.tolerance", 0.0);
    // 먼 거리 정렬(COARSE) 목표 거리 (m, 범퍼 기준). drive_node는 align.coarse_min(0.3)보다 작으면 실패시킨다
    // TODO(Mission · Controller): 값은 Controller(drive · 팔 길이)와 맞출 것 (추정)
    coarse_distance_ = declare_parameter<double>("align.coarse_distance", 0.45);
    // pick: grasped=false면 실패로 본다
    require_grasped_ = declare_parameter<bool>("pick.require_grasped", true);

    // /diagnostics: name_prefix로 시작하는 항목만 본다 (계약 "opencr/").
    // timeout > 0이면 그 시간 동안 opencr 진단이 안 오면 ERROR. 0 = 끔 (opencr 발행 확인 전)
    diag_prefix_ = declare_parameter<std::string>("diagnostics.name_prefix", "opencr/");
    diag_timeout_ = declare_parameter<double>("diagnostics.timeout", 3.0);

    // plan 모드 (주사위 기억 로봇): tag 대신 observe → plan (mission_planner_node) 으로 단계를 받는다
    const auto source = declare_parameter<std::string>("mission_source", "tag");
    if (source != "tag" && source != "plan") {throw std::invalid_argument("mission_source must be tag or plan");}
    plan_mode_ = source == "plan";
    plan_service_ = declare_parameter<std::string>("plan.service", "/mission_planner/plan");
    timeout_["observe"] = declare_parameter<double>("timeout.observe", 150.0);  // 점멸 12회 x 2 s x 2 묶음 + 텀 + 바코드
    timeout_["plan"] = declare_parameter<double>("timeout.plan", 60.0);
    timeout_["verify"] = declare_parameter<double>("timeout.verify", 40.0);
    observe_arm_pose_ = declare_parameter<std::string>("observe.arm_pose", "TAG_VIEW");
    verify_arm_pose_ = declare_parameter<std::string>("verify.arm_pose", "TAG_VIEW");
    // 신호 묶음을 이만큼 (점멸 수가 맞는 것) 보고도 확정이 안 되면 안전 정지. 한 묶음 약 26 s (12 x 2 s + 3 s 텀 - 1 s)
    observe_max_cycles_ = declare_parameter<int>("observe.max_cycles", 4);
    verify_mode_text_ = declare_parameter<std::string>("verify.mode", "READ_TAG+SEARCH_FRONT");
    verify_settle_ = declare_parameter<double>("verify.settle_s", 2.0);
    verify_tolerance_ = declare_parameter<double>("verify.tolerance", 0.0);   // 0 = perception 설정값
    verify_stop_on_fail_ = declare_parameter<bool>("verify.stop_on_fail", false);
    log_dir_ = declare_parameter<std::string>("log_dir", "");                // 비우면 기록 안 함

    // 퍽 측량 (perception_survey, #39 #53). 박스 1~6 마다 조사 자세 [x, y, yaw] (지도 = 발제 좌표)
    //   = 계획기 접근 자세표 (mission_planner approach_poses.yaml) 의 B<box>:<side> stage 를 읽음 (한 곳에서만 고치게).
    //   survey.poses 를 주면 그 값을 대신 씀 (자세표 없이 시험할 때)
    survey_enabled_ = declare_parameter<bool>("survey.enabled", false);
    survey_boxes_ = declare_parameter<std::vector<int64_t>>("survey.boxes", {1, 2, 3, 4, 5, 6});
    survey_sides_ = declare_parameter<std::vector<std::string>>("survey.sides", {"S", "S", "S", "S", "N", "W"});
    survey_poses_file_ = declare_parameter<std::string>("survey.poses_file", "");   // 빈 문자열 = mission_planner share
    survey_poses_ = declare_parameter<std::vector<double>>("survey.poses", std::vector<double>{});
    survey_approach_ = declare_parameter<double>("survey.approach_distance", 0.13);   // 0 = 조사 자세에서 바로 봄
    survey_arm_pose_ = declare_parameter<std::string>("survey.arm_pose", "BOX_VIEW");
    // 다시 보기 (#53): 처음과 다른 각도 · 거리로 문제 칸을 봄
    survey_revisit_arm_pose_ = declare_parameter<std::string>("survey.revisit_arm_pose", "TAG_VIEW");
    survey_revisit_approach_ = declare_parameter<double>("survey.revisit_approach_distance", 0.20);
    survey_mode_text_ = declare_parameter<std::string>("survey.mode", "READ_TAG+SEARCH_FRONT");
    survey_settle_ = declare_parameter<double>("survey.settle_s", 2.0);
    survey_max_rounds_ = declare_parameter<int>("survey.max_rounds", 2);              // 다시 보기 횟수 (perception max_revisits)
    survey_on_unconfirmed_ = declare_parameter<std::string>("survey.on_unconfirmed", "fail");   // fail | registered
    survey_pucks_out_ = declare_parameter<std::string>("survey.pucks_out", "/tmp/mission_log/pucks_survey.yaml");
    // 장소 태그 (#39): L1 · L2 · L3 의 태그 ID
    marker_ids_ = declare_parameter<std::vector<int64_t>>("marker.ids", {12, 13, 14});
    verify_check_marker_ = declare_parameter<bool>("verify.check_marker", true);
    verify_marker_wait_ = declare_parameter<double>("verify.marker_wait_s", 3.0);    // settle 뒤 태그를 더 기다리는 시간
    marker_check_before_place_ = declare_parameter<bool>("marker.check_before_place", false);   // ★ 놓는 자리에서 태그가 보이면 true
    timeout_["check_marker"] = declare_parameter<double>("timeout.check_marker", 10.0);
    timeout_["survey"] = declare_parameter<double>("timeout.survey", 20.0);          // start · finish 서비스
    timeout_["survey_view"] = declare_parameter<double>("timeout.survey_view", 30.0);
  }

  // 계획기 접근 자세표에서 survey.boxes 마다 B<box>:<side> 의 stage [x, y, yaw] 를 읽음. 없으면 노드 시작 실패
  void loadSurveyPoses()
  {
    if (survey_sides_.size() != survey_boxes_.size()) {
      throw std::invalid_argument("survey.sides must have one side (S | N | W | E) for each of survey.boxes");
    }
    std::string file = survey_poses_file_;
    try {
      if (file.empty()) {
        file = (ament_index_cpp::get_package_share_path("mission_planner") / "config" / "approach_poses.yaml").string();
      }
      const YAML::Node doc = YAML::LoadFile(file);
      for (size_t i = 0; i < survey_boxes_.size(); ++i) {
        const std::string key = "B" + std::to_string(survey_boxes_[i]) + ":" + survey_sides_[i];
        const YAML::Node a = doc[key];
        if (!a || !a["stage"] || a["stage"].size() != 3) {
          throw std::invalid_argument(key + " stage [x, y, yaw] not in " + file);
        }
        if (a["ok"] && !a["ok"].as<bool>()) {
          throw std::invalid_argument(key + " is not reachable (ok: false) in " + file);
        }
        for (const auto & v : a["stage"]) {survey_poses_.push_back(v.as<double>());}
      }
    } catch (const std::exception & e) {   // 패키지 · 파일 없음, YAML 오류, 키 없음
      throw std::invalid_argument("survey poses (" + (file.empty() ? std::string("mission_planner") : file) + "): " +
              e.what());
    }
    RCLCPP_INFO(get_logger(), "survey: %zu pose(s) from %s", survey_boxes_.size(), file.c_str());
  }

  void validation()
  {
    for (const auto & [name, v] : locations_) {
      for (double x : v) {
        if (!std::isfinite(x)) {throw std::invalid_argument("locations." + name + " must be finite");}
      }
    }
    for (const auto & [type, sec] : timeout_) {
      if (!std::isfinite(sec) || sec <= 0.0) {
        throw std::invalid_argument("timeout." + type + " must be > 0");
      }
    }
    if (max_retries_ < 0) {throw std::invalid_argument("max_retries must be >= 0");}
    if (!std::isfinite(retry_delay_) || retry_delay_ < 0.0) {
      throw std::invalid_argument("retry_delay must be >= 0");
    }
    if (!(stale_timeout_ > 0.0)) {throw std::invalid_argument("stale_timeout must be > 0");}
    for (const auto * r : {&tag_, &search_}) {
      if (r->confirm_count < 1 || !(r->view_wait > 0.0) || r->max_rotations < 0 ||
        !std::isfinite(r->rotate_step))
      {
        throw std::invalid_argument(
                "tag.* / search.*: confirm_count >= 1, view_wait > 0, max_rotations >= 0");
      }
    }
    if (!std::isfinite(align_tolerance_) || align_tolerance_ < 0.0) {
      throw std::invalid_argument("align.tolerance must be >= 0 (0 = drive default)");
    }
    if (!std::isfinite(coarse_distance_) || coarse_distance_ < 0.3) {
      throw std::invalid_argument("align.coarse_distance must be >= 0.3 (drive_node align.coarse_min)");
    }
    if (!(skill_default_timeout_ > 0.0) || skill_timeout_margin_ < 0.0) {
      throw std::invalid_argument("skill.default_timeout must be > 0, skill.timeout_margin >= 0");
    }
    if (!std::isfinite(diag_timeout_) || diag_timeout_ < 0.0) {
      throw std::invalid_argument("diagnostics.timeout must be >= 0 (0 = off)");
    }
    if (plan_mode_ && survey_enabled_) {
      for (int64_t b : survey_boxes_) {
        if (b < 1 || b > 6) {throw std::invalid_argument("survey.boxes must be 1..6");}
      }
      if (survey_poses_.empty()) {loadSurveyPoses();}
      if (survey_poses_.size() != 3 * survey_boxes_.size()) {
        throw std::invalid_argument("survey.poses must have [x, y, yaw] for each of survey.boxes");
      }
      if (!parseModeNames(survey_mode_text_)) {throw std::invalid_argument("survey.mode: names joined by '+'");}
      auto approach_ok = [](double d) {return d == 0.0 || d >= 0.13;};   // drive FINE 하한 (TFmini)
      if (!(survey_settle_ >= 0.0) || survey_max_rounds_ < 0 || !approach_ok(survey_approach_) ||
        !approach_ok(survey_revisit_approach_))
      {
        throw std::invalid_argument("survey.settle_s >= 0, survey.max_rounds >= 0, "
                "survey.approach_distance · revisit_approach_distance 0 or >= 0.13");
      }
      if (survey_on_unconfirmed_ != "fail" && survey_on_unconfirmed_ != "registered") {
        throw std::invalid_argument("survey.on_unconfirmed must be fail or registered");
      }
      if (survey_pucks_out_.empty()) {throw std::invalid_argument("survey.pucks_out must be set (planner pucks_file)");}
    }
    if (plan_mode_) {
      if (observe_max_cycles_ < 2) {throw std::invalid_argument("observe.max_cycles must be >= 2 (CONFIRMED needs 2)");}
      if (marker_ids_.size() != 3) {throw std::invalid_argument("marker.ids must be [L1, L2, L3] tag ids");}
      if (!(verify_marker_wait_ >= 0.0)) {throw std::invalid_argument("verify.marker_wait_s must be >= 0");}
      if (!parseModeNames(verify_mode_text_)) {throw std::invalid_argument("verify.mode: names joined by '+'");}
      if (!(verify_settle_ >= 0.0) || !(verify_tolerance_ >= 0.0)) {
        throw std::invalid_argument("verify.settle_s · verify.tolerance must be >= 0");
      }
    }
    // 미션표의 위치 이름 · 단계 인자 확인 (현장에서 오타로 멈추지 않게 시작 때 거부)
    for (const auto & [id, steps] : missions_) {
      for (const auto & s : steps) {
        checkStep(s, "missions.tag_" + std::to_string(id));
        if (!s.fallback.empty()) {checkStep(parseStep(s.fallback), "fallback of '" + s.text + "'");}
      }
    }
  }

  // "skill manipulator pick_cup_v1 20 mode=SEARCH_FRONT fallback=navigate:home" → Step
  static Step parseStep(const std::string & line)
  {
    Step s;
    s.text = line;
    std::istringstream in(line);
    std::string tok;
    while (in >> tok) {
      if (tok.rfind("fallback=", 0) == 0) {
        s.fallback = tok.substr(9);
        for (auto & c : s.fallback) {if (c == ':') {c = ' ';}}
      } else if (tok.rfind("mode=", 0) == 0) {
        s.mode_text = tok.substr(5);
      } else if (s.type.empty()) {
        s.type = tok;
      } else {
        s.args.push_back(tok);
      }
    }
    return s;
  }

  // "SEARCH_FRONT+ARM_ALIGN" → 3. 이름만 허용 (숫자 불가), 모르는 이름이면 nullopt
  static std::optional<uint8_t> parseModeNames(const std::string & text)
  {
    static const std::map<std::string, uint8_t> names = {
      {"OFF", Mode::OFF}, {"SEARCH_FRONT", Mode::SEARCH_FRONT}, {"ARM_ALIGN", Mode::ARM_ALIGN},
      {"READ_TAG", Mode::READ_TAG}, {"SEARCH_REAR", Mode::SEARCH_REAR}, {"READ_MARKER", Mode::READ_MARKER},
      {"READ_SIGNAL", Mode::READ_SIGNAL}, {"READ_BARCODE", Mode::READ_BARCODE}};
    if (text.empty()) {return std::nullopt;}
    uint8_t mode = 0;
    std::istringstream in(text);
    std::string name;
    while (std::getline(in, name, '+')) {
      auto it = names.find(name);
      if (it == names.end()) {return std::nullopt;}
      mode |= it->second;
    }
    return mode;
  }

  static bool isNumber(const std::string & s, double & out)
  {
    try {
      size_t pos = 0;
      out = std::stod(s, &pos);
      return pos == s.size() && std::isfinite(out);
    } catch (const std::exception &) {
      return false;
    }
  }

  void checkStep(const Step & s, const std::string & where) const
  {
    auto bad = [&](const std::string & why) {
        throw std::invalid_argument(where + ": '" + s.text + "' " + why);
      };
    double v = 0.0;
    if ((!s.mode_text.empty() || !s.fallback.empty()) && s.type != "skill") {
      bad("mode= / fallback= are only for skill steps");
    }
    if (s.type == "navigate") {
      if (s.args.size() != 1 || locations_.count(s.args[0]) == 0) {bad("needs a name from locations.names");}
    } else if (s.type == "pose") {
      if (s.args.size() != 1) {bad("needs a pose name");}
    } else if (s.type == "approach") {
      // FINE 정렬 거리는 미션표에서 준다 (TFmini 렌즈 → 박스 앞면). drive align.min_target_distance 와 같게
      if (s.args.size() != 1 || !isNumber(s.args[0], v) || v < 0.13) {
        bad("needs <distance> >= 0.13 (FINE align, TFmini lens to box front)");
      }
    } else if (s.type == "skill") {
      if (s.args.size() < 2 || s.args.size() > 3 || (s.args[0] != "drive" && s.args[0] != "manipulator")) {
        bad("needs <drive|manipulator> <policy> [timeout]");
      }
      if (s.args.size() == 3 && (!isNumber(s.args[2], v) || v <= 0.0)) {bad("timeout must be > 0");}
      if (!s.mode_text.empty() && !parseModeNames(s.mode_text)) {
        bad("mode= takes names joined by '+' (OFF · SEARCH_FRONT · ARM_ALIGN · READ_TAG · SEARCH_REAR · "
          "READ_MARKER · READ_SIGNAL · READ_BARCODE), no numbers");
      }
    } else if (s.type == "check_marker") {
      if (s.args.size() != 1 || markerIdOf(s.args[0]) < 0) {bad("needs <L1|L2|L3>");}
    } else if (s.type == "survey_view") {
      if (s.args.size() != 2 || !isNumber(s.args[0], v) || v < 1 || v > 6) {bad("needs <box 1-6> <arm_pose>");}
    } else if (s.type == "search" || s.type == "pick" || s.type == "observe" || s.type == "plan" ||
      s.type == "survey_start" || s.type == "survey_finish")
    {
      if (!s.args.empty()) {bad("takes no arguments");}
    } else if (s.type == "goto" || s.type == "pick_target") {
      if (s.args.size() != 3 || !isNumber(s.args[0], v) || !isNumber(s.args[1], v) || !isNumber(s.args[2], v)) {
        bad("needs <x> <y> <" + std::string(s.type == "goto" ? "yaw" : "z") + "> (map frame)");
      }
    } else if (s.type == "verify") {
      bool ok = s.args.size() == 6 && isNumber(s.args[1], v) && isNumber(s.args[2], v) && isNumber(s.args[3], v) &&
        s.args[4].size() == 9 && s.args[4].find_first_not_of("0123") == std::string::npos &&
        (s.args[5] == "0" || s.args[5] == "1" || s.args[5] == "2");
      if (!ok) {bad("needs <L> <cx> <cy> <z> <expect9: 0-3 x 9> <yellow_shape 0|1|2>");}
    } else if (s.type == "wait") {
      if (s.args.size() != 1 || !isNumber(s.args[0], v) || v < 0.0) {bad("needs <seconds> >= 0");}
    } else {
      bad("unknown step type (navigate · search · approach · pick · pose · skill · "
        "observe · plan · goto · pick_target · verify · wait)");
    }
  }

  // ------------------------------------------------------------------ 구독
  // 300ms 규칙 (계약, 예외 없음): header.stamp = ROS 시계 기준 촬영 시각. stamp가 비어 있으면 측정 시각을
  // 알 수 없으므로 오래된 값과 같이 버린다
  bool isFresh(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {return false;}
    return (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds() <= stale_timeout_;
  }

  void onPerceptionResult(PerceptionResult::ConstSharedPtr msg)
  {
    const bool fresh = isFresh(msg->header.stamp);
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!fresh) {
      // 버린 결과가 끼면 "연속 N번"이 끊긴 것으로 본다
      found_count_ = 0;
      tag_count_ = 0;
      last_tag_id_ = -1;
      return;
    }
    if (msg->mode & (Mode::SEARCH_FRONT | Mode::SEARCH_REAR)) {
      found_count_ = msg->object_found ? found_count_ + 1 : 0;
    }
    if (msg->mode & Mode::READ_TAG) {
      if (msg->tag_found && msg->tag_id == last_tag_id_) {
        ++tag_count_;
      } else {
        tag_count_ = msg->tag_found ? 1 : 0;
        last_tag_id_ = msg->tag_found ? msg->tag_id : -1;
      }
    }
  }

  // plan 모드: 신호 순서 (CONFIRMED · MISMATCH) 와 바코드 (READ) 는 observe 단계가 시작된 뒤 것만 쓴다
  void onSignal(SignalSequence::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    signal_status_ = msg->status;
    signal_seq_ = msg->sequence;
    signal_cycles_ = msg->cycles;
    signal_rx_ = now();
  }

  // 장소 태그: 보인 것만 (found) 300ms 규칙으로 걸러 기억
  void onMarker(TaskMarker::ConstSharedPtr msg)
  {
    if (!msg->found || !isFresh(msg->header.stamp)) {return;}
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    last_marker_ = msg;
    marker_rx_ = now();
  }

  void onBarcode(Barcode::ConstSharedPtr msg)
  {
    if (msg->source != Barcode::READ || !msg->read_ok || msg->text.empty()) {return;}
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    barcode_text_ = msg->text;
    barcode_rx_ = now();
  }

  // name_prefix("opencr/")로 시작하는 항목만 본다. 다른 패키지(controller_manager 등) 진단은 무시
  void onDiagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
  {
    bool seen = false;
    std::string err;
    for (const auto & st : msg->status) {
      if (st.name.rfind(diag_prefix_, 0) != 0) {continue;}
      seen = true;
      if (err.empty() && st.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        err = st.name + ": " + st.message;
      }
    }
    if (!seen) {return;}
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    diag_error_ = err;
    diag_rx_ = now();
  }

  // 정지 사유 (없으면 빈 문자열)
  std::string safetyReason()
  {
    if (estop_active_) {return "E-stop active";}
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!diag_error_.empty()) {return "diagnostics ERROR (" + diag_error_ + ")";}
    if (diag_timeout_ > 0.0 &&
      (diag_rx_.nanoseconds() == 0 || (now() - diag_rx_).seconds() > diag_timeout_))
    {
      return "no " + diag_prefix_ + "* diagnostics for " + std::to_string(diag_timeout_) + "s";
    }
    return "";
  }

  // ------------------------------------------------------------------ 시작 · 정지
  void onStart(Trigger::Response::SharedPtr res)
  {
    if (run_ == Run::RUNNING) {
      res->success = false;
      res->message = "already running (" + currentStepName() + ")";
      return;
    }
    const std::string why = safetyReason();
    if (!why.empty()) {
      res->success = false;
      res->message = "cannot start: " + why + " — reset E-stop / fix the error first";
      return;
    }
    start_requested_ = true;
    res->success = true;
    res->message = run_ == Run::PAUSED ? "resume requested" : "start requested";
  }

  void onStop(Trigger::Response::SharedPtr res)
  {
    start_requested_ = false;
    finish(Run::IDLE, "stopped by user");
    res->success = true;
    res->message = "stopped";
  }

  // ------------------------------------------------------------------ tick (10Hz)
  void onTick()
  {
    // 안전: E-stop · 진단 오류면 즉시 PAUSED (자동 재개 없음)
    if (run_ == Run::RUNNING) {
      const std::string why = safetyReason();
      if (!why.empty()) {
        pause(why);
        return;
      }
    }

    if (start_requested_) {
      if (!allServersReady()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "waiting for servers (/drive/*, /manipulator/*, /perception/set_mode%s) before start",
          plan_mode_ ? ", /mission_planner/plan, /perception/check_dice" : "");
      } else {
        start_requested_ = false;
        if (run_ == Run::PAUSED) {
          RCLCPP_INFO(get_logger(), "mission resume at step %zu (%s)", step_idx_, currentStepName().c_str());
          run_ = Run::RUNNING;
          reason_.clear();
          retry_count_.clear();
          enterStep(step_idx_);
        } else {
          beginMission();
        }
        return;
      }
    }
    if (run_ != Run::RUNNING) {return;}

    if (retry_pending_ && now() >= retry_at_) {
      retry_pending_ = false;
      enterStep(retry_idx_);
      return;
    }
    if (retry_pending_) {return;}

    const double limit = stepTimeout();
    if ((now() - step_start_).seconds() > limit) {
      failStep("timeout (" + std::to_string(limit) + "s)");
      return;
    }

    const Step & s = steps_[step_idx_];
    if (s.type == "read_tag") {
      tickReadTag();
    } else if (s.type == "search") {
      tickSearch();
    } else if (s.type == "observe") {
      tickObserve();
    } else if (s.type == "verify") {
      tickVerify();
    } else if (s.type == "survey_view") {
      if (watching_ && (now() - view_since_).seconds() >= survey_settle_) {stepSucceeded();}
    } else if (s.type == "check_marker") {
      tickCheckMarker();
    } else if (s.type == "wait") {
      if (now() >= wait_until_) {stepSucceeded();}
    }
  }

  double stepTimeout() const
  {
    const Step & s = steps_[step_idx_];
    if (s.type == "skill") {return skillTimeout(s) + skill_timeout_margin_;}
    if (s.type == "wait") {return std::stod(s.args[0]) + 5.0;}
    const std::string key = s.type == "goto" ? "navigate" : s.type == "pick_target" ? "pick" :
      (s.type == "survey_start" || s.type == "survey_finish") ? "survey" : s.type;
    auto it = timeout_.find(key);
    return it != timeout_.end() ? it->second : 60.0;
  }

  double skillTimeout(const Step & s) const
  {
    return s.args.size() >= 3 ? std::stod(s.args[2]) : skill_default_timeout_;
  }

  bool allServersReady() const
  {
    // execute_skill 서버는 쓰는 단계에서 확인 (없으면 fallback)
    return nav_client_->action_server_is_ready() && align_client_->action_server_is_ready() &&
           pose_client_->action_server_is_ready() && pick_client_->action_server_is_ready() &&
           mode_client_->service_is_ready() &&
           (!plan_mode_ || (plan_client_->service_is_ready() && dice_client_->service_is_ready()));
  }

  // ------------------------------------------------------------------ 미션 · 단계 전이
  // 공통 시작 단계: 팔을 태그 쪽으로 → 태그 읽기. 읽은 뒤 미션표 단계가 뒤에 붙는다
  void beginMission()
  {
    steps_.clear();
    if (plan_mode_) {
      // 출발 구역에서 신호 · 바코드 관측 → 계획기에 단계 요청 (뒤에 붙음)
      steps_.push_back(parseStep("observe"));
      if (survey_enabled_) {
        survey_round_ = 0;
        steps_.push_back(parseStep("survey_start"));
        for (size_t i = 0; i < survey_boxes_.size(); ++i) {
          appendSurveyBox(steps_, survey_boxes_[i], survey_arm_pose_, survey_approach_);
        }
        steps_.push_back(parseStep("survey_finish"));
      }
      steps_.push_back(parseStep("plan"));
    } else {
      steps_.push_back(parseStep("pose TAG_VIEW"));
      Step read;
      read.type = "read_tag";
      read.text = "read_tag";
      steps_.push_back(read);
    }
    tag_id_ = -1;
    marker_checked_.clear();
    mismatch_logged_cycles_ = -1;
    retry_count_.clear();
    fallback_used_.clear();
    run_ = Run::RUNNING;
    reason_.clear();
    RCLCPP_INFO(get_logger(), "mission start (%s)", plan_mode_ ? "plan" : "tag");
    logEvent("start", "\"source\": \"" + std::string(plan_mode_ ? "plan" : "tag") + "\"");
    enterStep(0);
  }

  std::string currentStepName() const
  {
    return step_idx_ < steps_.size() ? steps_[step_idx_].text : "";
  }

  // 같은 단계 안의 모드 전환 · 다음 동작은 stage_id_로 묶어 늦은 콜백을 무시한다
  void enterStep(size_t idx)
  {
    step_idx_ = idx;
    ++stage_id_;
    step_start_ = now();
    drive_done_ = false;
    arm_done_ = false;
    phase_ = 0;
    watching_ = false;
    rotating_ = false;
    rotations_ = 0;
    const Step & s = steps_[idx];
    RCLCPP_INFO(get_logger(), "-> [%zu] %s", idx, s.text.c_str());
    publishStatus();

    if (s.type == "pose") {
      drive_done_ = true;
      sendArmPose(s.args[0]);
    } else if (s.type == "read_tag") {
      // TAG_VIEW는 앞 단계에서 완료. 태그 모드를 켜고 tickReadTag에서 확인 · 회전
      setModeThen(Mode::READ_TAG, [this]() {beginWatch();});
    } else if (s.type == "navigate") {
      // 주행 + 팔 STOW 병렬 (센서와 무관한 동작)
      const uint64_t id = stage_id_;
      sendNavigate(locations_.at(s.args[0]));
      if (id != stage_id_) {return;}
      sendArmPose("STOW");
    } else if (s.type == "search") {
      // ① STOW 완료 → ② OFF(추적 초기화) → ③ SEARCH_FRONT | SEARCH_REAR → tickSearch
      sendArmPose("STOW");
    } else if (s.type == "approach") {
      // ① STOW 완료 → ② SEARCH_FRONT → ③ align COARSE → ④ READY → ⑤ SEARCH_FRONT | ARM_ALIGN → ⑥ align FINE
      sendArmPose("STOW");
    } else if (s.type == "pick") {
      // 바퀴는 정지 유지. approach FINE 과 같은 모드 (추적 위치 + arm_offset + 태그 → /arm_range)
      drive_done_ = true;
      setModeThen(kFineMode, [this]() {sendPick();});
    } else if (s.type == "goto") {
      const uint64_t id = stage_id_;
      sendNavigate({std::stod(s.args[0]), std::stod(s.args[1]), std::stod(s.args[2])});
      if (id != stage_id_) {return;}
      sendArmPose("STOW");
    } else if (s.type == "pick_target") {
      drive_done_ = true;
      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = map_frame_;
      target.header.stamp = now();
      target.pose.position.x = std::stod(s.args[0]);
      target.pose.position.y = std::stod(s.args[1]);
      target.pose.position.z = std::stod(s.args[2]);
      target.pose.orientation.w = 1.0;
      setModeThen(kFineMode, [this, target]() {sendPick(target);});
    } else if (s.type == "observe" || s.type == "verify") {
      // ① 팔 자세 → ② 모드 → ③ tick 에서 확인 (sequenced)
      drive_done_ = true;
      dice_pending_ = false;
      sendArmPose(s.type == "observe" ? observe_arm_pose_ : verify_arm_pose_);
    } else if (s.type == "plan") {
      requestPlan();
    } else if (s.type == "survey_start") {
      // 이전 미션의 등록표가 남아 있으면 이번 측량이 실패해도 계획기가 그 퍽 배치로 계획하므로 지운다
      std::error_code ec;
      std::filesystem::remove(survey_pucks_out_, ec);
      callTrigger(survey_start_client_, "/perception/survey/start", [this](bool ok, const std::string & msg) {
          if (!ok) {failStep("survey start: " + msg); return;}
          logEvent("survey", "\"event\": \"start\"");
          setModeThen(*parseModeNames(survey_mode_text_), [this]() {stepSucceeded();});
        });
    } else if (s.type == "survey_view") {
      // ① 팔 자세 → ② survey.mode → ③ settle_s 동안 perception_survey 가 관측을 쌓음 (sequenced)
      drive_done_ = true;
      sendArmPose(s.args[1]);
    } else if (s.type == "survey_finish") {
      finishSurvey();
    } else if (s.type == "wait") {
      wait_until_ = now() + rclcpp::Duration::from_seconds(std::stod(s.args[0]));
    } else if (s.type == "check_marker") {
      drive_done_ = true;
      setModeThen(Mode::READ_MARKER, [this]() {beginWatch();});
    } else if (s.type == "skill" && needsMarkerCheck(s)) {
      // 이 장소 첫 놓기 앞에서 한 번 장소 태그 확인 (marker.check_before_place)
      const std::string L = s.args[1].substr(6, 2);
      marker_checked_.insert(L);
      steps_.insert(steps_.begin() + static_cast<std::ptrdiff_t>(idx), parseStep("check_marker " + L));
      enterStep(idx);
      return;
    } else if (s.type == "skill") {
      const bool on_drive = s.args[0] == "drive";
      (on_drive ? arm_done_ : drive_done_) = true;
      const uint8_t mode = s.mode_text.empty() ?
        static_cast<uint8_t>(Mode::SEARCH_FRONT | Mode::ARM_ALIGN) : *parseModeNames(s.mode_text);
      const Step step = s;
      setModeThen(mode, [this, step]() {sendSkill(step);});
    }
  }

  // 단계가 여러 동작을 순서대로 하는 경우 (search · approach): 팔 · 바퀴 결과가 오면 다음 동작
  void advancePhase()
  {
    const Step & s = steps_[step_idx_];
    ++phase_;
    if (s.type == "search") {
      if (phase_ == 1) {
        // STOW 완료 → OFF로 perception 추적 초기화 → 앞뒤 탐색
        setModeThen(Mode::OFF, [this]() {
            setModeThen(Mode::SEARCH_FRONT | Mode::SEARCH_REAR, [this]() {beginWatch();});
          });
      }
    } else if (s.type == "observe" && phase_ == 1) {
      // 팔 자세 완료 → 신호 · 바코드 읽기. 이 시각 뒤에 받은 값만 쓴다
      setModeThen(Mode::READ_SIGNAL | Mode::READ_BARCODE, [this]() {beginWatch();});
    } else if (s.type == "verify" && phase_ == 1) {
      const uint8_t mode = *parseModeNames(verify_mode_text_) | (verify_check_marker_ ? Mode::READ_MARKER : 0);
      setModeThen(mode, [this]() {beginWatch();});
    } else if (s.type == "survey_view" && phase_ == 1) {
      setModeThen(*parseModeNames(survey_mode_text_), [this]() {beginWatch();});
    } else if (s.type == "approach") {
      switch (phase_) {
        case 1:  // STOW 완료 → 전방 탐색만 켜고 먼 거리 정렬 (drive는 ARM_ALIGN 비트가 없으면 먼 거리 모드)
          setModeThen(Mode::SEARCH_FRONT, [this]() {sendAlign(AlignStage::COARSE, coarse_distance_);});
          break;
        case 2:  // COARSE 완료 → 팔 READY
          sendArmPose("READY");
          break;
        case 3:  // READY 완료 → 정밀 정렬 (ARM_ALIGN 비트 → drive 정밀 모드: Arducam 좌우 + 1층 TFmini 박스 거리)
          setModeThen(kFineMode, [this]() {
              sendAlign(AlignStage::FINE, std::stod(steps_[step_idx_].args[0]));
            });
          break;
        default:  // FINE 완료
          stepSucceeded();
          break;
      }
    }
  }

  bool isSequenced() const
  {
    const auto & t = steps_[step_idx_].type;
    return t == "search" || t == "approach" || t == "observe" || t == "verify" || t == "survey_view";
  }

  void stepSucceeded()
  {
    retry_count_.erase(step_idx_);
    if (step_idx_ + 1 < steps_.size()) {
      enterStep(step_idx_ + 1);
      return;
    }
    finish(Run::DONE, "");
  }

  // ------------------------------------------------------------------ 보기 · 제자리 회전 (read_tag · search)
  void beginWatch()
  {
    watching_ = true;
    view_since_ = now();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    found_count_ = 0;
    tag_count_ = 0;
    last_tag_id_ = -1;
  }

  void tickReadTag()
  {
    if (!watching_ || rotating_) {return;}
    int count, id;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      count = tag_count_;
      id = last_tag_id_;
    }
    if (count >= tag_.confirm_count && id >= 0) {
      auto it = missions_.find(id);
      if (it == missions_.end()) {
        finish(Run::FAILED, "tag " + std::to_string(id) + " is not in missions.tag_ids");
        return;
      }
      tag_id_ = id;
      RCLCPP_INFO(get_logger(), "tag %d -> %zu step(s)", id, it->second.size());
      steps_.resize(step_idx_ + 1);
      steps_.insert(steps_.end(), it->second.begin(), it->second.end());
      stepSucceeded();
      return;
    }
    rotateIfNothing(tag_, "read_tag");
  }

  void tickSearch()
  {
    if (!watching_ || rotating_) {return;}
    int count;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      count = found_count_;
    }
    if (count >= search_.confirm_count) {
      // 뒤에서 찾아도 mission은 돌리지 않는다 — COARSE align에서 drive가 돌린다
      RCLCPP_INFO(get_logger(), "search: object found x%d", count);
      stepSucceeded();
      return;
    }
    rotateIfNothing(search_, "search");
  }

  // ------------------------------------------------------------------ plan 모드 단계
  void tickObserve()
  {
    if (!watching_) {return;}
    uint8_t status, cycles;
    std::string seq, code;
    bool sig_new, code_new;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      sig_new = signal_rx_ >= view_since_;
      code_new = barcode_rx_ >= view_since_;
      status = signal_status_;
      seq = signal_seq_;
      cycles = signal_cycles_;
      code = barcode_text_;
    }
    // 신호는 12회 점멸 → 3 s 꺼짐 → 다시 처음부터 반복된다. perception 은 같은 순서 2 묶음이면 CONFIRMED,
    // 묶음끼리 다르면 MISMATCH 지만 다음 묶음이 앞의 것과 같으면 CONFIRMED 로 바뀐다 → MISMATCH 는 바로 멈추지 않고
    // 묶음을 observe.max_cycles 개까지 본 뒤에도 확정이 안 되면 오류 기록 후 안전 정지 (발제: 오류 기록 · 안전 정지)
    if (sig_new && status == SignalSequence::MISMATCH) {
      if (cycles >= observe_max_cycles_) {
        logEvent("observe", "\"ok\": false, \"reason\": \"signal mismatch\", \"cycles\": " + std::to_string(cycles) +
          ", \"sequence\": \"" + seq + "\"");
        finish(Run::FAILED, "signal MISMATCH after " + std::to_string(cycles) + " cycles (" + seq + ")");
        return;
      }
      if (cycles != mismatch_logged_cycles_) {
        mismatch_logged_cycles_ = cycles;
        RCLCPP_WARN(get_logger(), "observe: cycles differ (%d seen, last %s) -> wait for next cycle", cycles, seq.c_str());
        logEvent("observe", "\"ok\": null, \"reason\": \"mismatch, waiting next cycle\", \"cycles\": " +
          std::to_string(cycles) + ", \"sequence\": \"" + seq + "\"");
      }
    }
    if (sig_new && status == SignalSequence::CONFIRMED && code_new) {
      RCLCPP_INFO(get_logger(), "observe: signal %s, barcode %s", seq.c_str(), code.c_str());
      logEvent("observe", "\"ok\": true, \"sequence\": \"" + seq + "\", \"barcode\": \"" + code + "\"");
      stepSucceeded();
    }
  }

  // ------------------------------------------------------------------ 퍽 측량 (#39 #53)
  // 박스 하나 조사: 조사 자세로 주행 (+ 정밀 접근) → 팔 자세로 보기.
  //   arm_pose · approach 는 처음 조사와 다시 보기 (#53: 거리 · 각도를 바꿔 관측) 가 다름
  void appendSurveyBox(std::vector<Step> & out, int64_t box, const std::string & arm_pose, double approach) const
  {
    for (size_t i = 0; i < survey_boxes_.size(); ++i) {
      if (survey_boxes_[i] != box) {continue;}
      std::ostringstream go;
      go << "goto " << survey_poses_[3 * i] << " " << survey_poses_[3 * i + 1] << " " << survey_poses_[3 * i + 2];
      out.push_back(parseStep(go.str()));
      if (approach > 0.0) {
        out.push_back(parseStep("approach " + std::to_string(approach)));
      }
      out.push_back(parseStep("survey_view " + std::to_string(box) + " " + arm_pose));
      return;
    }
  }

  void callTrigger(const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & name, Report cb)
  {
    if (!client || !client->service_is_ready()) {
      cb(false, name + " not ready");
      return;
    }
    const uint64_t id = stage_id_;
    client->async_send_request(std::make_shared<Trigger::Request>(),
      [this, id, cb](rclcpp::Client<Trigger>::SharedFuture future) {
        if (id != stage_id_) {return;}
        const auto res = future.get();
        cb(res->success, res->message);
      });
  }

  // finish 성공 → 완료 파일을 계획기 퍽 등록표로. 실패 → revisit 로 문제 칸만 다시 보기 (#53)
  void finishSurvey()
  {
    callTrigger(survey_finish_client_, "/perception/survey/finish", [this](bool ok, const std::string & msg) {
        if (ok) {
          std::string err;
          if (!writePucksFromSurvey(msg, err)) {
            logEvent("survey", "\"event\": \"finish\", \"ok\": false, \"reason\": \"" + escape(err) + "\"");
            finish(Run::FAILED, "survey: " + err);
            return;
          }
          RCLCPP_INFO(get_logger(), "survey complete: %s -> %s", msg.c_str(), survey_pucks_out_.c_str());
          logEvent("survey", "\"event\": \"finish\", \"ok\": true, \"file\": \"" + escape(msg) +
            "\", \"pucks\": \"" + escape(survey_pucks_out_) + "\"");
          stepSucceeded();
          return;
        }
        RCLCPP_WARN(get_logger(), "survey finish failed: %s", msg.c_str());
        logEvent("survey", "\"event\": \"finish\", \"ok\": false, \"reason\": \"" + escape(msg) + "\"");
        if (survey_round_ >= survey_max_rounds_) {
          surveyUnconfirmed("revisit rounds used up (" + std::to_string(survey_max_rounds_) + "): " + msg);
          return;
        }
        callTrigger(survey_revisit_client_, "/perception/survey/revisit", [this](bool rok, const std::string & rmsg) {
            onSurveyRevisit(rok, rmsg);
          });
      });
  }

  // revisit 응답 message = {"revisit": [{"slot": "box:side", ...}], "unconfirmed": [...]}
  void onSurveyRevisit(bool ok, const std::string & msg)
  {
    std::set<int64_t> boxes;
    std::string unconfirmed;
    try {
      const YAML::Node doc = YAML::Load(msg);
      for (const auto & r : doc["revisit"]) {
        const std::string slot = r["slot"].as<std::string>();
        boxes.insert(std::stoll(slot.substr(0, slot.find(':'))));
      }
      for (const auto & u : doc["unconfirmed"]) {unconfirmed += (unconfirmed.empty() ? "" : ", ") + u.as<std::string>();}
    } catch (const std::exception &) {
      if (ok) {
        finish(Run::FAILED, "survey revisit: cannot read response: " + msg);
        return;
      }
    }
    logEvent("survey", "\"event\": \"revisit\", \"ok\": " + std::string(ok ? "true" : "false") +
      ", \"message\": \"" + escape(msg) + "\"");
    if (!ok || boxes.empty()) {
      surveyUnconfirmed(unconfirmed.empty() ? msg : "unconfirmed " + unconfirmed);
      return;
    }
    ++survey_round_;
    std::vector<Step> added;
    for (int64_t b : boxes) {appendSurveyBox(added, b, survey_revisit_arm_pose_, survey_revisit_approach_);}
    added.push_back(parseStep("survey_finish"));
    RCLCPP_INFO(get_logger(), "survey revisit round %d: %zu box(es)", survey_round_, boxes.size());
    steps_.insert(steps_.begin() + static_cast<std::ptrdiff_t>(step_idx_) + 1, added.begin(), added.end());
    stepSucceeded();
  }

  // 미확인 칸이 남음: fail = 안전 정지 (할당에는 12칸이 다 필요), registered = 계획기 등록표 그대로 계획
  void surveyUnconfirmed(const std::string & why)
  {
    if (survey_on_unconfirmed_ == "registered") {
      RCLCPP_WARN(get_logger(), "survey incomplete (%s) -> plan with registered pucks_file", why.c_str());
      logEvent("survey", "\"event\": \"incomplete\", \"use\": \"registered\", \"reason\": \"" + escape(why) + "\"");
      stepSucceeded();
      return;
    }
    finish(Run::FAILED, "survey incomplete: " + why);
  }

  // perception_survey 완료 파일 (JSON) → mission_planner 퍽 등록표 yaml (pucks: [{id, color, shape, box, side}])
  bool writePucksFromSurvey(const std::string & path, std::string & err)
  {
    std::ostringstream out;
    try {
      const YAML::Node doc = YAML::LoadFile(path);
      const YAML::Node slots = doc["slots"];
      if (!slots || slots.size() != 12) {err = "complete file must have 12 slots: " + path; return false;}
      out << "# mission_node 가 perception_survey 완료 파일 (" << path << ") 에서 만든 퍽 등록표\npucks:\n";
      for (const auto & t : slots) {
        if (!t["id"] || t["id"].IsNull() || !t["color"] || t["color"].IsNull() || !t["shape"] || t["shape"].IsNull()) {
          err = "slot without id · color · shape in " + path;
          return false;
        }
        const std::string shape = t["shape"].as<std::string>() == "cuboid" ? "box" : t["shape"].as<std::string>();
        out << "  - {id: " << t["id"].as<int>() << ", color: " << t["color"].as<std::string>() << ", shape: " << shape
            << ", box: " << t["box"].as<int>() << ", side: " << t["side"].as<int>() << "}\n";
      }
    } catch (const std::exception & e) {
      err = std::string("cannot read ") + path + ": " + e.what();
      return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(survey_pucks_out_).parent_path(), ec);
    std::ofstream f(survey_pucks_out_);
    if (!f || !(f << out.str())) {err = "cannot write " + survey_pucks_out_; return false;}
    return true;
  }

  // ------------------------------------------------------------------ 장소 태그 (#39)
  // "L1" → marker.ids[0] (없으면 -1)
  int markerIdOf(const std::string & place) const
  {
    if (place.size() != 2 || place[0] != 'L' || place[1] < '1' || place[1] > '3') {return -1;}
    return static_cast<int>(marker_ids_[place[1] - '1']);
  }

  // place_<L>_* skill 이고 그 장소를 아직 확인하지 않았으면 true
  bool needsMarkerCheck(const Step & s) const
  {
    if (!plan_mode_ || !marker_check_before_place_ || s.args.size() < 2 || s.args[0] != "manipulator") {return false;}
    const std::string & name = s.args[1];
    if (name.rfind("place_L", 0) != 0 || name.size() < 8) {return false;}
    return marker_checked_.count(name.substr(6, 2)) == 0;
  }

  // 태그 기록 JSON: 기대 ID · 본 ID · 일치 · base_link 와 지도 위치 (새 관측). bad = 못 봤거나 ID 가 다름
  std::string markerJson(const TaskMarker::ConstSharedPtr & m, int expect, bool & bad)
  {
    std::ostringstream o;
    o << std::fixed << std::setprecision(4) << "{\"expect\": " << expect;
    if (!m) {
      bad = true;
      o << ", \"seen\": false}";
      return o.str();
    }
    bad = m->marker_id != expect;
    o << ", \"seen\": true, \"id\": " << m->marker_id << ", \"match\": " << (bad ? "false" : "true")
      << ", \"distance\": " << m->distance << ", \"camera\": " << int(m->camera)
      << ", \"base\": [" << m->pose.position.x << ", " << m->pose.position.y << ", " << m->pose.position.z << "]";
    try {
      geometry_msgs::msg::PoseStamped in, out;
      in.header = m->header;
      in.pose = m->pose;
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(map_frame_, m->header.frame_id, tf2_ros::fromMsg(m->header.stamp));
      } catch (const tf2::TransformException &) {
        tf = tf_buffer_->lookupTransform(map_frame_, m->header.frame_id, tf2::TimePointZero);
      }
      tf2::doTransform(in, out, tf);
      o << ", \"map\": [" << out.pose.position.x << ", " << out.pose.position.y << ", " << out.pose.position.z << "]";
    } catch (const tf2::TransformException & e) {
      o << ", \"map_error\": \"" << escape(e.what()) << "\"";
    }
    o << "}";
    return o.str();
  }

  // 놓기 전 장소 확인: 기대 ID 를 보면 성공, 다른 장소 태그면 실패 (timeout.check_marker 안에 못 보면 실패 → 재시도)
  void tickCheckMarker()
  {
    if (!watching_) {return;}
    TaskMarker::ConstSharedPtr m;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (last_marker_ && marker_rx_ >= view_since_) {m = last_marker_;}
    }
    if (!m) {return;}
    const std::string L = steps_[step_idx_].args[0];
    bool bad = false;
    const std::string j = markerJson(m, markerIdOf(L), bad);
    logEvent("check_marker", "\"place\": \"" + L + "\", \"marker\": " + j);
    if (bad) {
      failStep("place " + L + ": saw marker " + std::to_string(m->marker_id) + ", expected " +
        std::to_string(markerIdOf(L)));
      return;
    }
    stepSucceeded();
  }

  void requestPlan()
  {
    if (!plan_client_->service_is_ready()) {failStep(plan_service_ + " not ready"); return;}
    const uint64_t id = stage_id_;
    plan_client_->async_send_request(std::make_shared<Trigger::Request>(),
      [this, id](rclcpp::Client<Trigger>::SharedFuture future) {
        if (id != stage_id_) {return;}
        const auto res = future.get();
        if (!res->success) {
          logEvent("plan", "\"ok\": false, \"reason\": \"" + escape(res->message) + "\"");
          finish(Run::FAILED, "plan: " + res->message);   // 풀 수 없는 문제 · 미등록 바코드: 재시도 없음
          return;
        }
        std::vector<Step> added;
        std::istringstream in(res->message);
        std::string line;
        try {
          while (std::getline(in, line)) {
            if (line.find_first_not_of(" \t\r") == std::string::npos) {continue;}
            Step st = parseStep(line);
            checkStep(st, "plan");
            added.push_back(st);
          }
        } catch (const std::invalid_argument & e) {
          finish(Run::FAILED, std::string("plan step rejected: ") + e.what());
          return;
        }
        if (added.empty()) {finish(Run::FAILED, "plan returned no steps"); return;}
        RCLCPP_INFO(get_logger(), "plan -> %zu step(s)", added.size());
        logEvent("plan", "\"ok\": true, \"steps\": " + std::to_string(added.size()));
        steps_.resize(step_idx_ + 1);
        steps_.insert(steps_.end(), added.begin(), added.end());
        stepSucceeded();
      });
  }

  void tickVerify()
  {
    if (!watching_ || dice_pending_ || (now() - view_since_).seconds() < verify_settle_) {return;}
    const Step & s = steps_[step_idx_];
    // 장소 태그: settle 뒤 marker_wait_s 까지 새로 본 태그를 기다림 (없어도 칸 판정은 함)
    std::string marker_json;
    bool marker_bad = false;
    if (verify_check_marker_) {
      TaskMarker::ConstSharedPtr m;
      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        if (last_marker_ && marker_rx_ >= view_since_) {m = last_marker_;}
      }
      if (!m && (now() - view_since_).seconds() < verify_settle_ + verify_marker_wait_) {return;}
      marker_json = markerJson(m, markerIdOf(s.args[0]), marker_bad);
    }
    auto req = std::make_shared<CheckDice::Request>();
    req->header.frame_id = map_frame_;
    const double cx = std::stod(s.args[1]), cy = std::stod(s.args[2]), z = std::stod(s.args[3]);
    for (int k = 0; k < 9; ++k) {
      const int r = k / 3, c = k % 3;          // 위 줄 (+Y) 부터, 줄 안은 -X 부터
      req->cells[k].x = cx + (c - 1) * 0.06;
      req->cells[k].y = cy + (1 - r) * 0.06;
      req->cells[k].z = z;
      req->expect[k] = static_cast<uint8_t>(s.args[4][k] - '0');
    }
    req->yellow_shape = static_cast<uint8_t>(std::stoi(s.args[5]));
    req->tolerance = static_cast<float>(verify_tolerance_);
    if (!dice_client_->service_is_ready()) {failStep("/perception/check_dice not ready"); return;}
    dice_pending_ = true;
    const uint64_t id = stage_id_;
    const std::string place = s.args[0];
    dice_client_->async_send_request(req,
      [this, id, place, marker_json, marker_bad](rclcpp::Client<CheckDice>::SharedFuture future) {
        if (id != stage_id_) {return;}
        const auto r = future.get();
        std::ostringstream o;
        o << "\"place\": \"" << place << "\", \"observed\": " << (r->observed ? "true" : "false")
          << ", \"passed\": " << (r->passed ? "true" : "false") << ", \"message\": \"" << escape(r->message) << "\", \"cells\": [";
        for (int k = 0; k < 9; ++k) {
          o << (k ? ", " : "") << "{\"error\": " << int(r->error[k]) << ", \"count\": " << int(r->count[k])
            << ", \"offset\": " << r->offset[k] << ", \"color\": " << int(r->color[k]) << ", \"shape\": " << int(r->shape[k]) << "}";
        }
        o << "]";
        if (!marker_json.empty()) {o << ", \"marker\": " << marker_json;}
        logEvent("verify", o.str());
        RCLCPP_INFO(get_logger(), "verify %s: observed %d, passed %d (%s)", place.c_str(), r->observed, r->passed, r->message.c_str());
        if ((!r->observed || !r->passed) && verify_stop_on_fail_) {
          failStep("verify " + place + ": " + (r->observed ? r->message : "nothing observed"));
          return;
        }
        if (marker_bad && verify_stop_on_fail_) {
          failStep("verify " + place + ": place marker not seen or wrong id");
          return;
        }
        stepSucceeded();
      });
  }

  static std::string escape(const std::string & in)
  {
    std::string out;
    for (char c : in) {
      if (c == '"' || c == '\\') {out += '\\';}
      if (c == '\n') {out += "\\n"; continue;}
      out += c;
    }
    return out;
  }

  // log_dir/mission_log.jsonl 에 한 줄 (관측 · 계획 · 재검증 기록 = 발제 제출물 근거)
  void logEvent(const std::string & kind, const std::string & body)
  {
    if (log_dir_.empty()) {return;}
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    std::ofstream f(log_dir_ + "/mission_log.jsonl", std::ios::app);
    if (!f) {RCLCPP_WARN_ONCE(get_logger(), "cannot write %s/mission_log.jsonl", log_dir_.c_str()); return;}
    f << std::fixed << std::setprecision(3) << "{\"t\": " << now().seconds() << ", \"kind\": \"" << kind << "\", \"step\": "
      << step_idx_ << ", " << body << "}\n";
  }

  // view_wait 동안 못 찾으면 제자리 회전: 현재 위치 + yaw로 /drive/navigate (drive가 nav2 spin으로 처리)
  void rotateIfNothing(const Rotate & r, const std::string & what)
  {
    if ((now() - view_since_).seconds() < r.view_wait) {return;}
    if (rotations_ >= r.max_rotations) {
      failStep("nothing found after " + std::to_string(rotations_) + " rotation(s)");
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      failStep(std::string("TF ") + map_frame_ + " -> " + base_frame_ + ": " + e.what());
      return;
    }
    const double yaw = tf2::getYaw(tf.transform.rotation) + r.rotate_step;
    ++rotations_;
    rotating_ = true;
    RCLCPP_INFO(get_logger(), "%s: nothing -> rotate %d/%d", what.c_str(), rotations_, r.max_rotations);
    const uint64_t id = stage_id_;
    sendNavigate({tf.transform.translation.x, tf.transform.translation.y, yaw},
      [this, id, what](bool ok, const std::string & msg) {
        if (id != stage_id_) {return;}
        if (!ok) {failStep(what + " rotate: " + msg); return;}
        rotating_ = false;
        beginWatch();
      });
  }

  // ------------------------------------------------------------------ 결과 처리
  void onDriveResult(uint64_t id, bool ok, const std::string & msg)
  {
    if (id != stage_id_) {return;}
    if (!ok) {failStep("drive: " + msg); return;}
    if (isSequenced()) {advancePhase(); return;}
    drive_done_ = true;
    if (drive_done_ && arm_done_) {stepSucceeded();}
  }

  void onArmResult(uint64_t id, bool ok, const std::string & msg)
  {
    if (id != stage_id_) {return;}
    if (!ok) {failStep("manipulator: " + msg); return;}
    if (isSequenced()) {advancePhase(); return;}
    arm_done_ = true;
    if (drive_done_ && arm_done_) {stepSucceeded();}
  }

  // 실패 · 타임아웃: 진행 중 goal 전부 cancel → fallback · 재시도 · FAILED
  void failStep(const std::string & reason)
  {
    const size_t failed = step_idx_;
    Step & s = steps_[failed];
    RCLCPP_WARN(get_logger(), "[%zu] %s failed: %s", failed, s.text.c_str(), reason.c_str());
    cancelAll();
    ++stage_id_;  // 늦게 오는 결과 무시
    watching_ = false;

    // skill 실패 → fallback 단계로 대체 (한 번만)
    if (s.type == "skill" && !s.fallback.empty() && !fallback_used_[failed]) {
      fallback_used_[failed] = true;
      RCLCPP_WARN(get_logger(), "[%zu] fallback -> %s", failed, s.fallback.c_str());
      s = parseStep(s.fallback);
      scheduleRetry(failed);
      return;
    }
    int & count = retry_count_[failed];
    if (count >= max_retries_) {
      finish(Run::FAILED, s.text + " failed after " + std::to_string(count) + " retries: " + reason);
      return;
    }
    ++count;
    size_t target = failed;
    if (s.type == "pick") {
      // pick 실패(IK 해 없음 · 빈손 등)는 직전 search부터: search가 STOW · 추적 초기화부터 다시 한다
      for (size_t i = failed; i-- > 0;) {
        if (steps_[i].type == "search") {target = i; break;}
      }
    }
    RCLCPP_INFO(get_logger(), "retry %d/%d -> [%zu] %s in %.1fs", count, max_retries_, target,
      steps_[target].text.c_str(), retry_delay_);
    scheduleRetry(target);
  }

  void scheduleRetry(size_t idx)
  {
    retry_idx_ = idx;
    retry_at_ = now() + rclcpp::Duration::from_seconds(retry_delay_);
    retry_pending_ = true;
  }

  // E-stop · 진단 오류: goal 취소 후 PAUSED. 현재 단계는 /mission/start 때 처음부터 다시
  void pause(const std::string & why)
  {
    RCLCPP_ERROR(get_logger(), "PAUSED at [%zu] %s: %s", step_idx_, currentStepName().c_str(), why.c_str());
    cancelAll();
    ++stage_id_;
    watching_ = false;
    if (retry_pending_) {
      step_idx_ = retry_idx_;  // 재시도 대기 중이었으면 그 단계부터 재개
      retry_pending_ = false;
    }
    run_ = Run::PAUSED;
    reason_ = why;
    publishStatus();
  }

  // 끝 (DONE · FAILED · IDLE): goal 취소, set_mode(OFF) (카메라 정지 · 추적 삭제)
  void finish(Run r, const std::string & why)
  {
    if (run_ == Run::IDLE && r == Run::IDLE && !retry_pending_) {return;}
    if (r == Run::FAILED) {
      RCLCPP_ERROR(get_logger(), "-> FAILED: %s", why.c_str());
    } else if (r == Run::IDLE) {
      RCLCPP_WARN(get_logger(), "-> IDLE: %s", why.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "mission DONE");
    }
    cancelAll();
    retry_pending_ = false;
    watching_ = false;
    if (r != Run::IDLE || run_ != Run::IDLE) {
      logEvent("finish", "\"state\": \"" + std::string(r == Run::DONE ? "DONE" : r == Run::FAILED ? "FAILED" : "IDLE") +
        "\", \"reason\": \"" + escape(why) + "\"");
    }
    run_ = r;
    reason_ = why;
    ++stage_id_;
    publishStatus();
    setMode(Mode::OFF, [this](bool ok, const std::string & msg) {
        if (!ok) {RCLCPP_WARN(get_logger(), "set_mode(OFF) failed: %s", msg.c_str());}
      });
  }

  // 진행 중 goal 전부 취소. 하위 노드는 새 goal이 오면 이전 goal을 대체하는 계약이라
  // 재시도 전에 취소 완료를 기다리지 않는다 (실제 안전 정지는 /e_stop 경로가 담당)
  void cancelAll()
  {
    cancelGoal<NavigateToPose>(nav_client_, nav_goal_, "/drive/navigate");
    cancelGoal<AlignBase>(align_client_, align_goal_, "/drive/align");
    cancelGoal<ExecuteSkill>(drive_skill_client_, drive_skill_goal_, "/drive/execute_skill");
    cancelGoal<ArmPose>(pose_client_, pose_goal_, "/manipulator/pose");
    cancelGoal<Pick>(pick_client_, pick_goal_, "/manipulator/pick");
    cancelGoal<ExecuteSkill>(arm_skill_client_, arm_skill_goal_, "/manipulator/execute_skill");
  }

  // 이미 끝난 goal(UNKNOWN_GOAL_ID · GOAL_TERMINATED)에 대한 취소는 정상
  template<typename ActionT>
  void cancelGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & gh, const std::string & label)
  {
    if (!gh) {return;}
    client->async_cancel_goal(
      gh, [this, label](typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr res) {
        using CancelGoal = action_msgs::srv::CancelGoal::Response;
        if (!res || res->return_code == CancelGoal::ERROR_REJECTED) {
          RCLCPP_ERROR(get_logger(), "%s rejected cancel request", label.c_str());
        }
      });
    gh.reset();
  }

  // ------------------------------------------------------------------ status
  void publishStatus()
  {
    MissionStatus st;
    st.stamp = now();
    st.tag_id = tag_id_;
    st.step_index = static_cast<uint16_t>(step_idx_);
    st.step_name = run_ == Run::IDLE ? "" : currentStepName();
    st.state = runToMsg(run_);
    st.reason = reason_;
    status_pub_->publish(st);
  }

  // ------------------------------------------------------------------ 요청 보내기
  void setMode(uint8_t mode, Report cb)
  {
    if (!mode_client_->service_is_ready()) {
      cb(false, "/perception/set_mode not ready");
      return;
    }
    auto req = std::make_shared<SetPerceptionMode::Request>();
    req->mode = mode;
    mode_client_->async_send_request(
      req, [cb](rclcpp::Client<SetPerceptionMode>::SharedFuture future) {
        const auto res = future.get();
        cb(res->success, res->message);
      });
  }

  // 현재 단계에서 모드를 바꾸고 성공하면 next 실행 (단계가 바뀌었으면 무시, 실패면 failStep)
  void setModeThen(uint8_t mode, std::function<void()> next)
  {
    const uint64_t id = stage_id_;
    setMode(mode, [this, id, mode, next](bool ok, const std::string & msg) {
        if (id != stage_id_) {return;}
        if (!ok) {failStep("set_mode(" + std::to_string(mode) + "): " + msg); return;}
        next();
      });
  }

  // 공통: goal 응답 · 결과 콜백 연결. 지난 단계의 goal은 응답이 오는 즉시 취소
  template<typename ActionT, typename ResultFn>
  void sendGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & handle_slot,
    const std::string & label, Report report, ResultFn result_ok)
  {
    const uint64_t id = stage_id_;
    if (!client->action_server_is_ready()) {
      report(false, label + " server not ready");
      return;
    }
    typename rclcpp_action::Client<ActionT>::SendGoalOptions opt;
    opt.goal_response_callback =
      [this, id, client, &handle_slot, report, label](
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr gh) {
        if (id != stage_id_) {
          if (gh) {client->async_cancel_goal(gh);}
          return;
        }
        if (!gh) {
          report(false, label + " goal rejected");
          return;
        }
        handle_slot = gh;
      };
    opt.result_callback =
      [this, id, &handle_slot, report, label, result_ok](
      const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wr) {
        if (id != stage_id_) {return;}
        handle_slot.reset();
        std::string msg;
        const bool ok = wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
          result_ok(*wr.result, msg);
        if (!ok && msg.empty()) {msg = label + " did not succeed";}
        report(ok, msg);
      };
    client->async_send_goal(goal, opt);
  }

  Report driveReport()
  {
    const uint64_t id = stage_id_;
    return [this, id](bool ok, const std::string & m) {onDriveResult(id, ok, m);};
  }

  Report armReport()
  {
    const uint64_t id = stage_id_;
    return [this, id](bool ok, const std::string & m) {onArmResult(id, ok, m);};
  }

  void sendNavigate(const std::vector<double> & xyyaw, Report report = nullptr)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = map_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = xyyaw[0];
    goal.pose.pose.position.y = xyyaw[1];
    goal.pose.pose.orientation.z = std::sin(xyyaw[2] / 2.0);
    goal.pose.pose.orientation.w = std::cos(xyyaw[2] / 2.0);
    goal.behavior_tree = behavior_tree_;
    sendGoal<NavigateToPose>(nav_client_, goal, nav_goal_, "/drive/navigate",
      report ? report : driveReport(),
      [](const NavigateToPose::Result &, std::string &) {return true;});
  }

  // COARSE(먼 거리, 범퍼 기준) · FINE(정밀, TFmini 기준 >= 0.10m).
  // AlignBase에는 단계 필드가 없다 — drive가 goal 뒤 첫 result의 ARM_ALIGN 비트로 정하므로 모드를 먼저 켜고 보낸다
  void sendAlign(AlignStage stage, double distance)
  {
    AlignBase::Goal goal;
    goal.target_distance = static_cast<float>(distance);
    goal.tolerance = static_cast<float>(align_tolerance_);
    const std::string label = stage == AlignStage::COARSE ? "/drive/align(COARSE)" : "/drive/align(FINE)";
    sendGoal<AlignBase>(align_client_, goal, align_goal_, label, driveReport(),
      [](const AlignBase::Result & r, std::string & msg) {msg = r.message; return r.success;});
  }

  void sendArmPose(const std::string & pose_name)
  {
    ArmPose::Goal goal;
    goal.name = pose_name;
    sendGoal<ArmPose>(pose_client_, goal, pose_goal_, "/manipulator/pose(" + pose_name + ")",
      armReport(), [](const ArmPose::Result & r, std::string &) {return r.success;});
  }

  void sendPick(const std::optional<geometry_msgs::msg::PoseStamped> & target = std::nullopt)
  {
    Pick::Goal goal;  // target 비움 → manipulator가 정밀 정렬 결과 → 추적 위치 순서로 정한다
    if (target) {goal.target = *target;}   // pick_target: 계획기가 아는 퍽 자리 (지도 좌표)
    const bool need_grasp = require_grasped_;
    sendGoal<Pick>(pick_client_, goal, pick_goal_, "/manipulator/pick", armReport(),
      [need_grasp](const Pick::Result & r, std::string & msg) {
        msg = r.message;
        if (r.success && need_grasp && !r.grasped) {
          msg = "pick finished but nothing grasped" + (r.message.empty() ? "" : " (" + r.message + ")");
          return false;
        }
        return r.success;
      });
  }

  void sendSkill(const Step & s)
  {
    ExecuteSkill::Goal goal;
    goal.policy = s.args[1];
    goal.timeout = static_cast<float>(skillTimeout(s));
    auto ok_fn = [](const ExecuteSkill::Result & r, std::string & msg) {msg = r.message; return r.success;};
    if (s.args[0] == "drive") {
      sendGoal<ExecuteSkill>(drive_skill_client_, goal, drive_skill_goal_, "/drive/execute_skill",
        driveReport(), ok_fn);
    } else {
      sendGoal<ExecuteSkill>(arm_skill_client_, goal, arm_skill_goal_, "/manipulator/execute_skill",
        armReport(), ok_fn);
    }
  }

  // ------------------------------------------------------------------ 멤버
  // 파라미터
  std::string map_frame_, base_frame_, behavior_tree_;
  std::map<std::string, std::vector<double>> locations_;
  std::map<int, std::vector<Step>> missions_;
  std::map<std::string, double> timeout_;
  double skill_default_timeout_{30.0}, skill_timeout_margin_{5.0};
  int max_retries_{2};
  double retry_delay_{1.0};
  double stale_timeout_{0.3};
  Rotate tag_, search_;
  double align_tolerance_{0.0}, coarse_distance_{0.45};
  bool require_grasped_{true};
  std::string diag_prefix_{"opencr/"};
  double diag_timeout_{3.0};
  bool plan_mode_{false};
  std::string plan_service_, observe_arm_pose_, verify_arm_pose_, verify_mode_text_, log_dir_;
  double verify_settle_{2.0}, verify_tolerance_{0.0};
  bool verify_stop_on_fail_{false};
  std::vector<int64_t> marker_ids_;
  bool verify_check_marker_{true}, marker_check_before_place_{false};
  double verify_marker_wait_{3.0};
  std::set<std::string> marker_checked_;   // 놓기 전 태그를 확인한 장소 (미션마다 초기화)
  bool survey_enabled_{false};
  std::vector<int64_t> survey_boxes_;
  std::vector<double> survey_poses_;
  std::vector<std::string> survey_sides_;
  std::string survey_poses_file_;
  double survey_approach_{0.13}, survey_revisit_approach_{0.20}, survey_settle_{2.0};
  int survey_max_rounds_{2}, survey_round_{0};
  std::string survey_arm_pose_, survey_revisit_arm_pose_, survey_mode_text_, survey_on_unconfirmed_, survey_pucks_out_;

  // ROS 인터페이스
  rclcpp::CallbackGroup::SharedPtr mission_cb_group_, sensor_cb_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr result_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<SignalSequence>::SharedPtr signal_sub_;
  rclcpp::Subscription<Barcode>::SharedPtr barcode_sub_;
  rclcpp::Subscription<TaskMarker>::SharedPtr marker_sub_;
  rclcpp::Client<Trigger>::SharedPtr plan_client_;
  rclcpp::Client<Trigger>::SharedPtr survey_start_client_, survey_finish_client_, survey_revisit_client_;
  rclcpp::Client<CheckDice>::SharedPtr dice_client_;
  rclcpp::Client<SetPerceptionMode>::SharedPtr mode_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<AlignBase>::SharedPtr align_client_;
  rclcpp_action::Client<ExecuteSkill>::SharedPtr drive_skill_client_, arm_skill_client_;
  rclcpp_action::Client<ArmPose>::SharedPtr pose_client_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, status_timer_;

  // 진행 중 goal (cancel용)
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_;
  rclcpp_action::ClientGoalHandle<AlignBase>::SharedPtr align_goal_;
  rclcpp_action::ClientGoalHandle<ExecuteSkill>::SharedPtr drive_skill_goal_, arm_skill_goal_;
  rclcpp_action::ClientGoalHandle<ArmPose>::SharedPtr pose_goal_;
  rclcpp_action::ClientGoalHandle<Pick>::SharedPtr pick_goal_;

  // 센서 (sensor_cb_group_ 쓰기, mission_cb_group_ 읽기)
  std::mutex sensor_mutex_;
  int found_count_{0}, tag_count_{0}, last_tag_id_{-1};
  std::string diag_error_;
  rclcpp::Time diag_rx_{0, 0, RCL_ROS_TIME};
  uint8_t signal_status_{0}, signal_cycles_{0};
  int observe_max_cycles_{4}, mismatch_logged_cycles_{-1};
  std::string signal_seq_, barcode_text_;
  rclcpp::Time signal_rx_{0, 0, RCL_ROS_TIME}, barcode_rx_{0, 0, RCL_ROS_TIME};
  TaskMarker::ConstSharedPtr last_marker_;
  rclcpp::Time marker_rx_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> estop_active_{false};

  // 상태 (mission_cb_group_ 전용)
  std::atomic<bool> start_requested_{false};
  Run run_{Run::IDLE};
  std::string reason_;
  std::vector<Step> steps_;
  size_t step_idx_{0};
  int tag_id_{-1};
  uint64_t stage_id_{0};
  rclcpp::Time step_start_{0, 0, RCL_ROS_TIME};
  bool drive_done_{false}, arm_done_{false};
  int phase_{0};                    // search · approach 안의 진행 단계
  bool watching_{false};            // read_tag · search: 모드가 켜져서 결과를 보는 중
  bool rotating_{false};            // 제자리 회전 중
  int rotations_{0};
  rclcpp::Time view_since_{0, 0, RCL_ROS_TIME};
  std::map<size_t, int> retry_count_;
  std::map<size_t, bool> fallback_used_;
  bool retry_pending_{false};
  size_t retry_idx_{0};
  rclcpp::Time retry_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wait_until_{0, 0, RCL_ROS_TIME};
  bool dice_pending_{false};
};

}  // namespace mission

RCLCPP_COMPONENTS_REGISTER_NODE(mission::MissionNode)
