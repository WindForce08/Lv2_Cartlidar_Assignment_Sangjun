// ManipulatorNode — 팔 · 그리퍼 제어 (Controller 팀)
// 명세: docs/nodes/manipulator_node.md
//
// 통신 규약: docs/interfaces.md
// - Action 서버 /manipulator/pose, /manipulator/pick, /manipulator/execute_skill (← mission_node)
// - Action 클라이언트 /arm_controller/follow_joint_trajectory, /gripper_controller/gripper_cmd
// - Topic 발행 /arm_controller/joint_trajectory (강화학습 정책 스트리밍)
// - 팔 · 그리퍼 명령은 이 노드만 보낸다. 작업은 한 번에 하나: 새 goal이 오면 이전 goal은 abort
// - 인식 값은 header.stamp(ROS 시계 촬영 시각) 기준 stale_timeout(300ms)보다 오래되면 버린다 (예외 없음)
// - 물체 위치는 perception 결과(object_found · object_pose)만 쓴다. 이 노드는 따로 기억하지 않는다
// - robot_interfaces는 팀 공용 정의를 그대로 쓴다 (이 노드 때문에 바꾸지 않는다)
// - pick 목표 우선순위: ① 정밀 정렬 결과 (arm_offset · /arm_range → 팔 끝 축 위 지점) ② 추적 위치
// - /arm_range: perception_node 가 D435 · Arducam 측위로 낸 "팔 끝(range_frame = tool_link) → 정렬 중인 퍽 표면" 거리.
//   미세 보정의 전후 성분 · 최종 전진 거리에 쓴다. TFmini(1층, 박스 앞면 거리)는 drive 정렬 전용이라 여기서 쓰지 않는다
// - align 진행 중에는 팔을 움직이지 않는다 — 순서는 mission이 보장
// - 그리퍼 (gripper.mode=arm_joint, 실물): 6 번째 칸 gripper_roll = 그리퍼 개폐 모터 (열림 -0.764 · 닫힘 +0.923 rad).
//   arm_controller 궤적의 6 번째 값으로 여닫고, 모든 팔 궤적에 지금 그리퍼 명령을 같이 실어 쥔 퍽을 놓치지 않는다.
//   펌웨어 GripGuard 가 막히면 약하게 쥔다 → 닫은 뒤 위치가 닫힘 명령보다 grasp_margin 넘게 덜 닫혔으면 잡음
// - 주사위 미션 skill (/manipulator/execute_skill 의 이름, mission_planner mission_lines.py · 이슈 #63 #74). 같은 이름의
//   정책이 로드돼 있으면 정책을, 없으면 아래 고정 동작을 쓴다 (강화학습은 IK 대체 경로를 항상 유지)
//     toppick_tilt                   박스 위 퍽을 집게 45° 로 집기 (pick 과 같은 과정, 기울기만 toppick.pitch)
//     pallet_load_<열>_<칸>_tilt      쥔 퍽을 팔레트 칸에 놓기 = pallet_place_traj.json 의 칸 궤적 (3 열 쥔 구간 0.4 rad/s, #75)
//     pallet_unload_<열>_<칸>_tilt    팔레트 칸의 퍽을 집어 들기 = 같은 궤적, 그리퍼만 반대 (0.8 rad/s)
//     place_<L>_<dx>_<dy>_tilt        쥔 퍽을 장소 L 의 3×3 칸 (dx, dy = m1 · 0 · p1) 에 45° 로 놓기.
//                                    칸 좌표 = mission_planner config/approach_poses.yaml cells (지도) → TF 로 팔 기준
//     place_at:<frame>:<x>:<y>:<z>   쥔 퍽을 칸 좌표 (frame 기준 x, y, 윗면 높이 z) 에 놓기 — perception place_box 칸 (#74)
// - 결과를 콜백 안에서 기다리지 않는다. 각 단계는 이전 단계의 result 콜백에서 이어진다
//
// callback group
// - job_cb_group_: Action 서버 · 클라이언트 · tick 타이머. 작업 상태는 이 그룹에서만 바뀐다 (락 불필요)
// - sensor_cb_group_: /perception/result · /arm_range · /joint_states 구독. sensor_mutex_로 캐시 보호

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "action_msgs/srv/cancel_goal.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "control/arm_ik.hpp"
#include "robot_interfaces/action/arm_pose.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/action/pick.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "robot_policy/policy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "yaml-cpp/yaml.h"

namespace control
{

class ManipulatorNode : public rclcpp::Node
{
public:
  using ArmPose = robot_interfaces::action::ArmPose;
  using Pick = robot_interfaces::action::Pick;
  using ExecuteSkill = robot_interfaces::action::ExecuteSkill;
  using SkillGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSkill>;
  using PoseGoalHandle = rclcpp_action::ServerGoalHandle<ArmPose>;
  using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GripperCmd = control_msgs::action::ParallelGripperCommand;
  using PerceptionResult = robot_interfaces::msg::PerceptionResult;
  using SetPerceptionMode = robot_interfaces::srv::SetPerceptionMode;
  using Mode = SetPerceptionMode::Request;
  using Done = std::function<void (bool ok, const std::string & message)>;

  // Pick feedback.stage
  static constexpr uint8_t STAGE_PLAN = 0;
  static constexpr uint8_t STAGE_APPROACH = 1;
  static constexpr uint8_t STAGE_GRASP = 2;
  static constexpr uint8_t STAGE_LIFT = 3;

  explicit ManipulatorNode(const rclcpp::NodeOptions & options): rclcpp::Node("manipulator_node", options)
  {
    loadParameters();

    job_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(get_parameter("tf_cache_time").as_double()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- 구독 ---
    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = sensor_cb_group_;
    result_sub_ = create_subscription<PerceptionResult>(
      "/perception/result",
      rclcpp::QoS(1).reliable(),  // 계약: Reliable, depth 1
      [this](PerceptionResult::ConstSharedPtr msg) {onPerceptionResult(msg);}, sub_opt);

    range_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/arm_range", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr msg) {onRange(msg);}, sub_opt);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointStates(msg);}, sub_opt);

    // --- Action 클라이언트 ---
    arm_client_ = rclcpp_action::create_client<FollowJT>(this, "/arm_controller/follow_joint_trajectory", job_cb_group_);
    gripper_client_ = rclcpp_action::create_client<GripperCmd>(this, "/gripper_controller/gripper_cmd", job_cb_group_);

    // --- Action 서버 ---
    pose_server_ = rclcpp_action::create_server<ArmPose>(
      this, "/manipulator/pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmPose::Goal> goal) {
        if (poses_.count(goal->name) == 0) {
          RCLCPP_WARN(get_logger(), "pose: unknown name '%s' -> reject", goal->name.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<PoseGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<PoseGoalHandle> gh) {startPose(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);

    pick_server_ = rclcpp_action::create_server<Pick>(
      this, "/manipulator/pick",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<PickGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<PickGoalHandle> gh) {startPick(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);

    // 강화학습 정책 실행. 정책이 연결되지 않았거나(skill.enabled false) 모르는 정책이면 거절 → mission이 fallback
    skill_server_ = rclcpp_action::create_server<ExecuteSkill>(
      this, "/manipulator/execute_skill",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSkill::Goal> goal) {
        if (skill_ready_.count(goal->policy) == 0 && !parseTaskSkill(goal->policy)) {
          RCLCPP_WARN(get_logger(), "execute_skill '%s': not a mission skill and policy not loaded (skill.enabled · "
            "policies.yaml · ONNX Runtime) -> reject", goal->policy.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<SkillGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<SkillGoalHandle> gh) {startSkill(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);
    jt_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", rclcpp::QoS(10));

    // cancel 처리 · 미세 보정 대기용 (20Hz)
    tick_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.05), [this]() {onTick();},
      job_cb_group_);

    RCLCPP_INFO(get_logger(), "manipulator_node ready (poses: %zu, gripper mode: %s, pallet slots: %zu, place cells: %zu)",
      poses_.size(), gripper_mode_name_.c_str(), pallet_.size(), place_cells_.size());
  }

private:
  enum class Job { NONE, POSE, PICK, SKILL };
  enum class PickStep { PLAN, PREGRASP, CORRECT_WAIT, CORRECT_MOVE, OPEN, FINAL, CLOSE, CLOSE_WAIT, LIFT, TASK };
  enum class GripperMode { ARM_JOINT, ACTION, NONE };
  // 미션 skill 종류 (execute_skill 이름)
  enum class Task { NONE, TOPPICK, PALLET_LOAD, PALLET_UNLOAD, PLACE };
  struct TaskSkill
  {
    Task kind{Task::NONE};
    int row{0}, index{0};                 // pallet_*: 열 1~3 · 칸 0~3
    std::string place, cell;              // place_<L>_<dx>_<dy>: 장소 · "dx,dy" (-1 · 0 · 1)
    std::string frame;                    // place_at: 칸 좌표 frame
    double x{0.0}, y{0.0}, z{0.0};        // place_at: 칸 중심 · 윗면 높이
  };
  // pallet_place_traj.json 의 칸 하나: 구간 (이름 · 그리퍼 hold/open · 경유 관절각 5개)
  struct PalletSegment
  {
    std::string name;
    bool hold{true};
    std::vector<std::vector<double>> q;
  };

  // ------------------------------------------------------------------ 파라미터
  void loadParameters()
  {
    // 팔 관절: controllers.yaml의 arm_controller joints와 같아야 한다 (joint1~5 + gripper_roll)
    joint_names_ = declare_parameter<std::vector<std::string>>("arm.joint_names",
      {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"});
    // 관절 한계 = URDF (opencr_hardware/urdf/robot.urdf.xacro) = 펌웨어 config.h ARM_MIN/MAX_RAD (#83). 6 번째 = 그리퍼 개폐
    lower_ = declare_parameter<std::vector<double>>("arm.lower_limits",
      {-3.14159, -2.14, -1.6, -1.92, -2.8, -0.7946});
    upper_ = declare_parameter<std::vector<double>>("arm.upper_limits",
      {3.14159, 1.6, 1.6, 2.0, 2.8, 0.9541});
    max_joint_vel_ = declare_parameter<double>("arm.max_joint_velocity", 0.5);
    min_duration_ = declare_parameter<double>("arm.min_duration", 1.0);
    joint_state_timeout_ = declare_parameter<double>("arm.joint_state_timeout", 1.0);
    requireSize("arm.joint_names", joint_names_.size(), 6);
    requireSize("arm.lower_limits", lower_.size(), 6);
    requireSize("arm.upper_limits", upper_.size(), 6);

    // 기구 치수 = 차체 URDF = 기준 모델 B안 = kinematics.py (arm_ik.hpp 머리말). 값을 바꾸면 URDF 와 같이
    ArmIKParams ik;
    ik.shoulder_height = declare_parameter<double>("ik.shoulder_height", ik.shoulder_height);
    ik.upper_arm_x = declare_parameter<double>("ik.upper_arm_x", ik.upper_arm_x);
    ik.upper_arm_z = declare_parameter<double>("ik.upper_arm_z", ik.upper_arm_z);
    ik.forearm = declare_parameter<double>("ik.forearm", ik.forearm);
    ik.wrist_to_tool = declare_parameter<double>("ik.wrist_to_tool", ik.wrist_to_tool);
    const auto offsets = declare_parameter<std::vector<double>>(
      "ik.joint_offsets", std::vector<double>(5, 0.0));
    const auto signs = declare_parameter<std::vector<double>>(
      "ik.joint_signs", std::vector<double>(5, 1.0));
    requireSize("ik.joint_offsets", offsets.size(), 5);
    requireSize("ik.joint_signs", signs.size(), 5);
    std::copy(offsets.begin(), offsets.end(), ik.joint_offsets.begin());
    std::copy(signs.begin(), signs.end(), ik.joint_signs.begin());
    ik_ = std::make_unique<ArmIK>(ik);
    // 앞에서부터 순서대로 시도하는 집게 pitch 후보 (rad, 위로 +, -pi/4 = 45° 아래, -pi/2 = 바로 아래)
    approach_pitches_ = declare_parameter<std::vector<double>>(
      "ik.approach_pitches", {-M_PI_4, -M_PI_2, -M_PI / 3.0});
    allow_elbow_down_ = declare_parameter<bool>("ik.allow_elbow_down", false);
    use_object_yaw_ = declare_parameter<bool>("ik.use_object_yaw", false);
    wrist_roll_default_ = declare_parameter<double>("ik.wrist_roll_default", 0.0);
    // gripper.mode 가 action · none 일 때 6 번째 칸에 보낼 값 (예전 손목 롤 가정)
    gripper_roll_default_ = declare_parameter<double>("ik.gripper_roll_default", 0.0);

    // 이름 자세 [joint1~5, 6 번째]. gripper.mode=arm_joint 면 6 번째는 쓰지 않고 지금 그리퍼 명령을 유지 (쥔 퍽을 놓치지 않게)
    //   TAG_VIEW = 팔 끝 Arducam이 태그를 보는 자세, BOX_VIEW = 상자 윗면을 내려다보는 자세 (#74)
    const auto names = declare_parameter<std::vector<std::string>>(
      "poses.names", {"STOW", "READY", "PLACE", "TAG_VIEW", "BOX_VIEW", "TRANSIT_TILT"});
    for (const auto & name : names) {
      auto q = declare_parameter<std::vector<double>>("poses." + name, std::vector<double>(6, 0.0));
      requireSize("poses." + name, q.size(), 6);
      poses_[name] = q;
    }

    // 그리퍼: arm_joint (실물 — 6 번째 칸 gripper_roll 이 개폐 모터) | action (gripper_controller) | none (팔만 시험)
    gripper_mode_name_ = declare_parameter<std::string>("gripper.mode", "arm_joint");
    if (gripper_mode_name_ == "arm_joint") {
      gripper_mode_ = GripperMode::ARM_JOINT;
    } else if (gripper_mode_name_ == "action") {
      gripper_mode_ = GripperMode::ACTION;
    } else if (gripper_mode_name_ == "none") {
      gripper_mode_ = GripperMode::NONE;
    } else {
      throw std::invalid_argument("gripper.mode must be arm_joint | action | none");
    }
    gripper_joint_ = declare_parameter<std::string>("gripper.joint_name", "gripper_joint");   // action 모드
    // arm_joint: rad (열림 1550 tick · 닫힘 2650 tick, docs/arm_bench_test_2026-10-27.md). action: 컨트롤러 단위
    gripper_open_ = declare_parameter<double>("gripper.open_position", -0.764);
    gripper_close_ = declare_parameter<double>("gripper.close_position", 0.923);
    gripper_effort_ = declare_parameter<double>("gripper.max_effort", 0.0);  // action 모드, 0 = 지정 안 함
    gripper_move_time_ = declare_parameter<double>("gripper.move_time", 1.0);   // arm_joint: 끝에서 끝 0.9 s (벤치)
    gripper_settle_ = declare_parameter<double>("gripper.settle_time", 0.4);    // arm_joint: 닫은 뒤 판정까지 (GripGuard 0.3 s)
    // arm_joint: 닫힘 명령보다 이만큼 넘게 덜 닫혔으면 퍽을 잡음 (퍽 30 mm 에서 약 +0.51 rad 에서 막힘, 벤치)
    grasp_margin_ = declare_parameter<double>("gripper.grasp_margin", 0.15);
    // 닫았는데 끝까지 닫힘 = 빈손 → pick 실패
    gripper_fail_on_empty_ = declare_parameter<bool>("gripper.fail_on_empty", true);

    // 미션 skill — toppick_tilt: pick 과 같은 과정, 집게 기울기 · 접근 거리만 다름
    toppick_pitch_ = declare_parameter<double>("toppick.pitch", -M_PI_4);
    toppick_approach_ = declare_parameter<double>("toppick.approach_distance", 0.06);
    // 집게 끝(tool_link) = 퍽 중심 + 위로 grasp_z_offset (팔레트 궤적 grasp_dz 0.015 와 같게)
    grasp_z_offset_ = declare_parameter<double>("pick.grasp_z_offset", 0.015);
    // 팔레트 칸 궤적 (빈 문자열 = <control share>/config/pallet_place_traj.json, pallet_traj.py 가 만든 파일 사본)
    pallet_file_ = declare_parameter<std::string>("pallet.traj_file", "");
    pallet_speed_ = declare_parameter<double>("pallet.speed", 0.8);              // rad/s (#75)
    pallet_row3_hold_speed_ = declare_parameter<double>("pallet.row3_hold_speed", 0.4);   // 3 열 쥔 구간 (#75)
    pallet_require_held_ = declare_parameter<bool>("pallet.require_held", true);
    // place_<L>_*: 칸 좌표표 (빈 문자열 = <mission_planner share>/config/approach_poses.yaml), 칸 좌표 frame
    place_cells_file_ = declare_parameter<std::string>("place.cells_file", "");
    place_cells_frame_ = declare_parameter<std::string>("place.cells_frame", "map");
    place_pitch_ = declare_parameter<double>("place.pitch", -M_PI_4);
    place_surface_height_ = declare_parameter<double>("place.surface_height", 0.070);   // 장소 상자 윗면 (발제 Z 70 mm)
    puck_half_height_ = declare_parameter<double>("place.puck_half_height", 0.030);
    place_clearance_ = declare_parameter<double>("place.clearance", 0.002);
    place_approach_ = declare_parameter<double>("place.approach_distance", 0.04);
    place_retreat_ = declare_parameter<double>("place.retreat_distance", 0.04);
    // 바닥 frame (z = 바닥). arm_base_link 높이 = 이 frame → arm_base_link TF, 없으면 arm_base_height
    floor_frame_ = declare_parameter<std::string>("floor_frame", "base_footprint");
    arm_base_height_ = declare_parameter<double>("arm_base_height", 0.120);   // ★ base_link 바닥 위 0.00675 + 0.1135 (B안)

    base_frame_ = declare_parameter<std::string>("arm_base_frame", "arm_base_link");
    // 측정 시각 이후 차체 · 팔이 움직였을 수 있으므로 측정 시각 기준으로 fixed_frame을 거쳐
    // 현재 arm_base_link로 변환한다 (300ms 안이라 차이는 작다)
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    // /arm_range 기준 프레임 (perception arm.range.frame 과 같게). x = 팔 접근 방향
    range_frame_ = declare_parameter<std::string>("range_frame", "tool_link");
    declare_parameter<double>("tf_cache_time", 120.0);

    // 접근 위치 = 파지점(물체 중심) 앞 approach_distance
    approach_distance_ = declare_parameter<double>("pick.approach_distance", 0.15);
    // /arm_range → 파지점까지 거리 환산: 파지점 거리 = range + range_offset
    //   /arm_range 는 끝단(tool_link)에서 퍽 표면까지라 range_offset = 퍽 반지름 (perception puck.half_width 와 같게)
    range_offset_ = declare_parameter<double>("pick.range_offset", 0.015);
    // 측정 기반 파지점 거리와 계획(SEARCH object_pose) 거리 차이 허용값.
    // 넘으면 다른 퍽(이웃)을 잰 것으로 보고 /arm_range 값을 쓰지 않는다
    range_mismatch_tol_ = declare_parameter<double>("pick.range_mismatch_tolerance", 0.04);
    max_final_approach_ = declare_parameter<double>("pick.max_final_approach", 0.25);
    lift_height_ = declare_parameter<double>("pick.lift_height", 0.05);
    // 최종 접근 · 들어올리기 직선 경로: 경유점 간격 (m) · 끝단 속도 (m/s) · 경유점 사이 관절 변화 상한 (rad)
    cartesian_step_ = declare_parameter<double>("pick.cartesian_step", 0.01);
    cartesian_speed_ = declare_parameter<double>("pick.cartesian_speed", 0.05);
    max_waypoint_joint_step_ = declare_parameter<double>("pick.max_waypoint_joint_step", 0.2);
    if (cartesian_step_ <= 0.0 || cartesian_speed_ <= 0.0) {
      throw std::invalid_argument("pick.cartesian_step / pick.cartesian_speed must be > 0");
    }
    // 300ms 규칙 (계약): header.stamp 기준 이보다 오래된 인식 결과 · 거리는 버린다
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.3);

    // pick 목표 ① 정밀 정렬 결과: |arm_offset| <= offset_tolerance이고 /arm_range가 유효하면
    //   팔 끝 축 위 (range + object_radius) 지점을 파지점으로 쓴다
    // TODO(Perception 협의): arm_offset 단위 확정 후 offset_tolerance 조정
    // TODO(Controller): 대상 물체 크기 확정 후 object_radius 조정
    offset_tolerance_ = declare_parameter<double>("pick.offset_tolerance", 0.01);
    object_radius_ = declare_parameter<double>("pick.object_radius", 0.015);
    // pick 목표 ② 물체 위치: 300ms 안의 result에서 object_found = true인 object_pose
    //   (READY 팔이 D435를 가리면 ②는 없을 수 있다 → ① 정밀 정렬 결과가 기본)

    correction_enabled_ = declare_parameter<bool>("correction.enabled", true);
    correction_max_iter_ = declare_parameter<int>("correction.max_iterations", 5);
    // TODO(Perception/Controller): arm_offset 단위 · 축 (추정) — header.frame_id 축 기준,
    //   "카메라 축 → 물체" 벡터로 가정. gain = arm_offset 1단위당 이동 거리(m)
    //   음수면 보정 방향 반전 (부호 규약이 반대일 때), 0이면 좌우 보정 끔
    offset_gain_ = declare_parameter<double>("correction.offset_gain", 1.0);
    lateral_tol_ = declare_parameter<double>("correction.lateral_tolerance", 0.005);
    range_tol_ = declare_parameter<double>("correction.range_tolerance", 0.01);
    // 보정 중 /arm_range 목표값 = 접근 위치에서 나와야 할 측정값 (별도 파라미터로 두면 어긋나므로 계산)
    target_range_ = approach_distance_ - range_offset_;
    if (target_range_ <= 0.0) {
      throw std::invalid_argument("pick.approach_distance - pick.range_offset must be > 0");
    }
    max_step_ = declare_parameter<double>("correction.max_step", 0.02);
    correction_min_duration_ = declare_parameter<double>("correction.min_duration", 0.3);
    settle_time_ = declare_parameter<double>("correction.settle_time", 0.2);
    wait_timeout_ = declare_parameter<double>("correction.wait_timeout", 1.0);
    // true: 보정 실패(arm_offset 없음 · 반복 횟수 초과)면 abort, 재시도는 mission이 판단
    // false: 경고만 남기고 최종 접근 진행
    correction_required_ = declare_parameter<bool>("correction.required", true);

    // 강화학습 정책 (robot_policy 연결 전까지 false → execute_skill은 거절, mission이 fallback)
    skill_enabled_ = declare_parameter<bool>("skill.enabled", false);
    skill_names_ = declare_parameter<std::vector<std::string>>(
      "skill.policies", std::vector<std::string>{});
    // 빈 문자열 = <robot_policy share>/config/policies.yaml (학습 담당이 ONNX와 함께 제공, drive_node와 같은 파일)
    skill_file_ = declare_parameter<std::string>("skill.policies_file", "");
    // 스트리밍 1점 궤적의 time_from_start = horizon / rate_hz (JTC 떨림 방지, 실물 테스트 후 조정)
    skill_horizon_ = declare_parameter<double>("skill.horizon", 1.5);
    // 정책 목표의 한 tick 변화량 ≤ max_joint_velocity / rate_hz (arm.max_joint_velocity 와 별개, 학습 환경 한계와 맞출 것)
    skill_max_joint_vel_ = declare_parameter<double>("skill.max_joint_velocity", 0.5);

    validateParameters(signs);
    loadPolicies();
    loadPallet();
    loadPlaceCells();
  }

  // pallet_place_traj.json (JSON = YAML 이라 yaml-cpp 로 읽음). 없으면 pallet_* skill 이 실패를 돌려준다
  void loadPallet()
  {
    std::string file = pallet_file_;
    try {
      if (file.empty()) {
        file = (ament_index_cpp::get_package_share_path("control") / "config" / "pallet_place_traj.json").string();
      }
      const YAML::Node doc = YAML::LoadFile(file);
      if (doc["transit"] && doc["transit"]["q_front_tilt"]) {
        transit_tilt_ = doc["transit"]["q_front_tilt"].as<std::vector<double>>();
      }
      for (const auto & slot : doc["slots"]) {
        std::vector<PalletSegment> segs;
        for (const auto & g : slot["segments"]) {
          PalletSegment seg;
          seg.name = g["name"].as<std::string>();
          seg.hold = g["gripper"].as<std::string>() == "hold";
          seg.q = g["q"].as<std::vector<std::vector<double>>>();
          segs.push_back(seg);
        }
        pallet_[{slot["row"].as<int>(), slot["index"].as<int>()}] = segs;
      }
      RCLCPP_INFO(get_logger(), "pallet: %zu slot(s) from %s", pallet_.size(), file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "pallet trajectories not loaded (%s): %s -> pallet_* skills will fail",
        file.c_str(), e.what());
    }
  }

  // approach_poses.yaml: <L>.cells."dx,dy".x · y (지도 = 발제 좌표). 없으면 place_<L>_* skill 이 실패를 돌려준다
  void loadPlaceCells()
  {
    std::string file = place_cells_file_;
    try {
      if (file.empty()) {
        file = (ament_index_cpp::get_package_share_path("mission_planner") / "config" / "approach_poses.yaml").string();
      }
      const YAML::Node doc = YAML::LoadFile(file);
      for (const char * L : {"L1", "L2", "L3"}) {
        if (!doc[L] || !doc[L]["cells"]) {continue;}
        for (const auto & kv : doc[L]["cells"]) {
          place_cells_[std::string(L) + "/" + kv.first.as<std::string>()] =
            {kv.second["x"].as<double>(), kv.second["y"].as<double>()};
        }
      }
      RCLCPP_INFO(get_logger(), "place: %zu cell(s) from %s", place_cells_.size(), file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "place cells not loaded (%s): %s -> place_<L>_* skills will fail", file.c_str(), e.what());
    }
  }

  // 노드 시작 시 정책 전부 로드 (실행 중 로드 없음). 실패한 정책은 없는 것으로 → execute_skill 거절 → mission fallback
  void loadPolicies()
  {
    if (!skill_enabled_) {return;}
    std::string file = skill_file_;
    if (file.empty()) {
      file = (ament_index_cpp::get_package_share_path("robot_policy") / "config" / "policies.yaml").string();
    }
    std::vector<std::string> errors;
    policies_.load(file, errors);
    for (const auto & e : errors) {
      RCLCPP_ERROR(get_logger(), "policy load failed: %s", e.c_str());
    }
    for (const auto & name : skill_names_) {
      const auto * p = policies_.find(name);
      if (!p) {
        RCLCPP_ERROR(get_logger(), "skill '%s' not loaded from %s", name.c_str(), file.c_str());
        continue;
      }
      const auto & spec = p->spec;
      if (spec.action_kind != "joint_target" || spec.action_dim != static_cast<int>(joint_names_.size())) {
        RCLCPP_ERROR(get_logger(), "skill '%s': action must be joint_target(6)", name.c_str());
        continue;
      }
      bool known = true;
      for (const auto & o : spec.observation) {
        // 이 노드가 만들 수 있는 관측 (policies.yaml 이름과 같게)
        known &= (o.name == "joint_pos" && o.size == 6) || (o.name == "arm_offset" && o.size == 3) ||
          (o.name == "arm_range" && o.size == 1);
      }
      if (!known) {
        RCLCPP_ERROR(get_logger(), "skill '%s': observation must be joint_pos(6) · arm_offset(3) · arm_range(1)",
          name.c_str());
        continue;
      }
      skill_ready_.insert(name);
      RCLCPP_INFO(get_logger(), "skill '%s' loaded (%s, %.0f Hz)", name.c_str(), spec.model.c_str(), spec.rate_hz);
    }
  }

  // 잘못된 값(0으로 나누기 · 방향 반전 등)은 노드 시작 때 거부한다
  void validateParameters(const std::vector<double> & signs)
  {
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
        throw std::invalid_argument(
                "arm.lower_limits[" + std::to_string(i) + "] must be < arm.upper_limits[" +
                std::to_string(i) + "]");
      }
    }
    for (size_t i = 0; i < signs.size(); ++i) {
      if (!std::isfinite(signs[i]) || std::abs(signs[i]) < 1e-9) {
        throw std::invalid_argument("ik.joint_signs[" + std::to_string(i) + "] must be non-zero");
      }
    }
    if (approach_pitches_.empty()) {
      throw std::invalid_argument("ik.approach_pitches must not be empty");
    }
    for (const auto & [name, q] : poses_) {
      for (double v : q) {
        if (!std::isfinite(v)) {
          throw std::invalid_argument("poses." + name + " has a non-finite value");
        }
      }
    }

    requirePositive("arm.max_joint_velocity", max_joint_vel_);
    requirePositive("arm.min_duration", min_duration_);
    requirePositive("arm.joint_state_timeout", joint_state_timeout_);
    requirePositive("ik.shoulder_height", get_parameter("ik.shoulder_height").as_double());
    requirePositive("ik.forearm", get_parameter("ik.forearm").as_double());
    requirePositive("ik.wrist_to_tool", get_parameter("ik.wrist_to_tool").as_double());
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      for (double g : {gripper_open_, gripper_close_}) {
        if (!(g >= lower_[5] && g <= upper_[5])) {
          throw std::invalid_argument("gripper.open_position / close_position must be within arm limits[5] (gripper_roll)");
        }
      }
      if (!(gripper_close_ > gripper_open_)) {
        throw std::invalid_argument("gripper.close_position must be > open_position (닫을수록 증가)");
      }
    }
    requirePositive("gripper.move_time", gripper_move_time_);
    requireNonNegative("gripper.settle_time", gripper_settle_);
    requirePositive("gripper.grasp_margin", grasp_margin_);
    requirePositive("toppick.approach_distance", toppick_approach_);
    requireNonNegative("pick.grasp_z_offset", grasp_z_offset_);
    requirePositive("pallet.speed", pallet_speed_);
    requirePositive("pallet.row3_hold_speed", pallet_row3_hold_speed_);
    requireNonNegative("place.surface_height", place_surface_height_);
    requirePositive("place.puck_half_height", puck_half_height_);
    requireNonNegative("place.clearance", place_clearance_);
    requirePositive("place.approach_distance", place_approach_);
    requirePositive("place.retreat_distance", place_retreat_);
    requirePositive("arm_base_height", arm_base_height_);
    requireNonNegative("gripper.max_effort", gripper_effort_);
    requirePositive("tf_cache_time", get_parameter("tf_cache_time").as_double());
    requirePositive("pick.approach_distance", approach_distance_);
    requireNonNegative("pick.range_offset", range_offset_);
    requirePositive("pick.range_mismatch_tolerance", range_mismatch_tol_);
    requirePositive("pick.max_final_approach", max_final_approach_);
    requireNonNegative("pick.lift_height", lift_height_);
    requirePositive("pick.max_waypoint_joint_step", max_waypoint_joint_step_);
    requirePositive("stale_timeout", stale_timeout_);
    requirePositive("pick.offset_tolerance", offset_tolerance_);
    requireNonNegative("pick.object_radius", object_radius_);

    requirePositive("skill.horizon", skill_horizon_);
    requirePositive("skill.max_joint_velocity", skill_max_joint_vel_);
    if (correction_max_iter_ < 0) {
      throw std::invalid_argument("correction.max_iterations must be >= 0");
    }
    if (!std::isfinite(offset_gain_)) {
      throw std::invalid_argument("correction.offset_gain must be finite");
    }
    requirePositive("correction.lateral_tolerance", lateral_tol_);
    requirePositive("correction.range_tolerance", range_tol_);
    requirePositive("correction.max_step", max_step_);
    requirePositive("correction.min_duration", correction_min_duration_);
    requireNonNegative("correction.settle_time", settle_time_);
    requirePositive("correction.wait_timeout", wait_timeout_);
  }

  static void requirePositive(const std::string & name, double v)
  {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument(name + " must be > 0 (got " + std::to_string(v) + ")");
    }
  }

  static void requireNonNegative(const std::string & name, double v)
  {
    if (!std::isfinite(v) || v < 0.0) {
      throw std::invalid_argument(name + " must be >= 0 (got " + std::to_string(v) + ")");
    }
  }

  void requireSize(const std::string & name, size_t actual, size_t expected)
  {
    if (actual != expected) {
      throw std::invalid_argument(
              name + ": expected " + std::to_string(expected) + " values, got " +
              std::to_string(actual));
    }
  }

  // ------------------------------------------------------------------ 구독 콜백
  void onPerceptionResult(PerceptionResult::ConstSharedPtr msg)
  {
    // 최신 결과 하나만 둔다. 물체 위치는 perception 추적이 맡으므로 따로 기억하지 않는다
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    last_result_ = msg;
  }

  void onRange(sensor_msgs::msg::Range::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    last_range_ = msg;
  }

  void onJointStates(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      joint_pos_[msg->name[i]] = msg->position[i];
    }
    joint_rx_ = now();
  }

  // /joint_states = IK 시작값 · 궤적 시간 계산 기준
  std::optional<std::vector<double>> currentJoints()
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (joint_rx_.nanoseconds() == 0 || (now() - joint_rx_).seconds() > joint_state_timeout_) {
      return std::nullopt;
    }
    std::vector<double> q;
    for (const auto & name : joint_names_) {
      auto it = joint_pos_.find(name);
      if (it == joint_pos_.end()) {
        return std::nullopt;
      }
      q.push_back(it->second);
    }
    return q;
  }

  // ------------------------------------------------------------------ 공통
  bool withinLimits(const std::vector<double> & q) const
  {
    for (size_t i = 0; i < q.size(); ++i) {
      if (q[i] < lower_[i] || q[i] > upper_[i]) {
        return false;
      }
    }
    return true;
  }

  double trajDuration(
    const std::vector<double> & from, const std::vector<double> & to, double min_duration) const
  {
    double max_delta = 0.0;
    // 팔 관절 5개만 (6 번째 그리퍼는 쥐면 명령과 측정이 달라 시간이 늘어남. 그리퍼 이동 0.9 s 는 min_duration 안)
    for (size_t i = 0; i < std::min<size_t>(5, from.size()); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }
    return std::max(min_duration, max_delta / max_joint_vel_);
  }

  // 위치 p, pitch → 6관절 명령값 (joint1~5 + gripper_roll). 한계 안의 해 중 seed에 가장 가까운 것
  std::optional<std::vector<double>> solveArm(
    const Eigen::Vector3d & p, double pitch, const std::vector<double> & seed) const
  {
    double roll = wrist_roll_default_;
    if (use_object_yaw_ && object_yaw_valid_) {
      // 평행 그리퍼는 180도 대칭이라 [-pi/2, pi/2]로 접는다
      // TODO(Controller): joint5 부호 · 기준은 URDF 확정 후 확인
      roll = std::remainder(object_yaw_ - std::atan2(p.y(), p.x()), M_PI);
    }

    std::optional<std::vector<double>> best;
    double best_cost = 0.0;
    for (bool elbow_up : {true, false}) {
      if (!elbow_up && !allow_elbow_down_) {
        continue;
      }
      const auto sol = ik_->solve(p, pitch, roll, elbow_up);
      if (!sol || (ik_->forward(*sol) - p).norm() > 1e-3) {
        continue;
      }
      std::vector<double> q(sol->begin(), sol->end());
      q.push_back(sixthValue(seed));
      if (!withinLimits(q)) {
        continue;
      }
      double cost = 0.0;
      for (size_t i = 0; i < q.size(); ++i) {
        cost += (q[i] - seed[i]) * (q[i] - seed[i]);
      }
      if (!best || cost < best_cost) {
        best = q;
        best_cost = cost;
      }
    }
    return best;
  }

  // 취소 요청을 받은 작업은 새 명령을 내지 않는다 (마무리는 onTick)
  bool jobCanceling() const
  {
    return (job_ == Job::POSE && pose_gh_->is_canceling()) ||
           (job_ == Job::PICK && pick_gh_->is_canceling()) ||
           (job_ == Job::SKILL && skill_gh_->is_canceling());
  }

  // ------------------------------------------------------------------ 컨트롤러 Action
  // 관절 목표 하나로 이동 (끝 속도 0)
  void sendArm(
    const std::vector<double> & q, double duration, Done done,
    std::function<void(double)> on_progress = nullptr)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q;
    point.velocities.assign(q.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    sendTrajectory({point}, duration, done, on_progress);
  }

  // 여러 점 궤적 전송. duration = 마지막 점의 time_from_start (progress 계산용)
  void sendTrajectory(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points, double duration,
    Done done, std::function<void(double)> on_progress = nullptr)
  {
    if (jobCanceling()) {
      return;
    }
    if (!arm_client_->action_server_is_ready()) {
      done(false, "arm_controller action server not ready");
      return;
    }
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points = points;
    last_sent_ = points.back().positions;   // 그리퍼만 움직일 때 팔 기준

    const uint64_t id = job_id_;
    rclcpp_action::Client<FollowJT>::SendGoalOptions opt;
    opt.goal_response_callback =
      [this, id, done](rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr gh) {
        if (id != job_id_) {
          // 이미 끝난 작업의 goal → 바로 취소
          if (gh) {arm_client_->async_cancel_goal(gh);}
          return;
        }
        if (!gh) {
          done(false, "arm_controller rejected goal");
          return;
        }
        arm_goal_ = gh;
      };
    if (on_progress) {
      opt.feedback_callback =
        [this, id, duration, on_progress](
        rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr,
        const std::shared_ptr<const FollowJT::Feedback> fb) {
          if (id != job_id_) {return;}
          // TODO(Controller): arm_controller가 desired.time_from_start를 채우는지 빌드 후 확인
          //   (안 채우면 progress가 0에 머문다. 팔 동작 · mission에는 영향 없음)
          const double t = rclcpp::Duration(fb->desired.time_from_start).seconds();
          on_progress(std::clamp(t / duration, 0.0, 1.0));
        };
    }
    opt.result_callback =
      [this, id, done](const rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult & wr) {
        if (id != job_id_) {return;}
        arm_goal_.reset();
        const bool ok = wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
          wr.result->error_code == FollowJT::Result::SUCCESSFUL;
        done(ok, ok ? "" : "arm trajectory failed: " +
          (wr.result ? wr.result->error_string : std::string("no result")));
      };
    arm_client_->async_send_goal(goal, opt);
  }

  // 6 번째 칸에 실을 값: arm_joint 모드면 지금 그리퍼 명령 (아직 없으면 측정값), 아니면 ik.gripper_roll_default
  double sixthValue(const std::vector<double> & current) const
  {
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      return gripper_cmd_ ? *gripper_cmd_ : current[5];
    }
    return gripper_roll_default_;
  }

  // 그리퍼 여닫기. arm_joint: 팔은 마지막 명령 자리 그대로 두고 6 번째 값만 바꾼 궤적. action: gripper_cmd Action
  void moveGripper(double position, Done done)
  {
    if (gripper_mode_ == GripperMode::NONE) {
      done(true, "");
      return;
    }
    if (gripper_mode_ == GripperMode::ACTION) {
      sendGripper(position, done);
      return;
    }
    const auto current = currentJoints();
    if (!current) {
      done(false, "no recent /joint_states for arm joints");
      return;
    }
    std::vector<double> q = last_sent_.size() == joint_names_.size() ? last_sent_ : *current;
    gripper_cmd_ = position;
    q[5] = position;
    sendArm(q, gripper_move_time_, done);
  }

  // arm_joint 모드: 그리퍼 측정값이 닫힘 명령보다 grasp_margin 넘게 덜 닫혔으면 손가락이 퍽에 막힘 = 쥐고 있음
  std::optional<bool> gripperHolding()
  {
    const auto current = currentJoints();
    if (!current) {return std::nullopt;}
    return (*current)[5] < gripper_close_ - grasp_margin_;
  }

  void sendGripper(double position, Done done)
  {
    if (jobCanceling()) {
      return;
    }
    if (!gripper_client_->action_server_is_ready()) {
      done(false, "gripper_controller action server not ready");
      return;
    }
    GripperCmd::Goal goal;
    goal.command.name = {gripper_joint_};
    goal.command.position = {position};
    if (gripper_effort_ > 0.0) {
      goal.command.effort = {gripper_effort_};
    }

    const uint64_t id = job_id_;
    rclcpp_action::Client<GripperCmd>::SendGoalOptions opt;
    opt.goal_response_callback =
      [this, id, done](rclcpp_action::ClientGoalHandle<GripperCmd>::SharedPtr gh) {
        if (id != job_id_) {
          if (gh) {gripper_client_->async_cancel_goal(gh);}
          return;
        }
        if (!gh) {
          done(false, "gripper_controller rejected goal");
          return;
        }
        gripper_goal_ = gh;
      };
    opt.result_callback =
      [this, id, done](const rclcpp_action::ClientGoalHandle<GripperCmd>::WrappedResult & wr) {
        if (id != job_id_) {return;}
        gripper_goal_.reset();
        // stalled = 물체를 잡고 멈춤 (grasped 판정에 사용), reached_goal = 목표 위치까지 감
        gripper_stalled_ = wr.result && wr.result->stalled;
        const bool ok = wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
          (wr.result->reached_goal || wr.result->stalled);
        done(ok, ok ? "" : "gripper command failed");
      };
    gripper_client_->async_send_goal(goal, opt);
  }

  void cancelControllers()
  {
    cancelGoal<FollowJT>(arm_client_, arm_goal_, "arm_controller");
    cancelGoal<GripperCmd>(gripper_client_, gripper_goal_, "gripper_controller");
  }

  // 취소 요청 + 응답 확인 (거부되면 ERROR 로그). 이미 끝난 goal(UNKNOWN_GOAL_ID · GOAL_TERMINATED)은 정상
  template<typename ActionT>
  void cancelGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & gh, const std::string & label)
  {
    if (!gh) {
      return;
    }
    client->async_cancel_goal(
      gh, [this, label](typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr res) {
        using CancelGoal = action_msgs::srv::CancelGoal::Response;
        if (!res || res->return_code == CancelGoal::ERROR_REJECTED) {
          RCLCPP_ERROR(get_logger(), "%s rejected cancel request", label.c_str());
        }
      });
    gh.reset();
  }

  // 작업 종료: 진행 중인 컨트롤러 goal 취소, job_id_ 증가로 늦게 오는 콜백 무시
  void endJob()
  {
    cancelControllers();
    job_ = Job::NONE;
    task_ = Task::NONE;
    gripper_wait_ = nullptr;
    pose_gh_.reset();
    pick_gh_.reset();
    skill_gh_.reset();
    // 타이머 콜백 안에서 끝날 수 있으므로 파괴하지 않고 멈추기만 한다 (다음 startSkill에서 교체)
    if (skill_timer_) {skill_timer_->cancel();}
    ++job_id_;
  }

  // 새 goal이 오면 이전 작업은 abort (팔 명령 주체는 하나, 작업도 한 번에 하나)
  void preemptCurrent()
  {
    if (job_ == Job::POSE && pose_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "pose '%s' preempted by new goal", pose_name_.c_str());
      auto result = std::make_shared<ArmPose::Result>();
      result->success = false;
      pose_gh_->abort(result);
    } else if (job_ == Job::PICK && pick_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "pick preempted by new goal");
      auto result = std::make_shared<Pick::Result>();
      result->success = false;
      result->message = "preempted by new goal";
      pick_gh_->abort(result);
    } else if (job_ == Job::SKILL && skill_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "skill preempted by new goal");
      auto result = std::make_shared<ExecuteSkill::Result>();
      result->success = false;
      result->message = "preempted by new goal";
      skill_gh_->abort(result);
    }
    if (job_ != Job::NONE) {
      endJob();
    }
  }

  void onTick()
  {
    if (job_ == Job::POSE && pose_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "pose '%s' canceled", pose_name_.c_str());
      auto result = std::make_shared<ArmPose::Result>();
      result->success = false;
      pose_gh_->canceled(result);
      endJob();
    } else if (job_ == Job::PICK && pick_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "pick canceled");
      auto result = std::make_shared<Pick::Result>();
      result->success = false;
      result->message = "canceled";
      pick_gh_->canceled(result);
      endJob();
    } else if (job_ == Job::SKILL && skill_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "skill canceled");
      auto result = std::make_shared<ExecuteSkill::Result>();
      result->success = false;
      result->message = "canceled";
      skill_gh_->canceled(result);
      endJob();
    } else if (pickActive() && step_ == PickStep::CORRECT_WAIT) {
      correctionTick();
    } else if (gripper_wait_ && (now() - wait_since_).seconds() >= gripper_settle_) {
      auto next = std::move(gripper_wait_);   // 닫은 뒤 GripGuard 가 막힘을 잡을 시간을 두고 위치로 판정
      gripper_wait_ = nullptr;
      next();
    }
  }

  // /manipulator/pick 또는 toppick_tilt skill 이 집는 중
  bool pickActive() const
  {
    return job_ == Job::PICK || (job_ == Job::SKILL && task_ == Task::TOPPICK);
  }

  // ------------------------------------------------------------------ /manipulator/pose
  void startPose(std::shared_ptr<PoseGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::POSE;
    pose_gh_ = gh;
    pose_name_ = gh->get_goal()->name;

    const auto current = currentJoints();
    if (!current) {
      finishPose(false, "no recent /joint_states for arm joints");
      return;
    }
    auto target = poses_.at(pose_name_);
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      target[5] = sixthValue(*current);   // 자세를 바꿔도 그리퍼는 그대로 (쥔 퍽을 놓치지 않게)
    }
    if (!withinLimits(target)) {
      finishPose(false, "pose '" + pose_name_ + "' exceeds joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "pose '%s' start", pose_name_.c_str());
    sendArm(
      target, trajDuration(*current, target, min_duration_),
      [this](bool ok, const std::string & msg) {finishPose(ok, msg);},
      [this](double progress) {
        auto fb = std::make_shared<ArmPose::Feedback>();
        fb->progress = static_cast<float>(progress);
        pose_gh_->publish_feedback(fb);
      });
  }

  void finishPose(bool ok, const std::string & msg)
  {
    auto result = std::make_shared<ArmPose::Result>();
    result->success = ok;
    if (ok) {
      RCLCPP_INFO(get_logger(), "pose '%s' done", pose_name_.c_str());
      pose_gh_->succeed(result);
    } else {
      RCLCPP_WARN(get_logger(), "pose '%s' failed: %s", pose_name_.c_str(), msg.c_str());
      pose_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ /manipulator/pick
  // PLAN → PREGRASP → (CORRECT_WAIT ⇄ CORRECT_MOVE) → OPEN → FINAL → CLOSE → LIFT
  void startPick(std::shared_ptr<PickGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::PICK;
    pick_gh_ = gh;
    beginPick(gh->get_goal()->target, approach_pitches_, approach_distance_);
  }

  // 집기 과정 (pick · toppick_tilt 공용). target 이 비어 있으면 ① 정밀 정렬 결과 → ② 추적 위치
  void beginPick(
    const geometry_msgs::msg::PoseStamped & target, const std::vector<double> & pitches, double approach)
  {
    // 보정 중 /arm_range 목표값 = 접근 위치에서 나와야 할 측정값
    target_range_ = std::max(0.0, approach - range_offset_);
    step_ = PickStep::PLAN;
    correction_iter_ = 0;
    final_grasp_dist_.reset();
    grasped_ = false;
    gripper_stalled_ = false;
    publishStage(STAGE_PLAN);

    Eigen::Vector3d grasp;
    std::string err;
    if (!resolveTarget(target, grasp, err)) {
      finishPick(false, err);
      return;
    }
    // 집게 끝(tool_link) = 퍽 중심 + 위로 grasp_z_offset (팔레트 궤적 TCP 와 같은 잡는 높이)
    grasp += Eigen::Vector3d::UnitZ() * grasp_z_offset_;
    const auto current = currentJoints();
    if (!current) {
      finishPick(false, "no recent /joint_states for arm joints");
      return;
    }

    // pitch 후보를 순서대로: 접근 위치 · 파지 위치 · 들어올린 위치가 모두 풀리는 첫 pitch
    const double yaw = std::atan2(grasp.y(), grasp.x());
    std::optional<std::vector<double>> q_pre;
    for (double pitch : pitches) {
      const Eigen::Vector3d pre = grasp - ArmIK::approachDirection(yaw, pitch) * approach;
      auto q = solveArm(pre, pitch, *current);
      if (q && solveArm(grasp, pitch, *q) &&
        solveArm(grasp + Eigen::Vector3d::UnitZ() * lift_height_, pitch, *q))
      {
        q_pre = q;
        pitch_ = pitch;
        tool_pos_ = pre;
        grasp_pos_ = grasp;
        break;
      }
    }
    if (!q_pre) {
      finishPick(false, "IK: no solution within joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "pick: target (%.3f, %.3f, %.3f) in %s, pitch %.2f rad",
      grasp.x(), grasp.y(), grasp.z(), base_frame_.c_str(), pitch_);

    step_ = PickStep::PREGRASP;
    publishStage(STAGE_APPROACH);
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      gripper_cmd_ = gripper_open_;   // 접근하면서 연다
      (*q_pre)[5] = gripper_open_;
    }
    sendArm(*q_pre, trajDuration(*current, *q_pre, min_duration_),
      [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, msg);
        } else if (correction_enabled_) {
          beginCorrectionWait();
        } else {
          openGripper();
        }
      });
  }

  // 300ms 규칙: 측정 시각이 stale_timeout 안인 최신 결과 · 거리 (없으면 nullptr)
  void freshSensors(
    PerceptionResult::ConstSharedPtr & result, sensor_msgs::msg::Range::ConstSharedPtr & range)
  {
    const rclcpp::Time stale_before = now() - rclcpp::Duration::from_seconds(stale_timeout_);
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    result = (last_result_ && measuredAfter(last_result_->header.stamp, stale_before)) ? last_result_ : nullptr;
    range = (last_range_ && measuredAfter(last_range_->header.stamp, stale_before)) ? last_range_ : nullptr;
  }

  static bool rangeValid(const sensor_msgs::msg::Range & r)
  {
    return std::isfinite(r.range) && r.range >= r.min_range && r.range <= r.max_range;
  }

  // pick 목표 (arm_base_link 기준 파지점)
  //   goal.target 지정 → 그 위치
  //   비어 있으면 ① 정밀 정렬 결과 → ② perception 추적 위치 순서
  bool resolveTarget(
    const geometry_msgs::msg::PoseStamped & goal_target, Eigen::Vector3d & out, std::string & err)
  {
    geometry_msgs::msg::PoseStamped src = goal_target;
    if (src.header.frame_id.empty()) {
      PerceptionResult::ConstSharedPtr result;
      sensor_msgs::msg::Range::ConstSharedPtr range;
      freshSensors(result, range);
      const bool offset_ok = result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid &&
        std::hypot(result->arm_offset.x, result->arm_offset.y, result->arm_offset.z) <= offset_tolerance_;
      const bool range_ok = range && rangeValid(*range);
      if (offset_ok && range_ok) {
        // ① 팔 끝 축(x) 위 퍽 표면 + 물체 반지름 = 파지점 (range_frame 기준, /arm_range stamp 의 팔 자세)
        src.header = range->header;
        src.header.frame_id = range_frame_;
        src.pose = geometry_msgs::msg::Pose();
        src.pose.position.x = range->range + object_radius_;
        RCLCPP_INFO(get_logger(), "pick target: ① precision alignment (range %.3f m)", range->range);
      } else if (result && result->object_found) {
        // ② perception 물체 위치 (base_link 기준, 300ms 안의 result)
        src = result->object_pose;
        if (src.header.frame_id.empty()) {src.header = result->header;}
        RCLCPP_INFO(get_logger(), "pick target: ② perception object_pose");
      } else {
        err = "no pick target: precision alignment not ready (offset_valid · |arm_offset| <= " +
          std::to_string(offset_tolerance_) + " · /arm_range) and no object (fresh result with object_found)";
        return false;
      }
    }
    // object_pose 방향이 비어 있으면(quaternion 크기 0) yaw를 쓰지 않는다
    // TODO(Perception 협의): object_pose.orientation을 채울지 (물체 방향 추정). 안 채우면 use_object_yaw는 무의미
    auto & o = src.pose.orientation;
    const double qn = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    object_yaw_valid_ = std::isfinite(qn) && qn > 1e-6 && src.header.frame_id != range_frame_;
    if (object_yaw_valid_) {
      // 단위 quaternion이 아니면 getYaw 결과가 틀어지므로 정규화
      o.x /= qn; o.y /= qn; o.z /= qn; o.w /= qn;
    } else {
      o.x = o.y = o.z = 0.0;
      o.w = 1.0;  // 위치만 변환 (방향 무시)
      if (use_object_yaw_) {
        RCLCPP_WARN(get_logger(), "pick: no object orientation -> wrist roll = ik.wrist_roll_default");
      }
    }
    try {
      // 측정 시각의 src → fixed_frame → 현재 arm_base_link
      // stamp는 ROS 시계 기준 촬영 시각 (계약). TF는 측정값보다 몇 ms 늦게 들어오므로
      // 측정 시각의 TF가 아직 없으면 최신 TF로 변환한다 (측정값은 300ms 안이라 차이는 작다)
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(
          base_frame_, tf2::TimePointZero, src.header.frame_id,
          tf2_ros::fromMsg(src.header.stamp), fixed_frame_, tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException &) {
        tf = tf_buffer_->lookupTransform(base_frame_, src.header.frame_id, tf2::TimePointZero);
      }
      geometry_msgs::msg::PoseStamped dst;
      tf2::doTransform(src, dst, tf);
      out = {dst.pose.position.x, dst.pose.position.y, dst.pose.position.z};
      object_yaw_ = object_yaw_valid_ ? tf2::getYaw(dst.pose.orientation) : 0.0;
    } catch (const tf2::TransformException & e) {
      err = std::string("TF ") + src.header.frame_id + " -> " + base_frame_ + ": " + e.what();
      return false;
    }
    return true;
  }

  void beginCorrectionWait()
  {
    step_ = PickStep::CORRECT_WAIT;
    wait_since_ = now();
  }

  // 측정 시각 header.stamp (ROS 시계 기준 촬영 · 측정 시각, 계약)가 since 이후인지.
  // 300ms 규칙은 예외 없음 — stamp가 비어 있으면 측정 시각을 알 수 없으므로 버린다
  static bool measuredAfter(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & since)
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return false;
    }
    return rclcpp::Time(stamp, RCL_ROS_TIME) >= since;
  }

  // 미세 보정: arm_offset(좌우 · 상하) · /arm_range(전후)로 끝단을 작은 궤적으로 반복 이동
  void correctionTick()
  {
    const rclcpp::Time t = now();
    // 팔이 멈춘 뒤 settle_time 이후에 촬영 · 측정된 데이터만 사용 (인식 지연 중 움직이던 영상 제외)
    const rclcpp::Time fresh_after = wait_since_ + rclcpp::Duration::from_seconds(settle_time_);

    PerceptionResult::ConstSharedPtr result;
    sensor_msgs::msg::Range::ConstSharedPtr range;
    // + 300ms 규칙: 측정 시각이 stale_timeout보다 오래되면 버린다
    const rclcpp::Time stale_before = t - rclcpp::Duration::from_seconds(stale_timeout_);
    const rclcpp::Time since = std::max(fresh_after, stale_before);
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (last_result_ && measuredAfter(last_result_->header.stamp, since)) {result = last_result_;}
      if (last_range_ && measuredAfter(last_range_->header.stamp, since)) {range = last_range_;}
    }
    const bool offset_ok = result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid;
    const Eigen::Vector3d dir =
      ArmIK::approachDirection(std::atan2(tool_pos_.y(), tool_pos_.x()), pitch_);

    // /arm_range → 파지점 거리. 계획 거리와 크게 다르면(이웃 퍽을 잼) 쓰지 않는다.
    // 차이(= SEARCH 깊이 오차)는 보정으로 끝단이 움직여도 그대로 유지되므로 매번 같은 기준으로 판단
    bool range_ok = range && rangeValid(*range);
    if (range_ok) {
      const double measured = range->range + range_offset_;
      const double planned = (grasp_pos_ - tool_pos_).dot(dir);
      if (std::abs(measured - planned) > range_mismatch_tol_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "correction: range-based %.3fm vs planned %.3fm (> %.3fm) -> ignore /arm_range",
          measured, planned, range_mismatch_tol_);
        range_ok = false;
      }
    }
    // 보정을 끝내는 시점(현재 위치)의 측정 기반 파지점 거리 → 최종 접근에 사용
    final_grasp_dist_ =
      range_ok ? std::optional<double>(range->range + range_offset_) : std::nullopt;

    if (!offset_ok) {
      if ((t - wait_since_).seconds() > wait_timeout_) {
        correctionFailed("no valid arm_offset for " + std::to_string(wait_timeout_) + "s");
      }
      return;
    }

    // arm_offset (header.frame_id 축) → arm_base_link, 접근 방향 성분은 제거
    Eigen::Vector3d lateral;
    try {
      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = result->header.frame_id;
      v_in.vector = result->arm_offset;
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, result->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(v_in, v_out, tf);
      const Eigen::Vector3d v =
        offset_gain_ * Eigen::Vector3d(v_out.vector.x, v_out.vector.y, v_out.vector.z);
      lateral = v - v.dot(dir) * dir;
    } catch (const tf2::TransformException & e) {
      finishPick(false, std::string("correction TF failed: ") + e.what());
      return;
    }
    const double axial = range_ok ? range->range - target_range_ : 0.0;

    if (lateral.norm() < lateral_tol_ && (!range_ok || std::abs(axial) < range_tol_)) {
      RCLCPP_INFO(get_logger(), "correction: aligned after %d step(s)", correction_iter_);
      openGripper();
      return;
    }
    if (correction_iter_ >= correction_max_iter_) {
      correctionFailed("not aligned after max iterations (" +
        std::to_string(correction_max_iter_) + ")");
      return;
    }

    Eigen::Vector3d delta = lateral + axial * dir;
    if (delta.norm() > max_step_) {
      delta *= max_step_ / delta.norm();
    }
    const Eigen::Vector3d next = tool_pos_ + delta;
    const auto current = currentJoints();
    if (!current) {
      finishPick(false, "no recent /joint_states for arm joints");
      return;
    }
    const auto q = solveArm(next, pitch_, *current);
    if (!q) {
      finishPick(false, "IK: correction target out of reach");
      return;
    }

    step_ = PickStep::CORRECT_MOVE;
    ++correction_iter_;
    sendArm(*q, trajDuration(*current, *q, correction_min_duration_),
      [this, next](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, msg);
          return;
        }
        tool_pos_ = next;
        beginCorrectionWait();
      });
  }

  // 보정 실패: correction.required면 abort (재시도는 mission), 아니면 경고 후 최종 접근 진행
  void correctionFailed(const std::string & reason)
  {
    if (correction_required_) {
      finishPick(false, "correction failed: " + reason);
      return;
    }
    RCLCPP_WARN(get_logger(), "correction: %s -> continue (correction.required: false)",
      reason.c_str());
    openGripper();
  }

  void openGripper()
  {
    step_ = PickStep::OPEN;
    if (gripper_mode_ == GripperMode::NONE ||
      (gripper_mode_ == GripperMode::ARM_JOINT && gripper_cmd_ && *gripper_cmd_ == gripper_open_))
    {
      finalApproach();   // 이미 열림 (접근하면서 열었음)
      return;
    }
    moveGripper(gripper_open_, [this](bool ok, const std::string & msg) {
        if (ok) {
          finalApproach();
        } else {
          finishPick(false, msg);
        }
      });
  }

  // 최종 전진 거리: 보정 종료 시점의 측정 기반 파지점 거리 (range + range_offset).
  // 측정값이 없거나 계획과 어긋나 버렸으면 계획한 파지점(object_pose)까지의 거리.
  // 전진 중에는 집게가 퍽을 가려 측위가 흔들리므로 다시 재지 않고 IK 직선 경로로 정해진 거리만 간다.
  // 접촉 · 파지는 그리퍼 결과(stalled)로 확인한다 (closeGripper)
  void finalApproach()
  {
    step_ = PickStep::FINAL;
    const Eigen::Vector3d dir =
      ArmIK::approachDirection(std::atan2(tool_pos_.y(), tool_pos_.x()), pitch_);
    double distance;
    if (final_grasp_dist_) {
      distance = *final_grasp_dist_;
      RCLCPP_INFO(get_logger(), "final approach: %.3fm (from /arm_range)", distance);
    } else {
      distance = (grasp_pos_ - tool_pos_).dot(dir);
      RCLCPP_INFO(get_logger(), "final approach: %.3fm (to planned grasp point)", distance);
    }
    if (distance < 0.0 || distance > max_final_approach_) {
      finishPick(false, "final approach distance " + std::to_string(distance) +
        "m out of range [0, " + std::to_string(max_final_approach_) + "]");
      return;
    }
    moveTool(tool_pos_ + dir * distance, [this]() {closeGripper();});
  }

  void closeGripper()
  {
    step_ = PickStep::CLOSE;
    publishStage(STAGE_GRASP);
    if (gripper_mode_ == GripperMode::NONE) {
      // 그리퍼 없이 팔만 테스트: 파지 확인 불가 → grasped = true로 보고 (mission require_grasped 통과)
      RCLCPP_WARN(get_logger(), "gripper.mode none -> grasp not verified");
      grasped_ = true;
      lift();
      return;
    }
    moveGripper(gripper_close_, [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, "grasp failed: " + msg);
          return;
        }
        if (gripper_mode_ == GripperMode::ACTION) {
          afterGrasp(gripper_stalled_);
          return;
        }
        step_ = PickStep::CLOSE_WAIT;
        waitGripper([this]() {
            const auto holding = gripperHolding();
            if (!holding) {
              finishPick(false, "no recent /joint_states for gripper");
              return;
            }
            afterGrasp(*holding);
          });
      });
  }

  // 닫은 뒤 gripper.settle_time 지나서 then (onTick)
  void waitGripper(std::function<void()> then)
  {
    wait_since_ = now();
    gripper_wait_ = std::move(then);
  }

  void afterGrasp(bool holding)
  {
    grasped_ = holding;
    if (!grasped_ && gripper_fail_on_empty_) {
      finishPick(false, "empty grasp: gripper closed fully without contact");
      return;
    }
    lift();
  }

  void lift()
  {
    step_ = PickStep::LIFT;
    publishStage(STAGE_LIFT);
    moveTool(tool_pos_ + Eigen::Vector3d::UnitZ() * lift_height_,
      [this]() {finishPick(true, "");});
  }

  // 같은 pitch로 끝단을 target까지 "직선"으로 이동 (최종 접근 · 들어올리기).
  // 관절 목표 하나만 보내면 관절 공간 보간이라 끝단 경로가 휘므로,
  // cartesian_step 간격 경유점마다 IK를 풀어 여러 점 궤적으로 보낸다.
  // 성공하면 tool_pos_ 갱신 후 on_success
  void moveTool(const Eigen::Vector3d & target, std::function<void()> on_success)
  {
    const auto current = currentJoints();
    if (!current) {
      failJob("no recent /joint_states for arm joints");
      return;
    }
    const Eigen::Vector3d start = tool_pos_;
    const double length = (target - start).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(length / cartesian_step_)));
    const double segment_min_time = (length / n) / cartesian_speed_;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    std::vector<double> seed = *current;
    double t = 0.0;
    for (int i = 1; i <= n; ++i) {
      const Eigen::Vector3d p = start + (target - start) * (static_cast<double>(i) / n);
      const auto q = solveArm(p, pitch_, seed);
      if (!q) {
        failJob("IK: straight path out of reach at waypoint " +
          std::to_string(i) + "/" + std::to_string(n));
        return;
      }
      double max_delta = 0.0;
      for (size_t j = 0; j < 5; ++j) {   // 팔 관절만 (6 번째 = 그리퍼, 쥐면 명령과 측정이 다름)
        max_delta = std::max(max_delta, std::abs((*q)[j] - seed[j]));
      }
      // 경유점 사이 관절이 크게 튀면 특이점 · 해 전환 → 직선 경로가 아님
      if (max_delta > max_waypoint_joint_step_) {
        failJob("IK: joint jump " + std::to_string(max_delta) +
          " rad on straight path at waypoint " + std::to_string(i) + "/" + std::to_string(n));
        return;
      }
      t += std::max(segment_min_time, max_delta / max_joint_vel_);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = *q;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      points.push_back(point);
      seed = *q;
    }
    if (points.size() == 1) {
      // 한 점이면 끝 속도 0을 줘서 부드럽게 (여러 점은 위치만 → 경유점 사이 선형 보간)
      t = std::max(t, correction_min_duration_);
      points.front().velocities.assign(points.front().positions.size(), 0.0);
      points.front().time_from_start = rclcpp::Duration::from_seconds(t);
    }
    sendTrajectory(points, t,
      [this, target, on_success](bool ok, const std::string & msg) {
        if (!ok) {
          failJob(msg);
          return;
        }
        tool_pos_ = target;
        on_success();
      });
  }

  void publishStage(uint8_t stage)
  {
    if (job_ == Job::SKILL) {
      auto fb = std::make_shared<ExecuteSkill::Feedback>();
      fb->progress = static_cast<float>(stage) / 4.0f;
      skill_gh_->publish_feedback(fb);
      return;
    }
    auto fb = std::make_shared<Pick::Feedback>();
    fb->stage = stage;
    pick_gh_->publish_feedback(fb);
  }

  // 진행 중인 작업을 실패로 끝냄 (pick · 미션 skill 공용, moveTool · moveGripper 실패)
  void failJob(const std::string & msg)
  {
    if (job_ == Job::SKILL) {
      finishSkill(false, msg);
    } else if (job_ == Job::PICK) {
      finishPick(false, msg);
    }
  }

  void finishPick(bool ok, const std::string & msg)
  {
    if (job_ == Job::SKILL) {   // toppick_tilt
      finishSkill(ok, ok ? "grasped" : msg);
      return;
    }
    auto result = std::make_shared<Pick::Result>();
    result->success = ok;
    result->grasped = ok && grasped_;
    result->message = msg;
    if (ok) {
      RCLCPP_INFO(get_logger(), "pick done (grasped: %s)", grasped_ ? "true" : "false");
      pick_gh_->succeed(result);
    } else {
      // 재시도는 mission_node가 판단
      RCLCPP_WARN(get_logger(), "pick failed: %s", msg.c_str());
      pick_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ /manipulator/execute_skill
  // 강화학습 정책을 policies.yaml의 rate_hz로 실행: 관측 → 정책 → 관절 목표 6개 → /arm_controller/joint_trajectory (1점)
  // 끝: goal.timeout 도달 — policies.yaml finish_on_timeout true면 성공, 아니면 실패 · 추론 실패 · required 관측 무효
  // TODO(Controller · 학습 담당): 팔 정책의 성공 판정 방법 미정 (지금은 finish_on_timeout만)
  void startSkill(std::shared_ptr<SkillGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::SKILL;
    skill_gh_ = gh;
    skill_start_ = now();
    const double t = gh->get_goal()->timeout;
    skill_timeout_ = (std::isfinite(t) && t > 0.0) ? t : 30.0;
    RCLCPP_INFO(get_logger(), "skill '%s' start (timeout %.1fs)", gh->get_goal()->policy.c_str(),
      skill_timeout_);
    // 같은 이름 정책이 로드돼 있으면 정책, 아니면 미션 skill 고정 동작 (IK · 팔레트 궤적)
    if (skill_ready_.count(gh->get_goal()->policy) == 0) {
      startTaskSkill(*parseTaskSkill(gh->get_goal()->policy));   // accept 때 parseTaskSkill 로 확인함
      return;
    }
    active_policy_ = policies_.find(gh->get_goal()->policy);   // accept 때 skill_ready_로 확인함
    skill_rate_ = active_policy_->spec.rate_hz;
    skill_last_cmd_.reset();   // 첫 tick 에 현재 관절각으로 초기화
    const uint64_t id = job_id_;
    skill_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / skill_rate_),
      [this, id]() {if (id == job_id_) {skillTick();}}, job_cb_group_);
  }

  void skillTick()
  {
    if (job_ != Job::SKILL || skill_gh_->is_canceling()) {return;}  // 취소 마무리는 onTick
    const double elapsed = (now() - skill_start_).seconds();
    if (elapsed > skill_timeout_) {
      if (active_policy_->spec.finish_on_timeout) {
        finishSkill(true, "finished at timeout (" + std::to_string(skill_timeout_) + "s)");
      } else {
        finishSkill(false, "timeout (" + std::to_string(skill_timeout_) + "s)");
      }
      return;
    }
    const auto q = currentJoints();
    if (!q) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    // 관측: policies.yaml 순서 · 배치 (required 무효 → abort, 선택 무효 → fill + valid 0)
    PerceptionResult::ConstSharedPtr result;
    sensor_msgs::msg::Range::ConstSharedPtr range;
    freshSensors(result, range);
    robot_policy::ObsValues values;   // 없는 항목 = 무효 (선택 항목은 fill + valid 0)
    values["joint_pos"] = std::vector<float>(q->begin(), q->end());
    if (result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid) {
      values["arm_offset"] = std::vector<float>{static_cast<float>(result->arm_offset.x),
        static_cast<float>(result->arm_offset.y), static_cast<float>(result->arm_offset.z)};
    }
    if (range && rangeValid(*range)) {values["arm_range"] = std::vector<float>{range->range};}
    const auto & spec = active_policy_->spec;
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(spec).build(values, obs, err)) {
      finishSkill(false, "required observation invalid: " + err);
      return;
    }
    if (!active_policy_->runner->run(obs, raw)) {finishSkill(false, "policy inference failed"); return;}
    // action_low · action_high clamp (ActionMapper) 뒤 관절 한계로 다시 clamp (계약)
    const auto mapped = robot_policy::ActionMapper(spec).map(raw);
    // 기준 = 직전 목표 (측정값 기준이면 추종이 늦을 때 목표가 계속 뒤처짐). 정책 출력이 튀어도 급동작 없음
    if (!skill_last_cmd_) {skill_last_cmd_ = *q;}
    const double max_step = skill_max_joint_vel_ / skill_rate_;
    std::vector<double> action(joint_names_.size());
    for (size_t i = 0; i < action.size(); ++i) {
      const double prev = (*skill_last_cmd_)[i];
      const double target = std::clamp(static_cast<double>(mapped[i]), lower_[i], upper_[i]);
      action[i] = std::clamp(target, prev - max_step, prev + max_step);
    }
    skill_last_cmd_ = action;
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = action;
    point.time_from_start = rclcpp::Duration::from_seconds(skill_horizon_ / skill_rate_);
    jt.points.push_back(point);
    jt_pub_->publish(jt);

    auto fb = std::make_shared<ExecuteSkill::Feedback>();
    fb->progress = static_cast<float>(std::clamp(elapsed / skill_timeout_, 0.0, 1.0));
    skill_gh_->publish_feedback(fb);
  }

  void finishSkill(bool ok, const std::string & msg)
  {
    auto result = std::make_shared<ExecuteSkill::Result>();
    result->success = ok;
    result->message = msg;
    if (ok) {
      RCLCPP_INFO(get_logger(), "skill done");
      skill_gh_->succeed(result);
    } else {
      RCLCPP_WARN(get_logger(), "skill failed: %s", msg.c_str());
      skill_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ 미션 skill (execute_skill 이름)
  // "toppick_tilt" · "pallet_load_3_1_tilt" · "pallet_unload_2_0_tilt" · "place_L1_m1_p1_tilt" · "place_at:odom:x:y:z"
  static std::optional<TaskSkill> parseTaskSkill(const std::string & name)
  {
    TaskSkill t;
    if (name == "toppick_tilt") {
      t.kind = Task::TOPPICK;
      return t;
    }
    auto parts = [](const std::string & text, char sep) {
        std::vector<std::string> out;
        std::string tok;
        std::istringstream in(text);
        while (std::getline(in, tok, sep)) {out.push_back(tok);}
        return out;
      };
    if (name.rfind("place_at:", 0) == 0) {
      const auto f = parts(name, ':');   // place_at : frame : x : y : z
      if (f.size() != 5 || f[1].empty()) {return std::nullopt;}
      try {
        t.kind = Task::PLACE;
        t.frame = f[1];
        t.x = std::stod(f[2]);
        t.y = std::stod(f[3]);
        t.z = std::stod(f[4]);
      } catch (const std::exception &) {
        return std::nullopt;
      }
      return t;
    }
    const auto f = parts(name, '_');
    if (f.size() == 5 && f[0] == "pallet" && (f[1] == "load" || f[1] == "unload") && f[4] == "tilt" &&
      f[2].size() == 1 && f[3].size() == 1 && f[2][0] >= '1' && f[2][0] <= '3' && f[3][0] >= '0' && f[3][0] <= '3')
    {
      t.kind = f[1] == "load" ? Task::PALLET_LOAD : Task::PALLET_UNLOAD;
      t.row = f[2][0] - '0';
      t.index = f[3][0] - '0';
      return t;
    }
    // place_<L>_<dx>_<dy>_tilt, dx · dy = m1 · 0 · p1 (지도 +Y 가 위)
    static const std::map<std::string, std::string> idx = {{"m1", "-1"}, {"0", "0"}, {"p1", "1"}};
    if (f.size() == 5 && f[0] == "place" && (f[1] == "L1" || f[1] == "L2" || f[1] == "L3") && f[4] == "tilt" &&
      idx.count(f[2]) && idx.count(f[3]))
    {
      t.kind = Task::PLACE;
      t.place = f[1];
      t.cell = idx.at(f[2]) + "," + idx.at(f[3]);
      return t;
    }
    return std::nullopt;
  }

  void startTaskSkill(const TaskSkill & t)
  {
    task_ = t.kind;
    switch (t.kind) {
      case Task::TOPPICK:
        beginPick(geometry_msgs::msg::PoseStamped(), {toppick_pitch_}, toppick_approach_);
        return;
      case Task::PALLET_LOAD:
      case Task::PALLET_UNLOAD:
        runPalletSlot(t.row, t.index, t.kind == Task::PALLET_LOAD);
        return;
      case Task::PLACE: {
          geometry_msgs::msg::PointStamped cell;
          if (t.place.empty()) {   // place_at
            cell.header.frame_id = t.frame;
            cell.point.x = t.x;
            cell.point.y = t.y;
            cell.point.z = t.z;
            placeAt(cell, true);
            return;
          }
          const auto it = place_cells_.find(t.place + "/" + t.cell);
          if (it == place_cells_.end()) {
            finishSkill(false, "place: no cell " + t.place + " " + t.cell + " in place.cells_file");
            return;
          }
          cell.header.frame_id = place_cells_frame_;
          cell.point.x = it->second.first;
          cell.point.y = it->second.second;
          placeAt(cell, false);
          return;
        }
      default:
        finishSkill(false, "unknown mission skill");
        return;
    }
  }

  // 팔레트 칸 궤적 재생. load: 쥔 채 칸으로 → 놓기 → 빠져나옴 / unload: 벌린 채 칸으로 → 집기 → 쥔 채 빠져나옴
  //   궤적의 rotate · carry · place (칸까지) 와 retract · return · rotate_back (돌아오기) 사이에서 그리퍼를 바꾼다
  void runPalletSlot(int row, int index, bool load)
  {
    step_ = PickStep::TASK;
    const auto it = pallet_.find({row, index});
    if (it == pallet_.end()) {
      finishSkill(false, "pallet: no trajectory for row " + std::to_string(row) + " slot " + std::to_string(index) +
        " (pallet.traj_file)");
      return;
    }
    if (load && pallet_require_held_ && gripper_mode_ == GripperMode::ARM_JOINT) {
      const auto holding = gripperHolding();
      if (!holding || !*holding) {
        finishSkill(false, "pallet_load: not holding a puck (gripper closed fully or no /joint_states)");
        return;
      }
    }
    std::vector<const PalletSegment *> in, out;
    bool released = false;
    for (const auto & seg : it->second) {
      if (seg.name == "release") {released = true; continue;}
      (released ? out : in).push_back(&seg);
    }
    // 쥔 구간 속도: 놓기의 3 열만 0.4 rad/s (#75), 나머지 0.8
    const double in_speed = (load && row == 3) ? pallet_row3_hold_speed_ : pallet_speed_;
    const double out_speed = (!load && row == 3) ? pallet_row3_hold_speed_ : pallet_speed_;
    const double in_grip = load ? gripper_close_ : gripper_open_;
    const double out_grip = load ? gripper_open_ : gripper_close_;
    if (gripper_mode_ == GripperMode::ARM_JOINT) {gripper_cmd_ = in_grip;}
    sendSegments(in, in_speed, in_grip, [this, out, out_speed, out_grip, load](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "pallet approach: " + msg);
          return;
        }
        moveGripper(out_grip, [this, out, out_speed, out_grip, load](bool gok, const std::string & gmsg) {
            if (!gok) {
              finishSkill(false, "pallet gripper: " + gmsg);
              return;
            }
            auto leave = [this, out, out_speed, out_grip, load]() {
                sendSegments(out, out_speed, out_grip, [this, load](bool lok, const std::string & lmsg) {
                    finishSkill(lok, lok ? (load ? "loaded" : "unloaded") : "pallet leave: " + lmsg);
                  });
              };
            if (load || gripper_mode_ != GripperMode::ARM_JOINT) {
              leave();
              return;
            }
            waitGripper([this, leave]() {   // unload: 집었는지 확인 후 빠져나옴
                const auto holding = gripperHolding();
                if (gripper_fail_on_empty_ && (!holding || !*holding)) {
                  finishSkill(false, "pallet_unload: empty grasp (no puck in slot?)");
                  return;
                }
                leave();
              });
          });
      });
  }

  // 구간 경유점을 한 궤적으로. 경유점 사이 시간 = 관절 변화 최대값 / speed, 첫 점은 지금 자세에서 출발
  void sendSegments(
    const std::vector<const PalletSegment *> & segs, double speed, double grip, Done done)
  {
    const auto current = currentJoints();
    if (!current) {
      done(false, "no recent /joint_states for arm joints");
      return;
    }
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    std::vector<double> prev = *current;
    double t = 0.0;
    for (const auto * seg : segs) {
      for (const auto & q5 : seg->q) {
        if (q5.size() != 5) {
          done(false, "pallet: waypoint must have 5 joints (" + seg->name + ")");
          return;
        }
        std::vector<double> q(q5.begin(), q5.end());
        q.push_back(gripper_mode_ == GripperMode::ARM_JOINT ? grip : sixthValue(*current));
        if (!withinLimits(q)) {
          done(false, "pallet: waypoint outside joint limits (" + seg->name + ")");
          return;
        }
        double max_delta = 0.0;
        for (size_t j = 0; j < 5; ++j) {max_delta = std::max(max_delta, std::abs(q[j] - prev[j]));}
        // 첫 점은 지금 자세에서 오는 거리라 arm.min_duration 이상
        t += points.empty() ? std::max(min_duration_, max_delta / speed) : std::max(0.02, max_delta / speed);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = q;
        point.time_from_start = rclcpp::Duration::from_seconds(t);
        points.push_back(point);
        prev = q;
      }
    }
    if (points.empty()) {
      done(false, "pallet: empty trajectory");
      return;
    }
    points.back().velocities.assign(points.back().positions.size(), 0.0);
    sendTrajectory(points, t, done);
  }

  // 쥔 퍽을 칸 (frame 기준 x, y, 윗면 높이 z) 에 놓기. cell.z 를 안 쓰면 (has_z false) place.surface_height (바닥 기준)
  //   집게 place.pitch (45°) · 집게 끝 = 퍽 중심 + grasp_z_offset → APPROACH (위 · 뒤) → LOWER (직선) → 벌림 → RETREAT → 이동 자세
  void placeAt(const geometry_msgs::msg::PointStamped & cell, bool has_z)
  {
    step_ = PickStep::TASK;
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      const auto holding = gripperHolding();
      if (!holding || !*holding) {
        finishSkill(false, "place: not holding a puck (gripper closed fully or no /joint_states)");
        return;
      }
    }
    // 칸 → arm_base_link (최신 TF)
    geometry_msgs::msg::PointStamped arm;
    double floor_z = 0.0;   // arm_base_link 기준 바닥 높이
    try {
      geometry_msgs::msg::PointStamped src = cell;
      src.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      arm = tf_buffer_->transform(src, base_frame_);
    } catch (const tf2::TransformException & e) {
      finishSkill(false, std::string("place: TF ") + cell.header.frame_id + " -> " + base_frame_ + ": " + e.what());
      return;
    }
    try {
      floor_z = -tf_buffer_->lookupTransform(floor_frame_, base_frame_, tf2::TimePointZero).transform.translation.z;
    } catch (const tf2::TransformException &) {
      floor_z = -arm_base_height_;
      RCLCPP_WARN(get_logger(), "place: no TF %s -> %s, arm_base_height %.3f m", floor_frame_.c_str(),
        base_frame_.c_str(), arm_base_height_);
    }
    const double surface = has_z ? arm.point.z : floor_z + place_surface_height_;
    const Eigen::Vector3d tip(arm.point.x, arm.point.y, surface + puck_half_height_ + grasp_z_offset_ + place_clearance_);
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    pitch_ = place_pitch_;
    const Eigen::Vector3d dir = ArmIK::approachDirection(std::atan2(tip.y(), tip.x()), pitch_);
    const Eigen::Vector3d pre = tip - dir * place_approach_;
    const Eigen::Vector3d back = tip - dir * place_retreat_;
    const auto q_pre = solveArm(pre, pitch_, *current);
    if (!q_pre || !solveArm(tip, pitch_, *q_pre) || !solveArm(back, pitch_, *q_pre)) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "place: IK no solution for cell (%.3f, %.3f, %.3f) in %s, J1 distance %.3f m",
        tip.x(), tip.y(), tip.z(), base_frame_.c_str(), std::hypot(tip.x(), tip.y()));
      finishSkill(false, buf);
      return;
    }
    RCLCPP_INFO(get_logger(), "place: tip (%.3f, %.3f, %.3f) in %s", tip.x(), tip.y(), tip.z(), base_frame_.c_str());
    sendArm(*q_pre, trajDuration(*current, *q_pre, min_duration_),
      [this, pre, tip, back](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "place approach: " + msg);
          return;
        }
        tool_pos_ = pre;
        moveTool(tip, [this, back]() {
            moveGripper(gripper_open_, [this, back](bool gok, const std::string & gmsg) {
                if (!gok) {
                  finishSkill(false, "place release: " + gmsg);
                  return;
                }
                moveTool(back, [this]() {toTransit("placed");});
              });
          });
      });
  }

  // 다음 팔레트 꺼내기가 같은 자리에서 시작하게 이동 자세 (팔레트 궤적 transit q_front_tilt) 로
  void toTransit(const std::string & message)
  {
    const auto current = currentJoints();
    if (transit_tilt_.size() != 5 || !current) {
      finishSkill(true, message);
      return;
    }
    std::vector<double> q(transit_tilt_.begin(), transit_tilt_.end());
    q.push_back(sixthValue(*current));
    if (!withinLimits(q)) {
      finishSkill(true, message);
      return;
    }
    sendArm(q, trajDuration(*current, q, min_duration_), [this, message](bool ok, const std::string & msg) {
        finishSkill(ok, ok ? message : "transit: " + msg);
      });
  }

  // ------------------------------------------------------------------ 멤버
  // 파라미터
  std::vector<std::string> joint_names_;
  std::vector<double> lower_, upper_;
  double max_joint_vel_{0.5}, min_duration_{1.0}, joint_state_timeout_{1.0};
  std::unique_ptr<ArmIK> ik_;
  std::vector<double> approach_pitches_;
  bool allow_elbow_down_{false}, use_object_yaw_{false};
  double wrist_roll_default_{0.0}, gripper_roll_default_{0.0};
  std::map<std::string, std::vector<double>> poses_;
  GripperMode gripper_mode_{GripperMode::ARM_JOINT};
  std::string gripper_mode_name_{"arm_joint"};
  std::string gripper_joint_;
  double gripper_open_{-0.764}, gripper_close_{0.923}, gripper_effort_{0.0};
  double gripper_move_time_{1.0}, gripper_settle_{0.4}, grasp_margin_{0.15};
  bool gripper_fail_on_empty_{true};
  double toppick_pitch_{-M_PI_4}, toppick_approach_{0.06}, grasp_z_offset_{0.015};
  std::string pallet_file_;
  double pallet_speed_{0.8}, pallet_row3_hold_speed_{0.4};
  bool pallet_require_held_{true};
  std::map<std::pair<int, int>, std::vector<PalletSegment>> pallet_;   // (열, 칸) → 구간
  std::vector<double> transit_tilt_;                                   // 팔레트 궤적 이동 자세 (5 관절)
  std::string place_cells_file_, place_cells_frame_, floor_frame_;
  std::map<std::string, std::pair<double, double>> place_cells_;      // "L1/-1,0" → (x, y)
  double place_pitch_{-M_PI_4}, place_surface_height_{0.070}, puck_half_height_{0.030}, place_clearance_{0.002};
  double place_approach_{0.04}, place_retreat_{0.04}, arm_base_height_{0.120};
  std::string base_frame_, fixed_frame_, range_frame_;
  double approach_distance_{0.15}, range_offset_{0.015}, max_final_approach_{0.25};
  double range_mismatch_tol_{0.04};
  double lift_height_{0.05};
  double cartesian_step_{0.01}, cartesian_speed_{0.05}, max_waypoint_joint_step_{0.2};
  double stale_timeout_{0.3};
  double offset_tolerance_{0.01}, object_radius_{0.015};
  bool skill_enabled_{false};
  std::vector<std::string> skill_names_;
  std::string skill_file_;
  double skill_rate_{20.0}, skill_horizon_{1.5}, skill_max_joint_vel_{0.5};
  std::optional<std::vector<double>> skill_last_cmd_;   // 직전 정책 목표 (변화량 제한 기준)
  robot_policy::PolicyLibrary policies_;
  std::set<std::string> skill_ready_;   // 로드 · 형식 확인을 통과한 정책만 (없으면 execute_skill 거절)
  robot_policy::LoadedPolicy * active_policy_{nullptr};
  bool correction_enabled_{true}, correction_required_{true};
  int correction_max_iter_{5};
  double offset_gain_{1.0}, lateral_tol_{0.005}, range_tol_{0.01}, target_range_{0.13};
  double max_step_{0.02}, correction_min_duration_{0.3}, settle_time_{0.2}, wait_timeout_{1.0};

  // ROS 인터페이스
  rclcpp::CallbackGroup::SharedPtr job_cb_group_, sensor_cb_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client_;
  rclcpp_action::Client<GripperCmd>::SharedPtr gripper_client_;
  rclcpp_action::Server<ArmPose>::SharedPtr pose_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<ExecuteSkill>::SharedPtr skill_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jt_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // 센서 캐시 (sensor_cb_group_ 쓰기, job_cb_group_ 읽기)
  std::mutex sensor_mutex_;
  PerceptionResult::ConstSharedPtr last_result_;
  sensor_msgs::msg::Range::ConstSharedPtr last_range_;
  std::map<std::string, double> joint_pos_;
  rclcpp::Time joint_rx_{0, 0, RCL_ROS_TIME};

  // 작업 상태 (job_cb_group_ 전용)
  Job job_{Job::NONE};
  Task task_{Task::NONE};
  uint64_t job_id_{0};
  std::shared_ptr<PoseGoalHandle> pose_gh_;
  std::shared_ptr<PickGoalHandle> pick_gh_;
  std::shared_ptr<SkillGoalHandle> skill_gh_;
  rclcpp::TimerBase::SharedPtr skill_timer_;
  rclcpp::Time skill_start_{0, 0, RCL_ROS_TIME};
  double skill_timeout_{30.0};
  bool grasped_{false}, gripper_stalled_{false};
  std::optional<double> gripper_cmd_;                 // arm_joint 모드의 지금 그리퍼 명령 (6 번째 값)
  std::vector<double> last_sent_;                     // 마지막으로 보낸 6관절 목표
  std::function<void()> gripper_wait_;                // 그리퍼 판정 대기 (onTick 이 settle_time 뒤 부름)
  std::string pose_name_;
  rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr arm_goal_;
  rclcpp_action::ClientGoalHandle<GripperCmd>::SharedPtr gripper_goal_;
  PickStep step_{PickStep::PLAN};
  Eigen::Vector3d tool_pos_{Eigen::Vector3d::Zero()};   // 현재 끝단 목표 위치
  Eigen::Vector3d grasp_pos_{Eigen::Vector3d::Zero()};  // 계획한 파지점
  std::optional<double> final_grasp_dist_;              // 보정 종료 시점 측정 기반 파지점 거리
  double pitch_{0.0};
  double object_yaw_{0.0};
  bool object_yaw_valid_{false};
  int correction_iter_{0};
  rclcpp::Time wait_since_{0, 0, RCL_ROS_TIME};
};

}  // namespace control

RCLCPP_COMPONENTS_REGISTER_NODE(control::ManipulatorNode)
