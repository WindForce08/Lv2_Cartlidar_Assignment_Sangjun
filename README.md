# Lv2_Cartlidar_Assignment

퍽 운반 로봇 (4륜 · 5축 매뉴퓰레이터 · LiDAR · D435 × 2 · Arducam) — Perception · Controller · Firmware. ROS 2 Lyrical · C++17 component.

- 노드 설계 기준: [workspace/docs/robot_nodes.html](workspace/docs/robot_nodes.html) (4륜, XM430 ID 1~4)
- 통신 정의 (동결): [workspace/docs/interfaces.md](workspace/docs/interfaces.md)
- 설계 대비 검토 · 개선: [workspace/docs/review_robot_nodes.md](workspace/docs/review_robot_nodes.md)

## 패키지 (`workspace/ros2_ws/src`)

| 팀 | 패키지 | 내용 | 실행 위치 |
|---|---|---|---|
| 공동 | `robot_interfaces` | 메시지 · 서비스 · 액션, `names.hpp` (토픽 · 액션 이름과 QoS 를 한곳에) | — |
| Perception | `perception` | `perception_node` (D435 × 2 · Arducam · 검출 · 대상 추적 · AprilTag), `tfmini_node` | ⑦ perception_container / ⑧ |
| Controller | `center` | `mission_node` (미션표 · PAUSED 재개) | ⑧ control_container |
| Controller | `control` | `drive_node` (nav2 · 정렬 · skill), `manipulator_node` (IK · 집기 · skill), 공용 `fsm.hpp` | ⑧ control_container |
| Controller | `robot_policy` | 정책 추론 라이브러리 (ONNX Runtime 선택) | ⑧ (drive · manipulator 안) |
| Firmware | `opencr_hardware` | ros2_control SystemInterface (잠금식 E-stop · /diagnostics · 펌웨어 버전 확인), URDF · 컨트롤러 설정 | ① ros2_control_node |
| Firmware | `opencr_firmware` | OpenCR Arduino 스케치 (colcon 제외) | OpenCR |
| 공동 | `bringup` | 전체 실행 launch, ekf 설정 | — |

## 빌드

```bash
cd workspace/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

선택 의존성은 있으면 켜지고, 없으면 경고만 내고 빌드된다: librealsense2 (없으면 D435 검색 모드 거부), TensorRT (없으면 HSV 색 검출), ONNX Runtime (없으면 `skill_enabled:=false`).

펌웨어는 Arduino IDE 로 `opencr_firmware/opencr_firmware.ino` 를 올린다. `opencr_protocol.h` 는 `opencr_hardware/include/.../opencr_protocol.hpp` 와 같아야 한다 (시험이 확인).

## 실행

```bash
source workspace/ros2_ws/install/setup.bash
ros2 launch bringup bringup.launch.py
```

주요 인자: `use_hardware` · `use_teleop` · `use_perception` · `use_ekf` (기본 true), `port` (기본 `/dev/opencr`), `skill_enabled` (기본 false, ONNX 빌드에서만 true).
nav2 · slam_toolbox 는 기존 launch 를 쓰고, nav2 체인의 마지막 출력만 `/cmd_vel_nav` 로 둔다.

운용 순서:
1. 조이스틱을 켠다 (0.5 s 끊기면 E-stop 잠김).
2. 리셋 버튼 — 켤 때는 잠긴 상태로 시작한다.
3. 시작 버튼 (`/mission/start`) — TAG_VIEW → 태그 읽기 → 미션표 실행.
4. 진행 상황: `ros2 topic echo /mission/status`.

E-stop · 진단 오류가 나면 미션이 PAUSED 가 된다. 원인 해결 → 리셋 → 시작 버튼이면 현재 단계부터 다시 한다. 끝내려면 정지 버튼 (`/mission/stop`).

설정: 미션표 `center/config/mission.yaml`, 인식 `perception/config/perception.yaml` (HSV 범위 · 크기 검증 · D435 화이트밸런스), 정책 `robot_policy/config/policies.yaml`.

## 시험

```bash
cd workspace/ros2_ws
colcon test && colcon test-result --verbose
```

53개 (프로토콜 · 루프백 · 펌웨어 버전 불일치 거부 · 시리얼 독점, 미션표 문법, 정렬 · 안전, IK, 추적 · 검출, 정책 입출력).

## 실물에서 맞출 값 (★)

| 대상 | 맞출 값 |
|---|---|
| 접근 · 차체 | `approach.ready_reach`, `base.front_offset`, `target.exclude_min/max` |
| 인식 | HSV 범위 (현장 조명), `d435.white_balance` · `exposure`, `target.size_min_m/max_m` |
| TF · 카메라 | tfmini_link · arm_camera_link TF, Arducam 보정, 태그 크기 |
| 팔 | STOW · READY · PLACE · TAG_VIEW 관절각, 링크 길이 · 관절 한계 (manipulator.yaml · URDF · config.h 세 곳 같게) |
| 펌웨어 | 바퀴 ID · 방향 · 속도 한계, `WHEEL_MODEL` (바퀴 XM430-W210 = 1030 확정) |
| 조이스틱 · 스위치 | 버튼 번호 (`teleop.yaml`, URDF `estop_button` · `reset_button`), 물리 스위치 핀 |
| 그리퍼 | 개폐 액추에이터 (미정) |
