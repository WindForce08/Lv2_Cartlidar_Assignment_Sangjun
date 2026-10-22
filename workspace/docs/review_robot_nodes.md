# robot_nodes.html 대비 소스 검토 · 개선 (2026-10-22)

> **현재 하드웨어 구성 갱신:** 아래 검토는 이전의 4륜·단일 OpenCR 결정을 기록한 이력이다. 현재 희우님이 확정한 구성은 6륜 모터 전용 OpenCR 1대와 팔 모터 전용 OpenCR 1대이며, 최신 통신 계약은 [opencr_dual_board_v4.md](opencr_dual_board_v4.md)이다. 아래 과거 수치는 현재 설정 기준으로 사용하지 않는다.

> **저장소 통합 뒤 이름 대응 (2026-10-22)**: 이 검토는 `robot_ws` 시절 작성. 지금 패키지는
> `robot_perception` → `perception`, `robot_control` → `control` (drive · manipulator) + `center` (mission),
> `controller_bringup.launch.py` → `bringup/launch/bringup.launch.py`. STATE 패킷은 191 B (`fw_version` 추가).


기준: `robot_nodes.html` (5축 매뉴퓰레이터 · ROS 2 Lyrical · C++ component). 바퀴는 **4륜으로 확정** (문서의 6륜과 다름, 2026-10-22). 이전 소스는 v8 다이어그램 + v1.2 상태 관리였다 (백업: `../robot_ws_backup/src_v8_*.tgz`).

## 1. 무엇이 달랐고 어떻게 맞췄나

| 영역 | robot_nodes.html | 이전 소스 (v8) | 이번 구현 |
|---|---|---|---|
| perception 모드 | 비트 플래그 OFF · SEARCH_FRONT 1 · ARM_ALIGN 2 · READ_TAG 4 · SEARCH_REAR 8 | 단일 값 0/1/2 | 비트 플래그. 켜진 비트의 카메라만 열고, OFF 면 mode=0 result 한 번 (받는 쪽 초기화) |
| object_pose | PoseStamped, base_link 기준, 물체 하나만 추적 | 카메라 링크 기준 Pose | TF 로 base_link 변환. TargetTracker: exclude_box · min_confidence · 가장 가까운 것 · odom 0.3 m gate · 1 s 재선택 |
| AprilTag | READ_TAG → tag_id · tag_pose | 없음 | OpenCV aruco `DICT_APRILTAG_36h11` + solvePnP(IPPE_SQUARE) |
| 촬영 시각 · 300 ms | stamp = 촬영 시각(ROS 시계), 받는 쪽 300 ms 지나면 버림 | 일부만 | D435 global time → ROS 시계, Arducam grab 직후. drive · manipulator · mission 모두 stamp 300 ms 확인 |
| 미션 | 태그 → 미션표(navigate · search · approach · pick · pose · skill), fallback, PAUSED 재개 | 고정 순서 상태머신 | 미션표 문법 해석기(`mission_table.hpp`), 단계별 phase, 태그 30° 회전 탐색, search 45° 회전 · 물체 쪽 spin · 전방 재확인 |
| 접근 | 2단계: STOW · D435 먼 거리(범퍼 기준) → READY → Arducam · TFmini 정밀 | READY 와 정렬 동시 | 2단계. drive 가 goal 뒤 첫 result 의 ARM_ALIGN 비트로 모드를 정하고 고정, 먼 거리는 /scan 전 · 후방 감시 |
| 제자리 회전 | 같은 위치 목표 → nav2 spin | 없음 | drive 가 xy_goal_tolerance(0.25 m) 이내 목표를 spin(target_yaw) 으로 변환 |
| pick | 목표 우선순위 ① 정밀 정렬 ② 새 object_pose ③ odom 기억, 직선 전진, grasped | TFmini 거리 하나 | 세 단계 우선순위, SEARCH_FRONT 동안 odom 기억(OFF 에서 삭제), 마지막 유효 거리로 직선 전진(경유점 4), stalled → grasped |
| 강화학습 | robot_policy (ONNX Runtime CPU) · execute_skill · policies.yaml | 없음 | `robot_policy` 패키지 + drive · manipulator execute_skill. skill.enabled=false 면 거절 → mission fallback |
| E-stop | opencr_hardware 한 곳 · 잠금식 (joy heartbeat 0.5 s · 버튼 · 물리 스위치, 리셋 버튼으로만) | /e_stop 구독만, 팔은 별도 래치 + `/arm/estop_reset` | 플러그인이 판단 · /e_stop 10 Hz 발행. 팔은 /e_stop 을 그대로 따름 (별도 리셋 없앰) |
| 진단 | /diagnostics opencr/link · motors · battery · estop, mission 은 opencr/ 만 | 없음 | 1 Hz + 오류 즉시, mission 은 ERROR → PAUSED, 무수신 timeout(확인 전 0) |
| 배터리 | STATE 에 배터리 전압 | 없음 | STATE 190 B (battery_v 추가), 펌웨어 `getPowerInVoltage()` |
| 출력 토픽 | /cmd_vel_drive (align · skill 공용, twist_mux 50 · 0.25 s) | /cmd_vel_align | /cmd_vel_drive |
| ekf | 전진 속도는 바퀴, 회전은 IMU 만 | 바퀴 vyaw 도 사용 | vx 만 바퀴, yaw rate 는 IMU (4륜 스키드도 같은 이유) |
| 바퀴 | 6륜 XM430-W350 ID 1~6 | 파라미터 (기본 4) | **4륜 확정** — ID 1~4, `wheel_count` 기본 4, 패킷은 6자리 유지(남는 2자리 0) |
| ArmPose result | `success` 만 | `success` · `message` | `success` · `message` 로 확정 — mission status.reason 에 실패 이유가 남음 |

## 2. 문서보다 안정 · 효율을 위해 더하거나 다르게 한 것

1. **E-stop 해제 절차를 하나로.** 이전 소스는 팔에 별도 ESTOP 래치와 `/arm/estop_reset` 이 있어 리셋을 두 번 해야 했다. 문서대로 opencr_hardware 의 잠금식 /e_stop 하나만 따르게 바꿨다. 팔의 HOLD(관절 · 속도 한계, 공구 높이)는 그대로 둔다.
2. **미결 #1 (하드웨어 컴포넌트의 토픽) — 가능.** Lyrical hardware_interface 6.9 는 컴포넌트마다 노드(`get_node()`)를 주고 controller_manager executor 가 돌린다. /joy 구독 · /e_stop · /diagnostics 발행을 그 노드의 콜백 · 타이머에서 하고, 100 Hz read()/write() 와는 atomic 값으로만 주고받는다 (실시간 루프에서 할당 · 발행 없음). safety_node 분리는 필요 없다.
3. **E-stop 은 켤 때 잠긴 상태로 시작.** 전원을 켠 뒤 리셋 버튼을 눌러야 움직인다. 조이스틱 없는 벤치 시험은 `joy_required:=false`.
4. **AprilTag 는 OpenCV 내장 사전.** apriltag 라이브러리를 따로 넣지 않고 OpenCV aruco 의 tag36h11 로 읽는다 (OpenCV 4.6 contrib · 4.7+ objdetect 모두 빌드). 자세는 IPPE_SQUARE.
5. **ARM_ALIGN 검출 해상도 축소 (`arm_cam.detect_width: 640`).** SEARCH_FRONT | ARM_ALIGN 에서 15 Hz 이상을 맞추려고 Arducam 검출만 줄인 영상으로 한다. 오차는 정규화 값이라 해상도와 무관하다. READ_TAG 는 원본 해상도.
6. **미션표를 노드 시작 때 모두 검증.** 모르는 장소 · 자세 · 대상, 숫자 모드, 0.10 m 미만 정밀 거리, skill 안의 skill fallback 은 시작 단계에서 실패시킨다 (경기 중 오타 방지).
7. **pick 재시도 무한 반복 방지.** pick 실패 → 직전 search 부터 다시 할 때, search 가 성공해도 재시도 횟수를 초기화하지 않고 실패했던 단계를 넘어선 뒤에만 초기화한다.
8. **후방 D435 가 없으면 search 는 전방만.** SEARCH_REAR 를 거절당해도 실패로 보지 않고 SEARCH_FRONT 로 계속한다.
9. **phase 전환 뒤 요청 전송.** mission 은 단계 안에서 phase 를 바꿀 때 세대 번호를 올려 이전 요청의 늦은 응답을 버린다 — 취소 · 재시도 · PAUSED 뒤 엉킨 응답으로 단계가 넘어가는 일을 막는다.
10. **직선 전진 값 확인.** 마지막 유효 TFmini 거리로 다시 구한 집기 위치가 원래와 5 cm 넘게 다르면 잘못 잰 값으로 보고 버린다.
11. **팔 · 정밀 정렬 모드 고정.** 정밀 중 값이 끊기면 정지 후 0.5 s 뒤 abort 하고 먼 거리로 바꾸지 않는다 (문서 규칙 그대로, 코드에서 강제).

## 3. 결정 요청 (interfaces.md PR 대상)

| # | 내용 | 제안 |
|---|---|---|
| 1 | **skill 성공 조건이 문서에 없음** — "타임아웃 시 0 속도 후 abort" 만 있어 정해진 시간 동안 하는 주행(slalom 등)도 항상 실패 | policies.yaml 에 `finish_on_timeout: true` (이번 구현에 넣음, 기본 false = 문서대로). 장기적으로 v1.2 의 success · termination 조건을 명세에 |
| 2 | ~~ArmPose result 에 message 없음~~ | **반영 (2026-10-22)**: `string message` 추가. mission 이 pose · pick · align · skill 의 message 를 status.reason 에 붙임 (예: `read_tag 실패: pose TAG_VIEW 실패: 관절 상태 없음`) |
| 3 | **/arm/skill_state · /arm/safety_state** — 문서 토픽표에 없음 (v1.2 의 모니터링 토픽) | 토픽표에 추가 또는 삭제 결정. 지금은 모니터링용으로 남김 |
| 4 | **그리퍼 미정 동안 grasped** — 그리퍼가 없으면 grasped 를 판단할 수 없어 미션이 계속 재시도 | `gripper.assume_grasped_without_gripper: true` (벤치용, 그리퍼 정해지면 false) |
| 5 | ~~바퀴 수~~ | **4륜 확정 (2026-10-22)**: 기본값 4 그대로. robot_nodes.html 의 6륜 표기(하드웨어 표 · ID 1~6 · command velocity ×6)는 고쳐야 함 |

## 4. 검증 (이 PC: x86_64, ROS 2 Lyrical, RealSense · TensorRT · ONNX Runtime · controller_manager 없음)

- `colcon test` **48개 통과**:
  - 패킷 · CRC · 펌웨어 사본 일치 · 가상 시리얼 루프백(ResourceManager 로 플러그인 로드)
  - E-stop 잠금 판단, TFmini, 색 검출, 대상 추적, AprilTag 검출 · 자세
  - 정책 yaml · 관측 배치 · fill · clamp
  - 미션 · 스킬 · 안전 상태머신, 미션표 문법, IK, 정밀 · 먼 거리 정렬, /scan 판정
- **통합 시험** (실제 mission · drive · manipulator + 간이 시뮬레이터: 로봇 자세 적분 · TF · nav2 navigate/spin · arm_controller · 인식 · TFmini · /scan):
  - **정상:** 태그 안 보임 → 30° × 2 회전 → tag 7 → station_A → search(옆 물체 → 29° spin → 전방 확인) → 먼 거리 정렬(범퍼 0.40 m) → READY → 정밀 정렬(TFmini 0.15 m) → skill 거절 → fallback pick(① 정밀 정렬 목표) → home → DONE
  - **E-stop:** approach 중 /e_stop → PAUSED, 잠긴 동안 시작 거부, 해제 뒤 시작 → approach 부터 재개 → DONE
  - **인식 노드:** 비트 조합 수락 · 거부, OFF 처리 확인
  - **⑧ control_container:** launch 로 노드 4개 로드 확인
- **펌웨어:** Arduino 도구가 없어 이번 변경(배터리 · 물리 E-stop)은 보드 컴파일을 못 했다 — 실물에서 확인.

## 5. 남은 일 (★ 실물 · 1주차)

- READY 팔 끝 돌출(`approach.ready_reach`), base_link → 범퍼(`base.front_offset`), exclude_box
- tfmini_link · arm_camera_link TF 실측 (pick ① 정확도), Arducam 보정값, 태그 실제 크기
- STOW · READY · PLACE · TAG_VIEW 관절각 (STOW · TAG_VIEW 는 footprint 안)
- 조이스틱 E-stop · 리셋 · 시작 · 정지 버튼 번호, 물리 스위치 핀
- /diagnostics 발행 확인 뒤 `diagnostics.timeout: 3.0`
- 부하 측정: SEARCH_FRONT | ARM_ALIGN 에서 /perception/result ≥ 15 Hz (tegrastats · ros2 topic hz)
- RL 스트리밍 떨림 → `skill.stream_horizon_s` 조정
