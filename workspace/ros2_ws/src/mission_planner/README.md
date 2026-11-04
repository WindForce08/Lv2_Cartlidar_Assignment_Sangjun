# mission_planner — 주사위 기억 로봇 할당 · 적재 · 경로 계획기

발제 v5 (2026-11-04) 미션용. ROS 없이도 돌아가는 Python 모듈 + 명령. 위치 추정 (AMCL · loc_monitor) 은 `localization` 패키지.

| 하는 일 | 파일 |
|---|---|
| 경기장 SDF → 정적 점유 지도 2 장 (주행용 `map_nav`, AMCL 용 `map_amcl`) | `mission_planner/arena.py`, `maps/` |
| 접근 자세표 (시작 박스 · L1 ~ L3 · 출발 구역) | `mission_planner/poses.py`, `config/approach_poses.yaml` |
| 신호 · 바코드 → 그룹, 퍽 → 그룹 할당 (전체 탐색, 비교용 순차 선택) | `mission_planner/allocation.py` |
| 팔레트 적재 · 방문 순서 (일반화 TSP, Held-Karp) | `mission_planner/gtsp.py`, `planner.py` |
| 주행 비용 (A* + 시야선 당기기 + 회전) | `mission_planner/pathcost.py` |

## 쓰는 법

```bash
colcon build --packages-select mission_planner && source install/setup.bash
mission_make_maps                                   # worlds/dice_mission.sdf 가 바뀌면 (지도 · 자세표 다시)
mission_plan --problem config/problem_q1_example.yaml --mode per_location --out /tmp/plan   # plan.json · signal.json · allocation.json
mission_plan --problem <문제>.yaml --mode compare   # 두 운영 방식 비용 비교
python3 -m pytest test                               # 시험 15 개 (할당 6 건 = 도전 B 형식, GTSP 최적성, 지도 · 자세, 계획)
```

문제 파일 (`config/problem_q1_example.yaml` 형식): `signal` (12 점멸), `barcode` (P2-01 ~ 04), `pucks` (id 0 ~ 11 · color red/green · shape cylinder/box · box 1 ~ 6 · side −1/+1).
`placement.csv` 가 나오면 같은 뜻으로 옮겨 쓴다 (설명용 예제의 색 · 모양은 임의).

## 지도 (문서 좌표)

- 원본 `worlds/dice_mission.sdf` = `urdf_and_usd/waffle_4wd_description/worlds/dice_mission.sdf` 복사본 (발제 좌표 mm / 1000, 원점 = 왼쪽 아래 기둥 중심, +Y 위).
- `map_nav`: 바닥 위 0 ~ 0.45 m 와 겹치는 충돌 형상 전부 (벽판 · 기둥 · 박스 70 mm · 중앙 받침대 · 신호등 · 바코드 받침). 0.025 m 칸.
- `map_amcl`: LiDAR 스캔면 (바닥 위 0.082 m = LDS-03 base_link z 0.0755 + 0.00675) 을 지나는 것만. 박스 · 받침대 (0.07 m) 는 스캔에 안 보여 뺌.

## 접근 자세

- 로봇 외곽 (base_link): 앞 +0.1328 · 뒤 −0.2135 (뒤 카메라 기둥) · 폭 ±0.1676 (바퀴), J1 축 x 0.096. 판정은 격자가 아니라 SDF 원래 형상과 사각형 겹침 (분리축).
- 시작 박스: **두 퍽 (박스 중심 X ±40 mm) 을 잇는 선에 수직인 ±Y 면** (S = 남쪽에서 북쪽 보고, N = 북쪽에서). 두 퍽이 J1 앞 0.159 m · 옆 ±0.04 m (거리 0.164) 로 같아 한 번 서서 둘 다 집음.
  ±Y 가 둘 다 막힌 박스만 ±X 면 (inline, 틈 0.04 → 두 퍽 0.139 / 0.219 m).
  지금 맵: B1 S · B2 S · B3 S/N · B4 S/N · B5 N · **B6 W (inline)** (B6 북쪽은 벽, 남쪽은 B2 와 0.53 m 라 대기 자세가 안 들어감).
- L1 · L2 · L3: 장소 태그 면 (L1 서쪽, L2 받침대 북쪽, L3 남쪽). 칸마다 J1 거리 → 수직 (≤ 0.23) / 45° 기울여 (≤ 0.35, 팀 URDF IK).
  **L2 는 격자 중심이 받침대 북쪽 끝에서 0.21 m 안쪽이라 가운데 줄 (0.27) 과 먼 줄 (0.33) 은 기울여 놓아야 함 (Isaac 미검증)**.
- final = 집기 · 놓기 자세 (앞 끝 − 면 0.02), stage = nav2 목표 (standoff 0.45 → 막히면 0.35 · 0.30), 출발 구역 = 로봇 중심 (0.734, 0.95), −X.

## 계획 (planner)

할당: R = 빨강, G = 초록, 12 개 모두 → Y 에 넣을 빨강 수 = 빨강 − R, 초록 수 = 초록 − G 로 정해짐. 남는 자유 (Y 모양 · ID) 를 전부 나열해 경로 비용 최소를 고름.
비교용 `allocate_greedy` (ID 순 순차 선택) 는 "한 모양 부족" 사례 (cases_b V3) 에서 실패함.

| mode | 내용 | 팔레트 |
|---|---|---|
| `per_location` (기본) | L 마다 그 그룹 퍽 박스들을 돌고 → L 에 놓기 (3 번 왕복) | 한 번에 6 개 이하 → 수직 칸 8 개만 (기울임 칸 안 씀) |
| `single_load` | 12 개 모두 싣고 → L 3 곳 | 3 열 → 2 열 → 1 열로 싣고 1 → 2 → 3 열로 꺼냄 (먼저 배달할 L 을 1 열에), 열마다 3 · 4 번째는 기울임 칸 |

방문 순서 = 일반화 TSP (박스 = 묶음, 접근 면 = 선택지, Held-Karp 정확해). L 순서 · 열 나누기는 전부 나열.
비용 = stage 사이 A* (로봇 반지름 0.19 m 부풀림, 시야선 당김) 길이 + 회전 0.1 m/rad + 들어가고 나오기 2 × (standoff − gap).
놓는 칸 순서 = J1 에서 먼 칸부터. 마지막 L 에서 나머지 L 을 다시 들러 새 관측 (재검증) → 출발 구역.

Q1 예제 (설명용 색 · 모양): per_location 비용 66.5 (주행 44.5 m), single_load 57.5.

## AMCL

실행 · 설정은 `localization` 패키지 (`ros2 launch localization localization.launch.py`, `config/amcl.yaml`, 2026-11-04 이 패키지의 AMCL launch · 설정은 지움 — 둘 다 `/map` 을 내서 함께 띄우면 안 되므로).
아래는 이 패키지가 처음 정한 값의 비교 기록 (지금 팀 설정은 Omni 모델 등 5 개 값이 다름, #91). 시험 (`test/amcl_sim.py`: SDF 형상 광선 추적 스캔 + 지도에 없는 박스 위 퍽 12 개 + 오도메트리 눈금 오차 직진 2 % · 회전 6 %, Q1 경로 44 m, 4 배속):

| 설정 | 위치 오차 평균 / 95 % / 최대 | 방향 오차 최대 | 갱신 |
|---|---|---|---|
| **이 설정** (`map_amcl`) | **16 / 24 / 35 mm** | **1.7°** | 1778 |
| 같은 설정 + 박스 넣은 지도 | 17 / 25 / 35 mm | 1.8° | 1778 |
| nav2 기본값 (sigma_hit 0.2, alpha 0.2, update_min_d 0.25 …) | 32 / 69 / 134 mm | 6.0° | 505 |
| 오도메트리만 | 최종 1110 mm | | |

시뮬레이션 결과 (가짜 스캔) — 실물은 벽판 설치 오차 · LDS 잡음 · 바퀴 미끄럼으로 커질 수 있음. 출발 구역에서 시작 자세를 정확히 놓는 것이 전제 (전역 위치 찾기 안 함).

## mission_node 연동 (plan 모드)

```bash
ros2 launch mission_planner planner.launch.py pucks_file:=<퍽 등록표>.yaml [mode:=per_location] [log_dir:=/tmp/mission_log]
ros2 launch mission mission_bringup.launch.py source:=plan        # mission_plan.yaml 덧씌움 (기본 source:=tag 는 그대로)
ros2 service call /mission/start std_srvs/srv/Trigger
```

흐름: `observe` (팔 TAG_VIEW → READ_SIGNAL | READ_BARCODE → /perception/signal CONFIRMED + /perception/barcode READ)
→ `plan` (/mission_planner/plan, std_srvs/Trigger → 단계 문장) → 계획 단계 실행.
robot_interfaces 는 바꾸지 않음 (동결). 계획 단계 (`mission_planner/mission_lines.py`):

| 단계 | mission_node 동작 |
|---|---|
| `goto x y yaw` | /drive/navigate (지도 좌표) + 팔 STOW |
| `approach 0.13` | 기존 approach (COARSE → READY → FINE, TFmini 하한 0.13) |
| `skill manipulator <이름>` | /manipulator/execute_skill — ★ manipulator 구현 필요. 집기 · 놓기는 모두 집게 45°: `toppick_tilt`, `pallet_load_<열>_<칸>_tilt`, `pallet_unload_<열>_<칸>_tilt` (속도 규칙 #75), `place_<L>_<dx>_<dy>_tilt` (dx, dy = m1 · 0 · p1) |
| `skill drive turn_<p\|m>17` · `turn_0` | /drive/execute_skill — drive_node (#77, 실제 회전 중심 보정 `turn.icr_x`). 상자 퍽을 집기 전 FINE 정렬 방향에서 로봇 중심 둘레로 ±17° 제자리 회전 (J1→퍽 = 박스 축 = 퍽 면, ±3°), 집고 실은 뒤 `turn_0` 으로 되돌림. inline (박스 6) 은 회전 없음 |
| `verify L cx cy z expect9 yshape` | 팔 TAG_VIEW → READ_TAG+SEARCH_FRONT → /perception/check_dice (지도 좌표 칸) → 기록, 틀려도 계속 (stop_on_fail false) |
| `wait 3` | 복귀 뒤 정지 |

기록: 계획기 log_dir 에 signal.json · allocation.json · plan.json · mission_lines.txt, mission_node log_dir 에 mission_log.jsonl (observe · plan · verify · finish).
실패: 점멸 MISMATCH · 미등록 바코드 · 풀 수 없는 문제 = 재시도 없이 FAILED (발제: 오류 기록 후 안전 정지). 그 밖의 단계는 max_retries 재시도.

연동 시험 (`test/mock_robot.py` = 가짜 drive · manipulator · perception): Q1 예제로 /mission/start → DONE (단계 93, 주행 18 · 정렬 30 · drive skill 8 (turn) · manipulator skill 48 · check_dice 3, 26 s),
skill 거절 → 재시도 2 번 뒤 FAILED, 미등록 바코드 → 바로 FAILED.

## 남은 것 · 주의

- `placement.csv` 가 나오면 문제 파일로 옮겨 계획을 다시 돌릴 것. 경기장 설치 실측이 발제 좌표와 다르면 SDF (generate_world.py) 를 고치고 `mission_make_maps`.
- 모든 집기 · 놓기 45° (2026-11-03): 팔레트 12 칸은 Isaac 24/24 (#75). 박스 위 집기 · L 칸 놓기 45° 와 상자 퍽 ±17° 회전 집기는 Isaac · 실물 확인 전.
  J1 거리 범위 0.12 ~ 0.35 m (윗면 70 mm). 박스 ±Y 접근 gap 0.06 (회전 때 외곽-박스 여유 15 mm, 퍽 J1 0.203 m), L1 · L3 gap 0.06 (칸 0.139 ~ 0.266 m), L2 0.02 (0.207 ~ 0.332 m).
  TopPick 다시 학습 필요 (#78): tilt_prob 1.0, 원기둥 퍽 gap 0.06, 회전 뒤 상자 퍽은 옆 퍽이 접선 방향. 회전 skill 은 drive #77.
- per_location 은 팔레트 2 · 1 열 8 칸만 씀 (3 열 놓기는 쥔 구간 0.4 rad/s 라 칸당 약 6 s 더 걸림).
- 45° 로 놓은 상자 퍽은 칸에서 J1→칸 방향으로 돌아 앉음 (L1 · L3 최대 23°, L2 16°). 칸 판정에 퍽 방향이 들어가는지 확인.
- manipulator: 위 skill 이름 (팔레트 싣기 · 꺼내기, 칸 놓기, 기울여 집기) 구현 필요. 팔레트 궤적은 `urdf_and_usd/policy/manipulator/pallet_place_traj.json`.
- approach FINE 은 TFmini 하한 0.13 m 라 범퍼-박스 약 0.13 m 에서 멈춤 → 접근 자세표 final (박스 0.06 · inline 0.04 · L1 L3 0.06 · L2 0.02 m) 까지는 #64 에서 정함.
- observe · verify 팔 자세 (TAG_VIEW) 는 바코드 · 칸을 볼 수 있는지 실물 확인 (mission_plan.yaml observe.arm_pose · verify.arm_pose).
