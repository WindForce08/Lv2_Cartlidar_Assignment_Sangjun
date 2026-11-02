# mission_planner — 주사위 기억 로봇 할당 · 적재 · 경로 계획기

발제 v5 (2026-11-04) 미션용. ROS 없이도 돌아가는 Python 모듈 + 명령 + AMCL 위치 추정 설정.

| 하는 일 | 파일 |
|---|---|
| 경기장 SDF → 정적 점유 지도 2 장 (주행용 `map_nav`, AMCL 용 `map_amcl`) | `mission_planner/arena.py`, `maps/` |
| 접근 자세표 (시작 박스 · L1 ~ L3 · 출발 구역) | `mission_planner/poses.py`, `config/approach_poses.yaml` |
| 신호 · 바코드 → 그룹, 퍽 → 그룹 할당 (전체 탐색, 비교용 순차 선택) | `mission_planner/allocation.py` |
| 팔레트 적재 · 방문 순서 (일반화 TSP, Held-Karp) | `mission_planner/gtsp.py`, `planner.py` |
| 주행 비용 (A* + 시야선 당기기 + 회전) | `mission_planner/pathcost.py` |
| AMCL (map_server x 2 + amcl + lifecycle) | `config/amcl.yaml`, `launch/localization.launch.py` |

## 쓰는 법

```bash
colcon build --packages-select mission_planner && source install/setup.bash
mission_make_maps                                   # worlds/dice_mission.sdf 가 바뀌면 (지도 · 자세표 다시)
mission_plan --problem config/problem_q1_example.yaml --mode per_location --out /tmp/plan   # plan.json · signal.json · allocation.json
mission_plan --problem <문제>.yaml --mode compare   # 두 운영 방식 비용 비교
ros2 launch mission_planner localization.launch.py  # /map (주행) · /map_amcl (AMCL) · amcl
python3 -m pytest test                               # 시험 13 개 (할당 6 건 = 도전 B 형식, GTSP 최적성, 지도 · 자세, 계획)
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

`config/amcl.yaml` 주석 참고. 시험 (`test/amcl_sim.py`: SDF 형상 광선 추적 스캔 + 지도에 없는 박스 위 퍽 12 개 + 오도메트리 눈금 오차 직진 2 % · 회전 6 %, Q1 경로 44 m, 4 배속):

| 설정 | 위치 오차 평균 / 95 % / 최대 | 방향 오차 최대 | 갱신 |
|---|---|---|---|
| **이 설정** (`map_amcl`) | **16 / 24 / 35 mm** | **1.7°** | 1778 |
| 같은 설정 + 박스 넣은 지도 | 17 / 25 / 35 mm | 1.8° | 1778 |
| nav2 기본값 (sigma_hit 0.2, alpha 0.2, update_min_d 0.25 …) | 32 / 69 / 134 mm | 6.0° | 505 |
| 오도메트리만 | 최종 1110 mm | | |

시뮬레이션 결과 (가짜 스캔) — 실물은 벽판 설치 오차 · LDS 잡음 · 바퀴 미끄럼으로 커질 수 있음. 출발 구역에서 시작 자세를 정확히 놓는 것이 전제 (전역 위치 찾기 안 함).

## 남은 것 · 주의

- `placement.csv` 가 나오면 문제 파일로 옮겨 계획을 다시 돌릴 것. 경기장 설치 실측이 발제 좌표와 다르면 SDF (generate_world.py) 를 고치고 `mission_make_maps`.
- L2 가운데 · 먼 줄 기울여 놓기, 박스 6 inline 집기, single_load 의 상자 퍽 기울여 집기 (주행이 퍽 면을 J1 반경 ±3° 로 맞춰야 함) 는 Isaac · 실물 확인 전.
- mission_node 연동 (plan.json 단계 → navigate · approach · pick · place 실행) 은 아직 — 단계 형식은 plan.json `steps` 참고.
