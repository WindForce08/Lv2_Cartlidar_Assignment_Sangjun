# 주행 강화학습 작업 목록 — 2026-11-03 (작성 2026-11-02)

Isaac Lab 에서 **주행만** 가볍게 학습하기 위한 작업 순서. 위에서부터 순서대로 하면 된다.
근거는 2026-11-02 분석이다 (CLAUDE.md "2026-11-02 — 주행만 RL 가능 여부 분석"). ★ = 확인 안 된 것.

## 0. 목표 · 범위 · 규칙

- **내일 최소 목표**: 평지에서 목표점까지 가는 단계 A 를 지금 로봇 (waffle_4wd light) 으로 학습 1 회 하고 평가 숫자를 얻는다 (작업 0 ~ 6).
  시간이 남으면 경기장 단계 B 를 준비한다 (작업 7 ~ 9).
- Isaac 안에서만 한다. ROS · Nav2 · 실물 · 장치 쓰기는 없다.
- 팀 파일은 고치지 않고 import 해서 읽기만 한다: `workspace/urdf_and_usd/isaac/` (2026-10-30 LCY 커밋, 로봇 · 센서 설정 · 경기장 · 매니퓰레이터 RL).
  새 코드는 사용자 영역 **`workspace/tools/drive_rl/`** (arm_bench 옆) 에 둔다.
  팀 `rl/waffle_isaac/` 에 넣으려면 LCY 와 합의가 먼저다 (`__init__.py` · `run_rl.sh` 를 고쳐야 함).
- 커밋 · 푸시는 지시가 있을 때만 한다. 학습 결과 (`logs/`, `*.pt`) 는 커밋하지 않는다.
- 학습 중에는 창을 띄우지 않는다 (GPU 8 GB 부족, 팀 run_rl.sh 메모).

## 1. 시작 전 결정 — 기본값으로 바로 진행 가능

| 항목 | 기본값 (내일 이것으로 시작) | 다른 선택 |
|---|---|---|
| 행동 | **cmd_vel (vx, wz) + diff_drive · 펌웨어 한계 흉내** | 바퀴 속도 직접 (D1 방식). cfg 플래그 `action_mode` 로 둘 다 지원 |
| 로봇 | 팀 light USD 그대로 (축간 0.16, 좌우 0.317, 바퀴 1.8 N·m · 10.996 rad/s, 마찰 1.0) | 실측 뒤 `robot_params.json` → convert (팀 파일이라 LCY 에게 요청) |
| 코드 위치 | `workspace/tools/drive_rl/` | 팀 `urdf_and_usd/isaac/rl/` (합의 필요) |
| 단계 | A (평지 목표점) → B (경기장 경유점 추종) → C (무작위화 · 잡음) | — |

## 2. 만들 파일 목차 (번호 = 작업 순서)

```
workspace/tools/drive_rl/
├── README.md                      (1) 사용법 · 관측/행동 정의 · 학습 회차 기록 표
├── .gitignore                     (1) logs/ results/ cache/ *.pt *.onnx
├── run.sh                         (1) flat | flat_play | flat_eval | arena | arena_play | arena_eval | check_arena | test
├── train.py                       (1) 팀 rl/train.py 와 같은 틀 (waffle_drive 등록 후 isaaclab_rl 진입점)
├── play.py                        (1) train.py 와 같은 내용 (파일 이름이 play 로 시작하면 재생, 팀 방식)
├── waffle_drive/
│   ├── __init__.py                (4) gym.register: Waffle-DriveFlat-Direct (+ 나중에 Waffle-DriveArena-Direct)
│   ├── robot.py                   (2) 팀 waffle_4wd_isaac 를 sys.path 로 불러 다시 내보냄
│   ├── base_cmd.py                (3) cmd_vel → 바퀴 속도 (diff_drive 한계 + 펌웨어 비례 축소), torch 만 씀
│   ├── flat_env_cfg.py            (4) 단계 A 설정
│   ├── flat_env.py                (4) 단계 A 구현 (TeamRobot drive_env.py 를 옮김)
│   ├── arena_map.py               (7) dice_mission.sdf → 점유 격자 → 부풀림 → 목표별 측지 거리장 · 다음 칸 (numpy/scipy, Isaac 없이)
│   ├── arena_env_cfg.py           (9) 단계 B 설정
│   ├── arena_env.py               (9) 단계 B 구현
│   └── agents/
│       ├── __init__.py            (4)
│       └── rsl_rl_ppo_cfg.py      (4) 팀 WaffleDrivePPORunnerCfg 복사 (실험 이름만 바꿈)
├── eval_drive.py                  (6) 고정 목표 집합으로 성공률 · 시간 · 넘어짐 · 충돌, --controller policy|pp
├── check_arena.py                 (8) 공유 경기장 + 환경 간 충돌 거르기 확인 (로봇 2 대 같은 자리, 기둥에 부딪히기)
└── tests/
    ├── test_base_cmd.py           (3) venv python 으로 그냥 실행 (venv 에 pytest 없음, 설치는 승인 필요)
    └── test_arena_map.py          (7) 시스템 python3 + pytest (numpy 1.26 · scipy 1.11 · pytest 7.4 있음)
```

## 3. 작업 순서

### 작업 0 — 사전 확인 (코드 없음)

```bash
nvidia-smi                                                # RTX 5060 8 GB, 다른 Isaac 프로세스 없음
cd workspace/urdf_and_usd/isaac && ./run.sh check         # 약 1 분, results/check_light_drive.json
```
- 완료 기준: 점검 값이 README 6 절과 같다 (직진 0.1 m/s × 3 s → 0.298 m, 제자리 회전 3 s → 76 ~ 77°, 질량 3.8613 kg).
- venv (`~/pa-course/IsaacLab-6.1/.venv`, 11-02 확인): isaaclab 17.0.2 · isaaclab_rl 0.16.3 · isaaclab_physx 6.0.0 · rsl-rl-lib 5.4.1 · warp 1.16 ·
  scipy 1.17 · trimesh 4.11 · onnx 1.22 · onnxruntime 1.26 · tensorboard 2.21. pytest 는 **없음**.

### 작업 1 — 폴더 · 실행기 (`README.md`, `.gitignore`, `run.sh`, `train.py`, `play.py`)

- `run.sh` 는 팀 `urdf_and_usd/isaac/run.sh` 의 틀을 따른다.
  - `LAB=${ISAACLAB_DIR:-$HOME/pa-course/IsaacLab-6.1}`, `PY=$LAB/.venv/bin/python`
  - ROS 환경 변수를 뺀다: `env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION`
  - `PYTHONPATH=$HERE:<repo>/workspace/urdf_and_usd/isaac`, `HEADLESS=1` (play 만 창), `cd "$HERE"` (logs/ 가 여기 생김)
  - 명령:
    ```
    flat [환경 수=64] [반복=300]  → train.py --task Waffle-DriveFlat-Direct --num_envs N --max_iterations M --viz none
    flat_play [16]                → env -u HEADLESS play.py --task ... --checkpoint latest --viz kit
    flat_eval [64]                → eval_drive.py --task flat
    test                          → tests 둘 다
    ```
- `train.py` · `play.py` 는 팀 `rl/train.py` 와 같은 틀이다 (`warp.config.enable_backward=False`, `import waffle_drive`,
  `--rl_library rsl_rl` 을 붙여 `run_train_cli` / `run_play_cli`). 두 파일 내용은 같고, 파일 이름으로 학습 · 재생을 가른다.
- 완료 기준: `./run.sh` 를 인자 없이 실행하면 사용법이 나온다.

### 작업 2 — `waffle_drive/robot.py`

- 팀 `rl/waffle_isaac/robot_cfg.py` 처럼 `<repo>/workspace/urdf_and_usd/isaac` 를 `sys.path` 에 넣고 `waffle_4wd_isaac.robot_cfg` 에서 가져온다.
  - `WAFFLE_4WD_CFG` (light, 주행 기본)
  - `WHEEL_JOINTS_LEFT/RIGHT` (앞 · 뒤 순서)
  - `ARM_JOINTS` (6, gripper_roll 포함) · `FINGER_JOINTS`
  - `WHEEL_RADIUS` 0.033 · `WHEEL_SEPARATION` 0.317 · `WHEEL_SPEED_MAX` 10.995574
  - `CONTACT_MATERIAL` (= 바닥 재질) · `ROOT_HEIGHT`
- 이 파일에 둘 상수: diff_drive 한계 `VX_MAX 0.34` · `WZ_MAX 2.0` · `AX_MAX 1.5` (출처는 5 절), 차체 외곽 `BODY_X (-0.226, 0.1403)` · `BODY_Y 0.1676`
  (팀 `tools/make_maze_scene.py`), 출발 자리 base_link `(0.6936, 0.950)` · yaw π (팀 isaac README 7 절).
- 완료 기준: venv python 에서 `import waffle_drive.robot` 가 된다 (Isaac 앱 없이 import 가 안 되면 env 안에서만 쓰고, 상수는 따로 둔다).

### 작업 3 — `waffle_drive/base_cmd.py` + `tests/test_base_cmd.py`

실물 명령이 지나가는 길 (twist_mux → diff_drive → 호스트 → 펌웨어) 의 한계를 흉내 낸다. torch 만 쓰고 Isaac 은 쓰지 않는다.
```
vx_t = a0.clamp(-1,1) * VX_MAX ;  wz_t = a1.clamp(-1,1) * WZ_MAX          # 행동 → 목표 twist
vx   = vx_prev + (vx_t - vx_prev).clamp(-AX_MAX*dt, AX_MAX*dt)           # diff_drive linear.x 가속 한계 (angular 는 yaml 에 없음)
wz   = wz_t
wl   = (vx - wz*SEP/2) / R ;  wr = (vx + wz*SEP/2) / R                   # SEP 0.317, R 0.033
s    = max(1, max(|wl|,|wr|) / WHEEL_SPEED_MAX) ;  wl /= s ; wr /= s     # 펌웨어 비례 축소 (PR #10, 한 바퀴라도 넘으면 둘 다 같은 비율)
```
- `dt` 는 물리 dt (1/120) 를 쓰고 `_apply_action` (물리 스텝마다) 에서 부른다. 상태 `vx_prev` 는 env 마다 두고 reset 때 0 으로 한다.
- 시험 (venv python 으로 그냥 실행, assert):
  - 직진 a=(1,0), dt 1/120 로 120 스텝 (1 s) → vx 가 스텝마다 0.0125 m/s 씩 올라 28 스텝째 (약 0.23 s) 에 0.34 가 되고 그대로 유지
  - 제자리 a=(0,1) → |wl| = |wr| = min(2.0×0.1585/0.033, 10.996) = 9.61 rad/s
  - a=(1,1) → 비례 축소 뒤 max(|wl|,|wr|) = 10.996, wl/wr 비율은 축소 전과 같음
  - 바퀴 속도 모드 (D1): a × WHEEL_SPEED_MAX 그대로
- 완료 기준: `./run.sh test` 의 base_cmd 시험이 모두 통과한다 (실행 결과를 보고 나서만 통과라고 적는다).

### 작업 4 — 단계 A 환경 (`flat_env_cfg.py`, `flat_env.py`, `__init__.py`, `agents/`)

- 틀: `~/GIT/FreeCAD/TeamRobot/IsaacLab/teamrobot_isaac/drive_env.py` · `drive_env_cfg.py` (6륜, Isaac Lab 3.0 API, `~/Desktop/CartRider/TeamRobot_Isaac/` 와 같은 파일).
  D1 (Waffle 로 옮긴 판, 10-28) 의 `sim/isaac/waffle_isaac/drive_env*.py` 는 이 PC 에 없다 → 틀에서 다시 옮긴다.
- 틀에서 바꿀 것:
  1. `TEAMROBOT_CFG` → `WAFFLE_4WD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")`. 팔 · 손가락 고정 = `ARM_JOINTS + FINGER_JOINTS` 를 기본 자세 (drive) 위치 명령으로.
  2. `_apply_action`: `action_mode == "twist"` 면 `base_cmd` 로 (wl, wr) 를 만들어 왼쪽 2 · 오른쪽 2 바퀴에 `set_velocity_index`, `"wheels"` 면 틀 그대로.
  3. 관측 (11) — 실물에서 얻을 수 있는 값만:

     | 관측 | 개수 | 실물에서는 |
     |---|---|---|
     | 몸체 전진 속도 vx (`root_lin_vel_b[:,0]`) | 1 | 바퀴 odom · EKF (단계 C 에서 바퀴 속도로 계산한 값 + 잡음으로 바꿈) |
     | 요 각속도 wz | 1 | IMU |
     | 중력 방향 (`projected_gravity_b`) | 3 | IMU 가속도 |
     | 몸체 좌표 목표 (x, y) · 거리 | 3 | 위치 추정 + 목표 |
     | 한계 적용 뒤 vx 명령 (`base_cmd` 상태) | 1 | 호스트가 아는 값 |
     | 이전 행동 | 2 | — |

     틀에 있던 옆 속도 vy 는 실물에서 못 얻으므로 뺀다.
  4. 보상: D1 그대로 (진행 ×20, 도착 +10, 행동 변화² ×−0.05, 스텝 −0.02, 넘어짐 −10). 처음 회차는 D1 과 비교하려고 보상을 바꾸지 않는다.
  5. 기록 (`extras["log"]`): `Metrics/success_rate`, `Metrics/goal_distance`, `Metrics/time_to_goal` (도착한 에피소드의 걸린 시간 평균), `Metrics/vx_mean`.
- 설정 값 (D1 과 같게): 물리 1/120 · decimation 4 (30 Hz), 에피소드 12 s, 목표 1 ~ 2.5 m, 도착 반경 0.15 m, 시작 yaw 무작위, 64 환경 · 간격 6 m,
  `SimulationCfg(dt=1/120, render_interval=decimation, physics=PhysxCfg(), physics_material=CONTACT_MATERIAL)`.
- `__init__.py`: 팀 `rl/waffle_isaac/__init__.py` 형식을 따른다 (`gym.register(id, entry_point, disable_env_checker=True, kwargs={env_cfg_entry_point, rsl_rl_cfg_entry_point, default_agent: "rsl_rl"})`).
- `agents/rsl_rl_ppo_cfg.py`: 팀 `WaffleDrivePPORunnerCfg` (MLP 64-64 ELU, 관측 정규화, init_std 0.6, lr 1e-3 adaptive, 32 스텝) 를 복사하고
  `experiment_name = "waffle_drive_flat"` 으로 바꾼다. 단계 B 는 `"waffle_drive_arena"` (128-128 로 시작).

### 작업 5 — 연기 시험 → 첫 학습

```bash
cd workspace/tools/drive_rl
./run.sh flat 2 10        # 연기 시험: 오류 없음, 보상이 유한한 수, Metrics 가 찍힘
./run.sh flat 64 300      # D1 (예전 로봇) 은 3 분 13 초 · 반복당 0.66 s. 결과 → logs/rsl_rl/waffle_drive_flat/<시각>/
./run.sh flat_play 16     # 학습이 끝난 뒤 창으로 확인 (exported/policy.pt · policy.onnx 생성 ★ rsl_rl play 가 onnx 도 내보내는지 확인)
```
- 볼 것: tensorboard (`logs/rsl_rl/waffle_drive_flat`) 의 success_rate · time_to_goal · 에피소드 길이.
- 회차 표에 적는다 (README, 형식은 팀 정책_학습기록 2.2 와 같게: 회차 · 날짜 · 결과 폴더 · 바꾼 것 · 결과 · 비고).
- 바퀴 속도 모드 1 회 (`action_mode=wheels`) 도 같은 조건으로 돌려 D1 과 비교한다 (선택).

### 작업 6 — `eval_drive.py`

- 정해진 seed 의 고정 목표 집합 (64 환경 × 8 에피소드 = 512 개) 으로, 회차가 바뀌어도 같은 기준에서 비교한다. 정책은 결정적 (평균 행동) 으로 돌린다.
- 결과 (`results/eval_<과제>_<회차>.json` + 표준 출력):
  - 성공률
  - 걸린 시간 평균 · 90 %
  - 시간 비율 = 걸린 시간 / 이상 시간. 이상 시간 = |처음 방향 오차| / 2.0 + 거리 / 0.34
  - 넘어짐 비율 · 최대 기울기
  - 최대 전진 가속 (1.5 m/s² 를 넘는지)
  - 단계 B 에서는 충돌 비율 · 경로 길이 / 측지 거리
- `--controller pp` 는 같은 base_cmd 를 쓰는 간단한 추종기다 (기준선, 작업 10). 단계 A 에서는 "제자리에서 목표 쪽으로 돌고 직진" 으로 충분하다.
- **완료 기준 (단계 A)**: 성공률 ≥ 95 %, 넘어짐 0, 시간 비율을 기준선과 같은 표로 비교한다.

— 여기까지가 내일 최소 목표 —

### 작업 7 — `waffle_drive/arena_map.py` + `tests/test_arena_map.py` (Isaac 없이)

- 입력: `workspace/urdf_and_usd/waffle_4wd_description_261028/worlds/dice_mission.sdf` (경기장 4.404 × 6.304 m, 원점 = 왼쪽 아래 기둥 중심).
- 처리 (부록 A 의 11-02 코드가 바탕):
  1. 충돌 box · cylinder 를 2 cm 격자로 칠함 (바닥 표시 z < 0.01 제외, ground_plane 제외, 경기장 밖 = 막힘)
  2. `scipy.ndimage.distance_transform_edt` 로 장애물까지 거리
  3. 지나갈 수 있는 칸 = 거리 ≥ 0.1676 + 0.03 (직진 반폭 + 여유). 제자리 회전 칸 = 거리 ≥ 0.2814 + 0.02 (시작 자리용)
  4. 목표 집합 G:
     - 상자 6 · 놓는 자리 3 의 둘레 접근점 (반 크기 + 0.25 m, 15° 간격 중 지나갈 수 있는 것)
     - 무작위 빈 칸 (seed 고정)
  5. 목표마다 8 방향 Dijkstra → 측지 거리장 `D[g]` (ny × nx) 와 "0.25 m 앞 칸" 색인 `NEXT[g]` (거리장을 따라 내려가 0.25 m 간 칸)
  6. `cache/arena_<sdf sha1 앞 8>_<여유값>.npz` 로 저장 (다음 실행은 읽기만)
- 시험 (`test_arena_map.py`, 시스템 python3 + pytest):
  - 출발 (0.734, 0.95) 에서 각 물체까지 최단 경로가 11-02 값과 ±0.05 m 안 (부록 A 표)
  - 지나갈 수 있는 칸이 출발점과 모두 이어짐
  - NEXT 를 따라가면 목표에 도착 (몇 개 무작위 시작점)
  - 목표 집합 크기 · 저장 · 다시 읽기

### 작업 8 — `check_arena.py` (공유 경기장 확인, 학습 전에 한 번)

- 방식: 경기장 USD 하나를 `/World/Arena` 에 놓고 (global prim), 모든 환경의 로봇을 경기장 좌표에 직접 둔다.
  환경마다 경기장을 복제하지 않는다 (충돌체 272 개 × 환경 수가 됨).
  - `sim_utils.UsdFileCfg(usd_path=<isaac>/scenes/dice_mission.usda)` → `/World/Arena`. 이 USD 안에 ground_plane (12 × 12 m 충돌 상자), 경기장 충돌체 272 개, 물리 재질 (μ 1.0), Lights 가 들어 있어 바닥을 따로 만들지 않는다.
  - `cloner.clone_plan_from_env_0(..., global_paths=("/World/Arena",))`, `scene.filter_collisions(global_prim_paths=["/World/Arena"])` → 환경끼리는 충돌하지 않고, 모두 경기장과 충돌.
  - `env_spacing` 은 작게 둔다 (처음 복제 자리가 12 × 12 m 바닥 안에 있게). reset 때 world 좌표로 덮어쓴다.
- 확인 (★ 아직 안 해 본 것):
  1. 로봇 2 대를 같은 자리에 두었을 때 서로 밀지 않는다
  2. 기둥 쪽으로 직진하면 멈춘다 (경기장 충돌이 모든 환경에 걸림)
  3. 상자 (70 mm) 를 밀면 걸린다 (상자는 정적 충돌체)
  4. 64 환경일 때 반복당 시간 · GPU 메모리
- 라이다 (나중, 단계 B2): `MultiMeshRayCaster` 는 Cube 원시 도형을 지원한다 (`isaaclab/utils/mesh.py` PRIMITIVE_MESH_TYPES 확인).
  경기장 USD 의 충돌 prim 은 모두 Cube (purpose guide) 다:
  - 벽판 `arena/link/panel_<n>_collision` 62 개
  - 기둥 `arena/link/post_<n>_{base,shaft,top}_collision` 70 × 3 개
  - 상자 · 받침 `<물체>/link/box_collision`

  `/World/Arena` 에 놓으면 라이다 대상은 `/World/Arena/arena/link/.*_collision` 과 `/World/Arena/(start_box|place)_.*/link/.*collision.*` 이 된다.
  ★ 환경마다 자기 차체 가림 상자만 맞히는지는 따로 확인한다.

### 작업 9 — 단계 B 환경 (`arena_env_cfg.py`, `arena_env.py`)

- 시작: 제자리 회전 칸 중 무작위 (+ 일부는 출발 구역, yaw π). 목표: G 중에서 측지 거리 1.5 ~ 6 m 인 것.
- 관측 (약 20):

  | 관측 | 개수 |
  |---|---|
  | 다음 경유점 5 개 (0.25 m 간격, `NEXT` 를 GPU gather 로 따라감, 몸체 좌표) | 10 |
  | 남은 측지 거리 | 1 |
  | vx · wz · 한계 적용 뒤 vx 명령 | 3 |
  | 중력 방향 | 3 |
  | 이전 행동 | 2 |
  | 지도상 장애물까지 거리 (`distance_transform` 값) | 1 |

  `D` · `NEXT` 는 GPU 텐서다 (G=40 이면 약 20 MB).
- 보상: 측지 거리 줄어든 만큼 ×20, 도착 +10 (★ 나중에 도착 방향 조건 — 집기 자리 방향), 스텝 −0.02, 행동 변화² ×−0.05, 넘어짐 −10, 충돌 −10 + 종료.
- 충돌 판정: `ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/(base_link|wheel_.*)")` 의 힘 수평 성분 > 1 N.
  로봇 spawn 은 `activate_contact_sensors=True` 로 바꾼다 (팀 sidepick_env_cfg 방식). 안 되면 지도 거리 < 반폭 으로 대신한다.
- 에피소드 25 s (5.6 m / 0.34 = 16.5 s + 회전). 128 ~ 256 환경.
- 완료 기준: 평가에서 성공률 ≥ 90 %, 충돌 ≤ 2 %, 시간 비율이 기준선 (작업 10) 보다 낫다.

### 작업 10 — 기준선 추종기 (`eval_drive.py --controller pp`)

- 같은 경유점 · 같은 base_cmd 를 쓴다. 앞 0.3 m 경유점을 보고:
  - |방향 오차| > 60° 면 제자리 회전 (wz = ±2.0)
  - 아니면 vx = 0.34·cos(오차), wz = clamp(2·vx·sin(오차)/0.3, ±2.0)
- 강화학습이 이것보다 빠르지 않으면 실물에는 기준선 (또는 Nav2) 을 쓰는 것이 맞다 — 11-02 분석의 판단 기준.

### 작업 11 — 단계 C: 무작위화 · 잡음 (설계가 확정되기 전에는 가볍게만)

- 바닥 마찰 0.6 ~ 1.2, 질량 ±15 %, 무게중심 ±1 cm, 바퀴 감쇠 · 힘 한계 ±20 %
  (★ Isaac Lab 3.0 의 event / randomize 함수 이름은 확인 필요. 팀 코드에는 예가 없다).
- vx 관측을 바퀴 속도로 계산한 odom 값으로 바꾼다 (미끄럼을 못 보는 실물과 같게) + 잡음. IMU wz 잡음. 행동 지연 0 ~ 1 스텝.
- 축간 0.16 / 0.20 두 USD 섞기 (★ 관절 구조가 같으면 MultiAsset spawn 가능한지 확인).

## 4. 수치 기준 (값 · 출처)

| 값 | 출처 |
|---|---|
| diff_drive: vx ±0.34 m/s, wz ±2.0 rad/s, 가속 ±1.5 m/s² (angular 가속 한계 없음), cmd_vel_timeout 0.5 s, update 100 Hz | `ros2_ws/src/opencr_hardware/config/controllers_4wd.yaml`, `drive_belt.yaml` |
| 바퀴 좌우 0.317 m, 반지름 0.033 m | `drive_belt.yaml`, `robot_params.json` |
| 시뮬 바퀴 한계 10.995574 rad/s (0.363 m/s), 1.8 N·m, 감쇠 0.2133, armature 0.00222 | `urdf_and_usd/isaac/robot_params.json` |
| 펌웨어 바퀴 한계 = 모터 330 raw (7.91 rad/s) × 벨트 1.5 = 바퀴 11.87 rad/s (시뮬보다 큼, ★ 실물 벨트비 확인) | `opencr_firmware/config.h`, CLAUDE.md |
| 제자리 회전 이상 최대 = 10.996 × 0.033 / 0.1585 = 2.29 rad/s (시뮬 미끄럼으로 실제는 약 70 %) | 계산 · isaac README 6 절 |
| 질량 3.8613 kg, 주행 자세 무게중심 높이 0.142 m → 앞 들림 약 4.5 m/s², 제동 약 6.5 m/s² | isaac README · 11-02 계산 |
| 차체 외곽 x −0.226 ~ 0.1403, |y| 0.1676 (팔 빼고), 제자리 회전 반경 0.2814 | `tools/make_maze_scene.py` |
| 출발 base_link (0.6936, 0.950, 0.00975), yaw π | isaac README 7 절 |
| LDS-03 스캔면 높이 0.0823 m > 상자 0.07 m → 상자는 라이다에 안 보임 | isaac README 5 절, SDF |
| D1: 64 환경 × 300 반복 3 분 13 초, 50 반복부터 학습 중 도착 100 % (예전 로봇 261027) | `isaac/rl/정책_학습기록.md` 2 절 |

## 5. Isaac Lab 3.0 API 메모 — 팀 코드 · 틀에서 쓰는 것 (그대로 따라 쓰면 됨)

- 데이터는 warp 배열이라 `.torch` 를 붙인다:
  - `robot.data.root_pos_w.torch`, `heading_w.torch`, `root_lin_vel_b.torch`, `root_ang_vel_b.torch`
  - `projected_gravity_b.torch`, `default_root_pose.torch`, `default_root_vel.torch`, `default_joint_pos.torch`
- 명령:
  - `robot.actuators.target_command.set_velocity_index(value=..., joint_ids=...)`
  - `.set_position_index(...)` (값은 `.contiguous()`)
- reset: `write_root_pose_to_sim_index(root_pose=, env_ids=)` · `write_root_velocity_to_sim_index` · `write_joint_position_to_sim_index` · `write_joint_velocity_to_sim_index`.
- 장면:
  - `cloner.grid_transforms(n, spacing, device=)[0]`
  - `cloner.clone_plan_from_env_0(src, dest, n, device, pos, global_paths=(...))`
  - `cloner.replicate(plan, stage=self.scene.stage)`
  - `if "physx" in self.scene.physics_backend: self.scene.filter_collisions(global_prim_paths=[...])`
  - `self.scene.articulations["robot"] = self.robot`
- 접촉: `ContactSensorCfg` 를 scene cfg 에 둔다. 팀은 `data.normal_force_matrix_w.torch` (N, 몸, 상대, 3) 를 쓴다. ★ 필터 없는 합력 속성 이름은 확인 필요.
- 학습 진입점: `isaaclab_rl.entrypoints.run_train_cli / run_play_cli`.
  - 학습 인자: `--rl_library rsl_rl --task --num_envs --max_iterations --viz none`
  - 재생 인자: `--checkpoint latest --viz kit`

## 6. 확인 필요 ★ (내일 하면서 표시)

1. rsl_rl play 가 `exported/policy.onnx` 도 만드는지 (나중에 Jetson 용)
2. 공유 경기장 + `filter_collisions` 로 환경끼리 충돌이 정말 빠지는지 (작업 8)
3. 접촉 센서 합력 속성 이름 (작업 9), 무작위화 event 함수 (작업 11)
4. 라이다가 환경마다 자기 가림 상자만 맞히는지 (단계 B2)
5. 실물 수치 (설계 확정 뒤): 축간 0.16 vs 0.20, 바닥 마찰, 제자리 회전 미끄럼, 벨트비

## 7. 내일 하지 않는 것

- 실물 배포 (지도 기준 위치 추정, drive_node · ExecuteSkill 경유점 입력, Jetson ONNX Runtime) — 단계 B 결과를 보고 따로 계획한다.
- 팀 파일 수정 (`urdf_and_usd/isaac/*`, `ros2_ws/src/control/*`), 패키지 설치 (pytest 등), 장치 쓰기.

---

## 부록 A — 경기장 격자 계산 (11-02 실행한 코드의 핵심)

원본은 scratchpad 의 `arena_grid.py` 다 (`/tmp/claude-1000/.../scratchpad/arena_grid.py`, 재부팅하면 지워질 수 있어 핵심을 여기 옮겨 둠).
11-02 에 시스템 python3 로 실행해 아래 결과를 얻었다.

```python
RES = 0.02
HALF_W = 0.1676                       # 직진 반폭
ROBOT_R = math.hypot(0.226, 0.1676)   # 제자리 회전 외접 반경 0.2814

def rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y

def pose_of(el):
    t = el.findtext('pose')
    return [float(v) for v in t.split()] if t else [0.0] * 6

world = ET.parse(sdf_path).getroot().find('world')
W, H = 4.404, 6.304
nx, ny = int(round((W + 0.4) / RES)), int(round((H + 0.4) / RES))
ox, oy = -0.2, -0.2
shapes = []
for m in world.iter('model'):
    mp = pose_of(m)
    for link in m.iter('link'):
        lp = pose_of(link)
        for c in link.findall('collision'):
            cp = pose_of(c)
            yaw = mp[5] + lp[5] + cp[5]
            bx, by = rot(mp[5], lp[0], lp[1])
            cx, cy = rot(mp[5] + lp[5], cp[0], cp[1])
            x, y = mp[0] + bx + cx, mp[1] + by + cy
            z = mp[2] + lp[2] + cp[2]
            box, cyl = c.find('.//box/size'), c.find('.//cylinder')
            if box is not None:
                sx, sy, sz = [float(v) for v in box.text.split()]
                if z + sz / 2 < 0.01:          # 바닥 표시 (start_zone 등)
                    continue
                shapes.append(('box', x, y, yaw, sx, sy, m.get('name')))
            elif cyl is not None:
                r = float(cyl.findtext('radius'))
                shapes.append(('cyl', x, y, 0, r, r, m.get('name')))
gx = ox + (np.arange(nx) + 0.5) * RES
gy = oy + (np.arange(ny) + 0.5) * RES
X, Y = np.meshgrid(gx, gy)
occ = np.zeros((ny, nx), bool)
for kind, x, y, yaw, a, b, name in shapes:
    if name == 'ground_plane':
        continue
    dx, dy = X - x, Y - y
    if kind == 'box':
        c, s = math.cos(-yaw), math.sin(-yaw)
        u, v = c * dx - s * dy, s * dx + c * dy
        occ |= (np.abs(u) <= a / 2) & (np.abs(v) <= b / 2)
    else:
        occ |= dx * dx + dy * dy <= a * a
occ |= (X < 0) | (X > W) | (Y < 0) | (Y > H)          # 경기장 밖 = 막힘
dist = distance_transform_edt(~occ) * RES              # scipy.ndimage
free = dist >= HALF_W + 0.03
# 출발점 칸 (row = (y-oy)/RES, col = (x-ox)/RES) 에서 8 방향 Dijkstra (heapq, 대각선 비용 √2) → D[칸] × RES = 경로 길이 [m]
# 물체마다 둘레 (max(sx,sy)/2 + 0.25 m, 15° 간격) 점 중 D 가 가장 작은 곳 = 접근점
```

11-02 결과 (출발 (0.734, 0.95) → 가장 가까운 접근점, 괄호는 물체 중심까지 직선거리). `test_arena_map.py` 의 정답으로 쓴다.
지나갈 수 있는 칸은 모두 출발점과 이어져 있었다. Dijkstra 는 1 초 안에 끝났다.

| 물체 | 경로 [m] | 물체 | 경로 [m] |
|---|---|---|---|
| start_box_1 | 0.38 | start_box_4 | 4.06 |
| place_L2_pedestal | 2.41 | start_box_6 | 4.32 |
| place_L1 | 3.21 | start_box_5 | 4.63 |
| start_box_2 | 3.65 | start_box_3 | 5.09 |
| — | — | place_L3 | 5.60 |

## 부록 B — 주행 틀 (D1) 을 옮길 때 바꿀 점 요약

| 틀 (`teamrobot_isaac/drive_env*.py`) | 바꿀 것 |
|---|---|
| `from .robot_cfg import TEAMROBOT_CFG, ARM_JOINTS, FINGER_JOINTS, FLOOR_MATERIAL, WHEEL_*` | `waffle_drive.robot` 에서 (`WAFFLE_4WD_CFG`, `CONTACT_MATERIAL` → FLOOR) |
| 행동 = 좌우 바퀴 속도 × WHEEL_SPEED_MAX | `action_mode` 로 twist (base_cmd) / wheels |
| 관측 11 (vy 포함) | 작업 4 관측 표의 11 (vy 빼고 한계 적용 뒤 vx 명령 넣음) |
| 지면 `spawn_ground_plane("/World/ground")` | 단계 A 는 그대로, 단계 B 는 `/World/Arena` (경기장 USD 안의 바닥) |
| 기록 success_rate · goal_distance | + time_to_goal · vx_mean |
