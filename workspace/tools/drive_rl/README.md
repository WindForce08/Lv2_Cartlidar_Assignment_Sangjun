# drive_rl — waffle_4wd 주행 강화학습 (Isaac Lab 3.0)

Isaac 안에서만 주행을 학습한다 (ROS · Nav2 · 실물 · 장치 쓰기 없음). 계획과 근거는
[`workspace/docs/drive_rl_worklist_2026-11-03.md`](../../docs/drive_rl_worklist_2026-11-03.md).
팀 로봇 설정 `workspace/urdf_and_usd/isaac/` 은 import 해서 읽기만 한다 (고치지 않음).

## 실행

```bash
cd workspace/tools/drive_rl
./run.sh test                 # base_cmd 시험 (Isaac 없이, 수 초)
./run.sh flat 2 10            # 연기 시험 (약 1 분)
./run.sh flat 64 300          # 단계 A 학습 -> logs/rsl_rl/waffle_drive_flat/<시각>/
./run.sh flat_eval            # 최근 정책 평가 -> results/eval_flat_<시각>.json (exported/policy.pt 가 없으면 먼저 만듦)
./run.sh flat_eval_pp         # 같은 목표 집합에서 기준선 -> results/eval_flat_pp.json
./run.sh flat_play            # 창으로 보기 (학습이 끝난 뒤에만, GPU 8 GB)
```

Isaac Lab 은 `~/pa-course/IsaacLab-6.1` (다른 곳이면 `ISAACLAB_DIR=` 또는 `ISAACLAB_PY=`). ROS 를 source 한 터미널이어도 된다.
학습 곡선: `tensorboard --logdir logs/rsl_rl/waffle_drive_flat` (Metrics/success_rate · time_to_goal · goal_distance · vx_mean).

## 미로 경로 학습 (`Waffle-DriveMaze-Direct`)

경기장 (dice_mission) 여러 벌에서 경로 [`course.yaml`](course.yaml) (59 점 · 정지점 10, 출발점으로 돌아오기까지) 를 따라가는 법을 배운다.

```bash
./run.sh course               # 경로 검사 + course_map.png (경로를 고친 뒤)
./run.sh maze                 # 학습: 64 미로, 1000 반복, 창에 미로 4 개 (창을 닫거나 Ctrl+C 로 멈춤, 50 반복마다 저장)
./run.sh maze 64 1000 0       # 창 없이 (약 2 배 빠름)
./run.sh maze_start 1024 300  # 최근 학습을 이어서, 재설정의 25 % 를 출발점에서 (첫 구간 보강, k = 경로 끝까지). 텐서보드 Metrics/start0_passed_b0 = 첫 모서리 통과율
./run.sh maze_start 1024 500 0.3 2026-11-02_20-14-44   # 그 학습 폴더의 마지막 체크포인트에서, 탐색 폭을 0.3 으로 올려 (set_std.py, 사본은 cache/)
./run.sh maze_play            # 최근 학습 결과를 창으로 (미로 4 개, k 가 1 부터라 한 점씩 가고 다시 놓임)
./run.sh maze_full            # 최근 학습 결과로 1 대가 출발점에서 경로 끝까지 (학습 안 함, 끝나면 다시 출발점에서, 회차마다 결과를 터미널에)
./run.sh maze_full 1          # 1 번 점 (A0) 에서 시작
./run.sh maze_full 1 1        # 1 번 점 (A0) 에서 시작해 다음 1 점 (B0) 까지만
```

창에서 보이는 것: 회색 = 벽 · 기둥, 주황 = 퍽 상자, 파랑 = L1 ~ L3, 빨강 = 신호등, 바닥의 보라 점 = 경로, 자홍 점 + 노랑 막대 = 정지점과 바라볼 방향,
떠 있는 공 = 지금 가는 점 (초록 = 지나가는 점, 빨강 = 정지점, 노랑 = 바라볼 쪽). 처음 창을 띄우면 셰이더 컴파일로 몇 분 "응답 없음" 이 뜰 수 있다 → "기다리기".
학습 곡선: `./run.sh tb` (= `tensorboard --logdir logs/rsl_rl/waffle_drive_maze`, 브라우저 http://localhost:6006) — `Curriculum/k` (지날 점 수), `Episode_End/*` (성공 · 벽 · 물체 · 넘어짐 · 시간 초과 비율),
`Metrics/waypoints_reached`, `Episode_Reward/*` (보상 항별 합).

| 항목 | 값 (설정: `waffle_drive/maze_env_cfg.py`) |
|---|---|
| 구간 학습 | 에피소드마다 경로의 무작위 점 s 에서 시작 (± 3 cm, ± 20°), s+1 ~ s+k 를 지나면 성공. k 는 처음 1, 지금 k 로 시작한 에피소드 max(128, 환경 수) 개의 성공률 ≥ 0.8 이면 +1 (끝까지 = 경로 끝) |
| 지나가는 점 | 반경 0.35 m 안 (모서리 자르기 허용, B안). 0.15 m = A안 (`pass_radius`) |
| 정지점 | 5 cm · 10° 안에서 \|vx\| < 0.03 m/s · \|wz\| < 0.1 rad/s 로 0.5 s. 물체 면 10.75 cm 앞이라 제자리 한 바퀴는 안 됨 → 살짝 물러나서 돌아야 함 (실물과 같음) |
| 끝 | 벽 · 물체에 닿음, 넘어짐 (실패) / 한 점에 15 s 넘게 (시간 초과) |
| 행동 · 주기 | 단계 A 와 같음: (vx, wz) × (0.3 m/s, 1.5 rad/s) → base_cmd (diff_drive · 펌웨어 한계), 20 Hz |
| 관측 53 | vx, wz, 중력 (3), 다음 점 (몸체 x, y), 거리, 정지점인지, 바라볼 방향 오차 (sin, cos), 그다음 점 (x, y), 멈춤 유지 비율, vx 명령, 이전 행동 (2), 라이다 36 (10° 마다, / 2 m) |
| 보상 | 다음 점까지 줄어든 거리 ×20, 도착 +10, 정지점 25 cm 안에서 방향 오차 줄어든 만큼 ×3, 스텝 −0.02, 행동 변화² ×−0.05, 벽 · 물체 5 cm 안 −1 × 가까움, 벽 닿음 −10, 물체 닿음 −10, 넘어짐 −10 |
| 학습 | RSL-RL PPO, MLP 128-128, 64 환경 × 32 스텝 |

지도 기반 판정 ([`waffle_drive/maze_geom.py`](waffle_drive/maze_geom.py), 시험 `tests/test_maze_geom.py`): 미로는 보기용으로만 Isaac 에 놓고 (충돌 없음)
부딪힘 · 라이다는 SDF 충돌체 287 개 (모두 상자) 로 계산한다. 차체 = base_link 기준 x −0.2135 ~ 0.1328, y ±0.168 직사각형의 둘레 1 cm 간격 점,
닿음 = 장애물까지 0.5 cm 미만. 라이다 = 로봇 가운데에서 360°, 윗면이 8.2 cm 보다 낮은 것 (퍽 상자 · 받침대 7 cm) 은 안 보임.
단순 추종기 (방향 P) 로 확인 (2026-11-02, 8 환경 800 스텝): 163 에피소드 중 성공 158 · 벽 4 · 물체 1, k 1 → 2.

단순화한 것 ★: 라이다 두 대 (앞 · 뒤 LDS-03) 를 가운데 한 대로 · 잡음 없음, 팔 · 그리퍼가 차체 밖으로 나오는 부분은 안 봄, 바닥 마찰 1.0 고정 (무작위화는 다음),
시뮬 축간 0.16 (실물 확인 전). 학습을 이어서 하면 (`--checkpoint`) k 가 1 부터 다시 시작한다 (커리큘럼 상태는 저장 안 함).

## 단계 A 과제 (`Waffle-DriveFlat-Direct`)

평지에서 1 ~ 2.5 m 떨어진 무작위 목표 (반경 0.15 m) 까지 간다. 시작 방향 무작위, 에피소드 15 s, 넘어지면 (약 53° 이상) 끝.

| 항목 | 값 |
|---|---|
| 로봇 | 팀 light USD (축간 0.16 · 좌우 0.317 m, 바퀴 1.8 N·m · 10.996 rad/s, 마찰 1.0, 팔 drive 자세 고정) |
| 제어 주기 | 20 Hz (물리 120 Hz / 6) = 실물 정책 주기 (`robot_policy/config/policies.yaml` rate_hz) |
| 행동 2 | [-1, 1] × (0.3 m/s, 1.5 rad/s) = `drive_node` skill 한계 → `base_cmd.py` 가 실물 사슬을 흉내 냄 (아래) |
| 관측 11 | 몸체 vx, 요 각속도, 중력 방향 (3), 몸체 좌표 목표 (x, y), 목표 거리, 한계 적용 뒤 vx 명령, 이전 행동 (2) |
| 보상 | 거리 줄어든 만큼 ×20, 도착 +10, 행동 변화² ×−0.05, 스텝 −0.02, 넘어짐 −10 (예전 로봇 D1 과 같음) |
| 학습 | RSL-RL PPO, MLP 64-64 ELU, 관측 정규화, 64 환경 × 32 스텝 (팀 WaffleDrivePPORunnerCfg 와 같음) |

`base_cmd.py` (물리 스텝마다, torch 만 씀 — `tests/test_base_cmd.py`):

```
정책 [-1, 1]² × (0.3, 1.5)            drive_node skill.max_linear · max_angular
→ |vx| ≤ 0.34, |wz| ≤ 2.0, vx 가속 · 감속 ≤ 1.5 m/s²     diff_drive (controllers_4wd.yaml · drive_belt.yaml)
→ 바퀴 ω = (vx ∓ wz · 0.317/2) / 0.033
→ 한 바퀴라도 10.996 rad/s 를 넘으면 좌우를 같은 비율로 줄임      펌웨어 비례 축소 (실물 한계 11.87 보다 시뮬 한계가 작아 시뮬 값 사용)
```
`action_mode="wheels"` (cfg) 는 예전 D1 방식 (행동 = 좌우 바퀴 속도) 으로, 비교용이다.

## 평가 (`eval_drive.py`)

고정 목표 집합 (64 환경 × 8 에피소드 = 512 개, seed 12345) 에서 결정적 행동으로 돌린다. 회차가 바뀌어도 같은 목표로 비교한다.
지표: 성공률, 걸린 시간 평균 · 90 %, 시간 비율 (= 걸린 시간 / (|처음 방향 오차| / 1.5 + 거리 / 0.3)), 넘어짐, 최대 기울기,
에피소드별 최대 전진 가속. 기준선 (`--controller pp`) = wz = 2 × 방향 오차 (±1.5), |오차| < 30° 일 때만 vx = 0.3 cos(오차).

시뮬 스키드 조향에서 알게 된 것 (2026-11-02): 앞으로 가며 주는 작은 회전 명령 (< 약 0.15 rad/s) 은 거의 돌지 않고,
그 위로도 명령의 50 ~ 65 % 만 돈다 (제자리 1.5 rad/s → 약 1.24). 그래서 곡률만 맞추는 추종기는 성공 56.5 % 였다.
실물에서도 같은지는 측정해야 한다 ★ (diff_drive `wheel_separation_multiplier` 로 보정하는 값).

## 회차 기록

| 회차 | 날짜 | 결과 폴더 | 바꾼 것 | 평가 (512 목표) | 비고 |
|---|---|---|---|---|---|
| 기준선 | 2026-11-02 | results/eval_flat_pp.json | 방향 P 기준선 | 성공 100 %, 평균 6.15 s · 90 % 8.2 s, 시간 비율 0.898 | 넘어짐 0 |

## 실물로 옮길 때 (아직 안 함 ★)

- 정책 출력은 [-1, 1] 이다. `drive_node` 는 m/s · rad/s 를 받으므로, ONNX 에 × (0.3, 1.5) 를 붙이거나 robot_policy 에 행동 배율이 필요하다.
- 관측의 "몸체 좌표 목표 · 거리" 는 robot_policy 관측 목록 (scan · odom_twist · object_pose_base 등) 에 없다 → Controller 팀에 요청.
- 실물 값: 축간 (시뮬 0.16 vs 저장소 URDF 0.20), 바닥 마찰, 제자리 회전 미끄럼, 벨트비.
