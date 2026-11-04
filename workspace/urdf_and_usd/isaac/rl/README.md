# waffle_4wd 강화 학습 실행 (rl/)

Isaac Lab 3.0 (RSL-RL PPO) 과제 모음. 실행기는 `run_rl.sh` 하나이고, 과제 설정 · 보상 · 회차 기록은 [정책_학습기록.md](정책_학습기록.md) 에 있다.
이 문서는 **주행 강화 학습 (주행 · 박스 앞 세우기)** 실행 순서를 정리한다. 팔 과제 (위에서 집기 등) 명령은 `./run_rl.sh` (인자 없이) 도움말 참고.

## 0. 준비 (터미널을 새로 열 때마다)

```bash
cd ~/git/Lv2_Cartlidar_Assignment/workspace/urdf_and_usd/isaac/rl
export ISAACLAB_PY=~/pa-course/IsaacLab-6.1/.venv/bin/python
```

- `ISAACLAB_PY` 를 안 정하면 `Isaac Lab 을 찾지 못했습니다` 로 끝난다 (기본 위치는 Docker 컨테이너 `/workspace/isaaclab`).
- GPU 8 GB (RTX 5060 Laptop) 기준: **학습 중에는 창 (`*_play`) 을 띄우지 말 것**, Isaac 을 두 개 동시에 돌리지 말 것.
- 창 모드 (`*_play`) 를 처음 띄우면 RTX 셰이더 컴파일로 2 ~ 3 분 아무것도 안 보일 수 있다 (멈춘 것 아님).
  진행은 `~/pa-course/IsaacLab-6.1/.venv/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/IsaacLab/3.0/` 의 최근 `kit_*.log` 에서 `RtPso async compilation` 줄로 확인.

## 1. 주행 `Waffle-Drive-Direct` (빈 바닥, 무작위 목표점)

1 ~ 2.5 m 떨어진 무작위 점까지 4륜 스키드 주행. 학습 파이프라인 확인용 (64 대 x 300 반복 약 3 분, 50 반복이면 수렴).

```bash
./run_rl.sh drive 16 5        # 짧은 시험 (오류 없이 도는지)
./run_rl.sh drive 64 300      # 학습 -> logs/rsl_rl/waffle_drive/<시각>/model_*.pt
./run_rl.sh drive_play 16     # 최근 학습을 창으로
```

## 2. 박스 앞 세우기 `Waffle-Dock-Direct` (주사위 경기장)

경기장 (`../scenes/dice_mission.usda`, 원본 `waffle_4wd_description/worlds/dice_mission.sdf`) 안에서, Nav2 가 데려다 주는 stage 근처부터
`ros2_ws/src/mission_planner/config/approach_poses.yaml` 의 final 자세 (앞 끝 - 박스 면 = gap 2 cm) 로 바짝 들어가 멈춘다 = 위에서 집기 정책이 요구하는 자리.

| 항목 | 값 (`waffle_isaac/dock_env_cfg.py`) |
|---|---|
| 목표 | 접근 자세표에서 ok 인 11 곳: B1:S, B2:S, B3:S, B3:N, B4:S, B4:N, B5:N, B6:W, L1, L2, L3 |
| 시작 | final 에서 뒤로 0.10 ~ 0.50 m (`back_range`), 옆 ±6 cm (`lat_range`), 요 ±17° (`yaw_range`) |
| 행동 2 | 왼쪽 · 오른쪽 바퀴 속도, 최대의 60 % (약 0.22 m/s, `wheel_scale`) |
| 관측 9 | 목표의 몸체 좌표 (x, y) · 요 오차 (cos, sin) + AMCL 수준 잡음, 몸체 속도 (x, y), 요 각속도, 이전 행동 2 |
| 성공 | 앞뒤 ±12 mm, 옆 ±25 mm, 요 ±3.4° 안에서 멈춤 (`tol_along`, `tol_lat`, `tol_yaw`, `stop_speed`) |
| 실패 | 앞 끝이 박스 면을 넘음 · 차체 수평 접촉 1 N 이상 · 목표에서 1 m 넘게 벗어남 · 15 s 시간 초과 |

경기장 USD 나 접근 자세표를 다른 파일로 쓰려면 환경 변수 `WAFFLE_ARENA_USD`, `WAFFLE_APPROACH_POSES` 로 바꾼다.

### 2-1. 학습

```bash
./run_rl.sh dock 8 2          # 짧은 시험 (목표 11 곳을 읽고 오류 없이 도는지: "[Dock] 목표 11 곳" 줄)
./run_rl.sh dock              # 학습 (기본 128 대 x 1000 반복, 반복당 약 0.7 ~ 0.9 s)
./run_rl.sh dock 128 600      # 환경 수 · 반복 지정
```

결과: `logs/rsl_rl/waffle_dock/<시각>/model_*.pt` (100 반복마다 저장).

### 2-2. 학습 그래프 (다른 터미널, 선택)

```bash
~/pa-course/IsaacLab-6.1/.venv/bin/python -m tensorboard.main --logdir logs/rsl_rl/waffle_dock
# 브라우저 http://localhost:6006 ("TensorFlow installation not found" 는 무시해도 됨)
```

| 지표 | 뜻 |
|---|---|
| `Episode/success_rate` | 그 스텝에 끝난 대 중 성공 비율 (들쭉날쭉함, 정확한 값은 2-4 평가로) |
| `Episode/collision_rate`, `Episode/body_contact_rate` | 면 넘음 + 차체 접촉 비율, 그중 차체 접촉 |
| `Metrics/err_along_mm`, `err_lat_mm`, `err_yaw_deg` | 모든 대의 현재 오차 평균 (앞뒤 · 옆 · 요) |
| `Train/mean_reward` | 평균 보상 |

### 2-3. 창으로 보기 (학습이 끝난 뒤)

```bash
./run_rl.sh dock_play         # 가장 최근 학습 폴더의 마지막 체크포인트, 8 대, 경기장 0 번을 위에서 봄
```

### 2-4. 평가 (창 없이)

목표 11 곳에 같은 수씩 대를 나눠, 대마다 첫 에피소드 결과를 센다 (시작 분포 · 관측 잡음은 학습과 같음, 행동 = 정책 평균).

```bash
./run_rl.sh dock_eval                       # 가장 최근 학습의 마지막 model_*.pt, 220 대 (목표당 20), 1 ~ 2 분
./run_rl.sh dock_eval 440                   # 대 수 지정 (11 의 배수가 좋음)
./run_rl.sh dock_eval 220 --checkpoint logs/rsl_rl/waffle_dock/<시각>/model_500.pt   # 특정 모델
./run_rl.sh dock_eval 220 --seed 1          # 다른 무작위 시작
```

- 출력: 목표별 `성공 / 면 넘음 / 차체 접촉 / 벗어남 / 시간 초과` 대 수, 성공까지 걸린 시간 (중앙값), 실패한 대의 끝 오차 (앞뒤 / 옆 mm · 요 °, 중앙값).
- 같은 내용이 체크포인트 옆 `eval_<모델 이름>.json` 에 저장된다 (`--out 경로` 로 바꿈).
- 내보낸 JIT 정책 (`exported/policy.pt`) 도 `--checkpoint` 로 줄 수 있다.

예 (2026-11-03, `2026-11-03_16-52-34/model_999.pt`, 220 대): 전체 성공 114/220 (52 %), 실패 103 대는 모두 시간 초과,
대부분 앞뒤 1 cm · 요 1° 안이지만 **옆 오차 4 ~ 5 cm 가 남음** (B5:N · L2 는 옆 약 16 cm). 충돌 3, 벗어남 0.

### 2-5. 바꿔 볼 값 (`waffle_isaac/dock_env_cfg.py` 의 `WaffleDockEnvCfg`)

- 시작 범위: `back_range`, `lat_range`, `yaw_range`
- 보상: `lat_weight` (옆 오차 가중, 3.0), `yaw_weight`, `rew_*`
- 에피소드 길이 `episode_length_s` (15 s), 성공 허용 범위 `tol_*`
- PPO (망 크기 · 반복 수): `waffle_isaac/agents/rsl_rl_ppo_cfg.py` 의 `WaffleDockPPORunnerCfg`

값을 바꿔 다시 학습했으면 2-4 평가 결과와 함께 [정책_학습기록.md](정책_학습기록.md) 에 회차를 남긴다.
