# waffle_4wd Isaac Sim / Isaac Lab 공유본 (2026-10-29, B안 판 2026-11-02)

`../waffle_4wd_description` (ROS 2 · Gazebo 패키지, B안 상판 팔레트 v3) 와 같은 로봇을 Isaac Sim 6.1 / Isaac Lab 3.0 에서 바로 쓰게 만든 폴더다.
물리값은 2026-10-29 에 하나씩 정했고 (아래 2절), 값 · 근거는 모두 `robot_params.json` 에 있다.
학습 과제 (환경 · 보상) 코드는 들어 있지 않다. 과제에서 이 로봇 · 센서 설정을 불러 쓰면 된다 (4절).

## 1. 빠른 시작 (Linux)

```bash
cd workspace/urdf_and_usd/isaac
./run.sh check                 # 창 없이 물리 · 센서 점검 -> results/ (약 1 분), --full · --loaded 로 다른 판
./run.sh check --cameras       # + D435 앞 · 뒤 · 팔 카메라 그림 (results/img/)
./run.sh window                # Isaac Sim 창으로 로봇 + 상자 장면 (창을 닫으면 끝)
./run.sh maze                  # Isaac Sim 창으로 주사위 경기장 + 로봇 (출발 구역), 로봇을 비추고 재생 (--light 가벼운 판)
```

- Isaac Lab 위치가 `~/pa-course/IsaacLab-6.1` 이 아니면 `ISAACLAB_DIR=/경로/IsaacLab ./run.sh check` 또는 `ISAACLAB_PY=/경로/python ./run.sh check`.
- ROS 2 를 source 한 터미널이어도 된다 (run.sh 가 ROS 환경 변수를 빼고 Isaac 을 실행).
- 압축은 폴더 구조 그대로 풀 것: `isaac/urdf/*.urdf` 가 `../../waffle_4wd_description/meshes/` 를 상대 경로로 쓴다 (USD 는 메시가 안에 들어 있어 USD 만 옮겨도 됨).
- Windows: `run.sh` 대신 Isaac Lab Python 으로 `set PYTHONPATH=<이 폴더>` 뒤 `python sim_check.py` (시험 안 함).

| 파일 | 내용 |
|---|---|
| `usd/light/waffle_4wd_light/waffle_4wd_light.usda` | 가벼운 판: 차체 충돌 = 상자 (팔레트 = 바닥판 상자 + 칸마다 꽉 찬 상자). 주행 · 미로 학습 기본 |
| `usd/full/waffle_4wd_full/waffle_4wd_full.usda` | 정밀 판: 차체 · 팔레트 충돌 = CoACD 볼록 조각 (전체 478). 퍽 집기 · 팔레트 · 벽 가까이 시험 |
| `usd/loaded/waffle_4wd_loaded/waffle_4wd_loaded.usda` | 가벼운 판 + 팔레트 칸마다 퍽 (12개, 원기둥 12 g · 사각 18 g, 겉모양 · 질량만). 퍽 실은 주행 |
| `robot_params.json` | 물리값 (value · 단위 · 근거 status · 출처 source) |
| `waffle_4wd_isaac/robot_cfg.py` | Isaac Lab 관절체 설정 `WAFFLE_4WD_CFG` (가벼운) · `WAFFLE_4WD_FULL_CFG` (정밀) · `WAFFLE_4WD_LOADED_CFG` (퍽 12개), 시작 자세 `pose()` |
| `waffle_4wd_isaac/sensors_cfg.py` | LDS-03 ×2 · TFmini · IMU · D435 ×2 · 팔 카메라 설정, LiDAR 자기 가림 |
| `urdf/waffle_4wd_{light,full,loaded}.urdf`, `urdf/sensors.json` | 변환에 쓴 URDF (gazebo 태그 뺌), 센서 위치표 |
| `convert_urdf.py` | URDF -> USD + `robot_params.json` 값을 USD 에 씀 (`./run.sh convert`) |
| `sim_check.py` | 점검 (`./run.sh check`) |
| `scenes/dice_mission_waffle_{full,light}.usda` | **주사위 경기장 + 로봇 장면** (2026-10-30). Isaac Sim 에서 File → Open 후 Play. 로봇 = 출발 구역 (신호등 앞 0.6 m 사각) 가운데, 신호등을 봄, 팔 주행 자세 |
| `scenes/dice_mission.usda` | 경기장만 (`worlds/dice_mission.sdf` 를 `tools/sdf_to_usd.py` 로 옮긴 것, pxr 필요). 원점 = 왼쪽 아래 기둥, 칸 0.734 m |
| `maze_view.py` | 위 장면을 창으로 열어 로봇을 비추고 재생 (`./run.sh maze`) |
| `tools/make_maze_scene.py` | 위 장면 파일 생성 (기본 출발 구역, `--cell i j` 칸 · `--yaw` 방향 rad 로 바꿈) |
| `tools/make_isaac_urdf.py` | ROS 패키지 xacro -> `urdf/` (ROS 2 가 있는 PC 에서. 패키지를 바꿨을 때만) |
| `results/` | `./run.sh check` 결과 (만들어짐) |

## 2. 정한 값 (2026-10-29)

| 항목 | 값 | 근거 |
|---|---|---|
| 바퀴 모터 한계 | 1.8 N·m, 10.996 rad/s (0.363 m/s) | 받은 URDF (벨트 1:1.5) 그대로. **팀 펌웨어는 직결** (WHEEL_GEAR 1.0, W210 3.0 N·m, 한계 7.91 rad/s) |
| 바퀴 좌우 간격 | 0.317 m | 좌우 바퀴 폭 가운데 사이 (URDF 바퀴 y ±0.1585) = 실물 (팀원 확인) = 팀 `controllers_4wd.yaml`. 바퀴 관절 원점 간격 0.3092 m 는 3.9 mm 안쪽이라 쓰지 않음 |
| 바퀴 속도 게인 | 감쇠 0.2133 N·m·s/rad, armature 0.00222 kg·m² | W210 기본 Velocity P 100 모터 축 환산 0.48 / 1.5² |
| 팔 6관절 (joint1~5, gripper_roll) | 강성 18.88 N·m/rad, 감쇠 0.851, armature 0.01, 4.1 N·m · 4.8 rad/s | 팀 펌웨어 위치 P 800 · I 0 환산 (arm_bench 와 같은 식). I 게인은 시뮬에 없음 -> 실물처럼 처짐 |
| 손가락 2개 | 힘 13.0 N, 강성 6500 N/m, 감쇠 130, 41 mm/s | 펌웨어 쥐는 힘 33 tick -> 모터 0.96 N·m -> 기구 효율 0.5 (추정) |
| 마찰 | 정지 1.0 · 미끄럼 1.0 (로봇 · 바닥) | Gazebo 공유본과 같게 (미측정) |
| 충돌 형상 | 가벼운 판 + 정밀 판 둘 다 | |
| 자기 충돌 | 팔 ↔ 차체 · 바퀴 켬, 팔 몸체끼리 뺌 | 팔 충돌 상자가 실물보다 커서 팔끼리는 관절 한계로 다룸 |
| 센서 | LiDAR · TFmini · IMU 기본, 카메라는 설정만 (켜는 옵션) | 값은 Gazebo `config/sensors.yaml` 과 같음 |
| 시작 자세 | 주행 자세 `drive` (아래 표) | |

시작 자세 (`pose('drive' | 'stow' | 'zero')`, 바퀴 0, 손가락 0):

| 이름 | joint1 | joint2 | joint3 | joint4 | 비고 |
|---|---|---|---|---|---|
| `drive` (기본) | 0 | −1.35 (−77.3°) | 1.1497 (+65.9°) | 0.8094 (+46.4°) | 웹 시뮬레이터 주행 자세에서 J2 를 4.9° 덜 접음 (가벼운 판 기둥 상자에 닿아서). 앞 D435 화면 아래 가운데에 그리퍼가 보임 |
| `stow` | 1.5739 (+90.2°) | −2.0831 | 1.5463 | −1.8653 | 10-26 실물 전원 켤 때 자세 (1안 기준). 2안에서는 윗팔이 차체에 약 6 N 으로 기댐 |
| `zero` | 0 | 0 | 0 | 0 | URDF · Gazebo 와 같음. 팔이 차체 앞으로 0.41 m |

**`stow` ↔ `drive` 는 `zero` 를 거칠 것**: 가벼운 판에서는 바로 가면 J1 이 도는 중에 접힌 팔이 D435 기둥 쪽 상자 (부품 외곽 상자) 에 걸린다 (정밀 판은 괜찮음).

## 3. Isaac Sim 창에서 USD 만 쓸 때

- `usd/light/.../waffle_4wd_light.usda` 를 열거나 장면에 참조로 넣는다. 관절 구동 · 한계 · armature · 자기 충돌 · 마찰 재질 (1.0) 이 USD 에 들어 있다.
- 바퀴 = 속도 구동 (강성 0, 감쇠 0.2133). 바퀴 4개 모두 축이 +y 라 양수 = 앞으로. 팔 · 그리퍼 모터 · 손가락 = 위치 구동.
- 바닥 마찰도 1.0 으로 둘 것 (PhysX 기본 결합 = 평균).
- 루트 몸체 `base_link` 원점은 지면 위 6.75 mm (하판 기준). 처음에 몇 mm 띄워 놓으면 된다.

## 4. Isaac Lab 과제에서 쓸 때

```python
import sys; sys.path.insert(0, "<이 폴더>")          # 또는 PYTHONPATH
from waffle_4wd_isaac.robot_cfg import WAFFLE_4WD_CFG, WAFFLE_4WD_FULL_CFG, CONTACT_MATERIAL, pose, WHEEL_JOINTS_LEFT, WHEEL_JOINTS_RIGHT
from waffle_4wd_isaac import sensors_cfg as S

robot = WAFFLE_4WD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")           # 정밀 판은 WAFFLE_4WD_FULL_CFG
robot.init_state.joint_pos = pose("drive")
# 센서 (InteractiveSceneCfg 안에서). targets = 광선이 맞힐 장면 메쉬 (정규식)
lds_front = S.lds03_cfg("lds03_front", "{ENV_REGEX_NS}/Robot", S.lidar_targets(["/World/ground"]))   # 맞힐 장면 메쉬 prim 정규식을 넣을 것
imu = S.imu_cfg("{ENV_REGEX_NS}/Robot", "imu")
cam = S.d435_cfg("{ENV_REGEX_NS}/Robot", "front", "color")                 # rgb + 깊이. 앱을 --enable_cameras 로
```

- 물리값은 `robot_params.json` 을 읽으므로 실측하면 그 파일만 고치면 된다 (Isaac Lab 은 USD 값을 이 값으로 덮어씀). USD 안 값 (Isaac Sim 창용) 은 `./run.sh convert` 를 다시.
- LiDAR 가 자기 차체에 가리는 것까지 흉내 내려면 로봇을 만든 뒤 `S.spawn_lidar_occluders(<로봇 prim>)` 를 부르고 `lidar_targets(..., robot_prim=<로봇 prim>)` 로 넘긴다 (`sim_check.py` 참고).
- 관절 이름은 팀 Lv2 (origin/dev) 와 같다: `wheel_{left,right}_{front,rear}_joint`, `joint1~5` (joint5 = 손목 롤), `gripper_roll` (그리퍼 개폐 모터, 펌웨어 칸 5), `gripper_left/right_joint` (직선, mimic 없음 -> 같은 명령).

## 5. 센서 (위치는 `urdf/sensors.json`)

| 센서 | 붙는 몸체 · 위치 [m] | 시뮬 값 | 기본 |
|---|---|---|---|
| LDS-03 앞 · 뒤 | base_link (±0.1089, 0, 0.0755), 상판 밑에 뒤집어. 스캔면 지면 위 0.0823 | 400 광선 (0.9°), 0.05 ~ 12 m, 10 Hz, 광선 시작 반경 25 mm | 켬 |
| TFmini Plus | base_link (0.130, 0, 0.0243) +x | 광선 1개, 0.1 ~ 12 m, 100 Hz | 켬 |
| IMU | base_link (0.0095, 0, 0.0202) = 아래층 OpenCR 칩 (`imu_link`, 하나뿐) | 100 Hz | 켬 |
| D435 앞 · 뒤 컬러 + 깊이 | base_link, 컬러 optical frame 그대로 (깊이 = frame 기준, 더할 것 없음) | 640 × 480, 30 Hz, 가로 화각 0.9571 rad, 0.105 ~ 10 m | 끔 |
| D435 IR 좌 · 우 | base_link, infra1 · infra2 optical frame | 640 × 480, 가로 화각 1.3829 rad (rgb 로 흉내, 패턴 투사 없음) | 끔 |
| 팔 카메라 | gripper_roll_link | 1280 × 720, 30 Hz, 가로 화각 1.9721 rad (왜곡 없는 바늘구멍), 0.02 ~ 5 m | 끔 |

## 6. 점검 결과 (2026-10-29 261028 판, RTX 5060 Laptop 8 GB, 로봇 1대, 창 없이)

| 항목 | 가벼운 판 | 정밀 판 |
|---|---|---|
| USD 안 값 = robot_params.json | 같음 (관절 12, 팔끼리 뺀 쌍 28, 마찰 1.0) | 같음 |
| 질량 · 가만히 | 3.8613 kg (10-30 퍽 플레이트 판) — **B안 판은 4.264 kg (loaded 4.444), 아래 6-1** | 같음 |
| 주행 자세 처짐 (J2 · J3 · J4) | 0.007 · 0.047 · 0.015 rad | 같음 |
| 자세 이동 zero → stow → zero → drive | 가장 큰 오차 0.059 rad (영 자세 J2 처짐) | 같음 |
| 직진 0.1 m/s × 3 s | 0.298 m (미끄럼 없을 때 0.300) | 0.298 m |
| 제자리 회전 3 s (바퀴 ±0.1 m/s) | 76 ~ 77° (미끄럼 없을 때 108.4°) | 76° |
| TFmini (앞 상자) | 0.7599 m (계산 0.7599) | 같음 |
| IMU 가만히 | 가속도 (0, 0, 9.809) m/s² | 같음 |
| LDS-03 상자 거리 | 수직 방향 광선이 가리지 않은 상자는 수직 거리와 1 mm 안 (가린 쪽은 비스듬한 광선의 최소 거리) | 같음 |
| LDS-03 차체에 가림 | 앞 139.5°, **뒤 195.3°** | 같음 (가림 상자 공통) |
| 카메라 (--cameras) | D435 앞 깊이 가운데 0.842 m (frame 높이 0.4905 m · 광축 35.57° 아래로 바닥까지 계산 0.843 m), 뒤 · 팔 카메라 정상 | 시험 안 함 |
| 실시간 비율 | 카메라 끔 1.7 ~ 2.1배, 카메라 3대 1.5배 | 약 1.0 ~ 1.2배 |

### 6-1. B안 판 점검 (2026-11-02, 생성기 폴더에서, 자세히는 `../waffle_4wd_description/CHANGES_B.md`)

| 항목 | light | full | loaded |
|---|---|---|---|
| 가만히 base_link 높이 | 6.8 mm, 기울기 0.03° | 같음 | 같음 |
| 직진 0.1 m/s × 3 s | 0.298 m | 0.298 m | 0.298 m |
| 제자리 회전 3 s | 74.0° (미끄럼 없을 때 108.4°) | 74.0° | 73.2° |
| 자세 zero · drive 오차 | 0.059 · 0.047 rad | 같음 | 같음 |
| **자세 stow 오차** | **0.211 rad** (위팔이 팔레트 칸에 걸림) | 0.200 | 0.211 |
| USD 값 = robot_params.json | 같음 | 같음 | 같음 |

- 센서 자리가 바뀌었다 (5절 표는 B안 값으로 고침): TFmini base_link (0.130, 0, 0.0243), IMU 하나 = `imu_link` (아래층 OpenCR 칩, `imu_left/right` 없음).
- LDS-03 차체 가림이 늘었다: 앞 179.1°, 뒤 220.5° (위층 OpenCR 이 스캔면을 지남).

## 7. 알아 둘 점 · 미확정

- **경기장 장면 (2026-10-30).** `scenes/dice_mission_waffle_full.usda` 는 경기장을 바탕 레이어로, 로봇 USD 를 참조로 넣은 것 (물리 장면 · 중력 포함).
  로봇 base_link (0.6936, 0.950, 0.00975) m (B안 판 차체 외곽 x −0.2135 ~ +0.1328), 방향 180° = 출발 구역 start_zone (가운데 (0.734, 0.95), 0.6 m 사각) 가운데에 차체 외곽 가운데를 맞추고
  신호등 (x 0.08) 을 봄 (발제 주석 "(0.734 0.95) 에서 -X 를 봄"). 구역 가장자리까지 127 mm. 3 초 재생 시험: 그 자리에 바로 섬 (높이 6.75 mm).
  `./run.sh maze` 는 Isaac Sim SimulationApp 으로 연다 (Isaac Lab 실행기는 Play 해도 물리를 안 돌려 팔이 영 자세로 보임).
  자리를 바꾸려면 `python3 tools/make_maze_scene.py --cell <i> <j> --yaw <rad>` (USD 를 다시 만들 필요 없음). 로봇 USD 를 다시 convert 해도 장면은 그대로 씀.

- **퍽 보관 = B안 상판 팔레트 v3 (2026-11-02).** 칸 12곳 자리 · 방향 · 칸 바닥 높이는 `../waffle_4wd_description/config/pallet_slots.yaml` (base_link 기준, 칸마다 loaded 판 퍽 이름).
  팔레트는 base_link 에 합쳐져 따로 몸체가 없다. 10-30 판의 퍽 플레이트 (`puck_plate_link`, `puck_plate_check.py`) 는 없어졌다.
  `rl/` 는 위에서 집기 (`Waffle-TopPick-Direct`) + 놓기 고정 궤적 (`rl/pallet_traj.py`, `./run_rl.sh pallet_check`) 으로 바꿈 (`rl/정책_학습기록.md` 4.15 절). 옛 통 (C · P · N) 기준 store · insert 는 쓰지 않음.
  Isaac Lab 3.0 쿼터니언 순서는 (x, y, z, w) - 퍽을 돌려 놓을 때 주의.
- **stow 자세 (B안).** zero → stow 를 모든 관절 같이 움직이면 J1 약 81° 에서 위팔이 팔레트 칸에 걸린다. J1 을 먼저 다 돌린 뒤 접을 것.

- **바퀴: 벨트 vs 직결.** 이 공유본은 받은 URDF 대로 벨트 1:1.5 값이다. 팀 펌웨어 (`config.h`) 는 직결 (2026-10-26 실물 확인 주석) 이라, 실물과 맞추려면 `robot_params.json` 의 바퀴 한계 · 감쇠를 바꿔야 한다.
- **바퀴 간격**은 0.317 m (바퀴 폭 가운데 기준) 로 URDF · 실물 · 팀 제어기가 같다. URDF 바퀴 관절 원점 (y ±0.1546) 은 바퀴 가운데가 아니니 바퀴 간격으로 쓰지 말 것.
- **뒤 LDS-03 이 많이 가린다 (195°, 보이는 각 약 165°)**: D435 기둥 · 뒤쪽 부품 · 옆벽. 앞은 139.5°. 상자 어림이지만 정밀 충돌 메시 단면으로 따로 계산해도 앞 135.9° · 뒤 195.3° 로 거의 같다. LDS-03 앞뒤 x (±108.9 mm) 는 귀 구멍 실측 전 임시.
- LiDAR · TFmini · IMU 에 잡음은 넣지 않았다 (Gazebo `sensors.yaml` 의 잡음 값은 Gazebo 용). 학습 때 관측에 직접 더할 것.
- TFmini 는 광선 1개 (실물 빔 3.6°). 카메라 내부값 · 화각은 대표값 (실물 camera_info 로 바꿀 것), 왜곡 없음.
- 팔: 시뮬 구동기에 I 게인이 없어 처짐이 남는다 (실물도 P 800 · I 0 이면 처짐). 손가락 기구 효율 0.5 는 추정.
- 고정 관절 (센서 frame, Jetson, OpenCR, 배터리, TFmini 브라켓 ...) 은 USD 에서 `base_link` · `gripper_roll_link` 에 합쳐진다. 센서 위치는 `urdf/sensors.json`.
- `./run.sh window` (창) 은 이 PC 에서 띄워 보지 않았다 (창 없는 점검만 함). Windows 는 시험 안 함.
- USD 폴더의 `config.yaml` 에 만든 PC 의 절대 경로가 적혀 있지만 변환 기록일 뿐 동작과 무관하다.
