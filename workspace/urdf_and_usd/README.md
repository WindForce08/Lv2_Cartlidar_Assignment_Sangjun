# waffle_4wd (4륜 + RM-X52 팔 + B안 상판 팔레트) URDF · USD

시뮬레이션 (ROS 2 · Gazebo Sim, Isaac Sim 6.1 / Isaac Lab 3.0) 과 강화 학습에 쓰는 로봇 · 경기장 파일 묶음. **B안 판 (2026-11-02)**.

| 폴더 | 쓰는 곳 | 시작 |
|---|---|---|
| `waffle_4wd_description/` | ROS 2 (RViz) · Gazebo Sim 패키지: `urdf/waffle_4wd.urdf` · `waffle_4wd_loaded.urdf` · `.urdf.xacro` (collision:=full/light, pucks:=true/false), 메시, 센서 값, 팔레트 칸 자리, launch. 작업 공간 `src/` 에 넣고 `colcon build` | 그 폴더 `README.md`, 바뀐 것 `CHANGES_B.md` |
| `isaac/` | Isaac Sim / Isaac Lab: USD 3 개 (`usd/light` 가벼운 · `usd/full` 정밀 · `usd/loaded` 가벼운 + 퍽 12개), 경기장 장면 (`scenes/`), 로봇 · 센서 설정 (`waffle_4wd_isaac/`), 물리값 `robot_params.json`, 실행기 `run.sh` | 그 폴더 `README.md`, `./run.sh check` · `./run.sh maze` |
| `isaac/rl/` · `policy/manipulator/` (같은 내용) | 강화 학습 과제 코드 · 정책 · 체크포인트. B안 팔레트: 위에서 집기 `Waffle-TopPick-Direct` + 놓기 고정 궤적 (`pallet_traj.py` · `pallet_place_check.py`) | `정책_학습기록.md` 4.15 절, `run_rl.sh` |

## 주요 파일
| 파일 | 내용 |
|---|---|
| `waffle_4wd_description/urdf/waffle_4wd.urdf` | 로봇 URDF (Gazebo 태그 포함, 빈 팔레트). 메시는 `package://waffle_4wd_description/meshes/...` |
| `waffle_4wd_description/urdf/waffle_4wd_loaded.urdf` | 같은 로봇 + 팔레트 칸마다 퍽 (12개) |
| `waffle_4wd_description/config/pallet_slots.yaml` | 팔레트 칸 12곳 (base_link 기준) |
| `isaac/urdf/waffle_4wd_{light,full,loaded}.urdf` | Isaac 변환용 URDF (gazebo 태그 뺌, 메시 상대 경로) |
| `isaac/usd/<판>/waffle_4wd_<판>/waffle_4wd_<판>.usda` | 로봇 USD (`light` 주행 · 학습, `full` 퍽 집기 · 팔레트, `loaded` 퍽 12개 질량 주행) - `payloads/` 와 함께 옮길 것 |
| `isaac/scenes/dice_mission_waffle_{full,light}.usda` | 주사위 경기장 + 로봇 (출발 구역, 신호등을 봄) |
| `isaac/scenes/dice_mission.usda` | 경기장만 (`isaac/tools/sdf_to_usd.py` 가 `worlds/dice_mission.sdf` 에서 만듦) |

## 이번 판 (B안, 2026-11-02) 에서 바뀐 것
- 패키지 이름 `waffle_4wd_description_261028` → **`waffle_4wd_description`** (날짜 뺌). URDF 파일 이름은 `waffle_4wd*.urdf` 그대로.
- 차체: 증속벨트 v4.2 · 바닥 브라켓 v4 · 하단 전장 (OpenCR 2층, IMU 하나 `/imu/data`). **바퀴 관절 x 앞 0.0925 · 뒤 −0.0675** (회전 중심이 base_link 보다 12.5 mm 앞).
- 퍽 보관: 10-30 판의 퍽 플레이트 (B안 팔레트 앞 판) → **B안 상판 팔레트 v3** (칸 깊이 30, 원점 = 조인트1 축 · 상판 윗면 base_link (0.096, 0, 0.111) m). 팔레트는 base_link 에 합쳐져 있어 `puck_plate_link` 는 없다.
- 전체 질량 3.861 → **4.264 kg** (퍽 12개 4.444 kg). Isaac USD 3판 · Gazebo 점검 통과 (`waffle_4wd_description/CHANGES_B.md`).
- A안 저장소 (원호 레일 · 매거진) 는 **보류** (저장소에 넣지 않음, 생성기 `~/Downloads/Claude outputs/puck_storage_design/시뮬레이션_모델_A안`).

## 주의
- 폴더 구조 그대로 둘 것: `isaac/urdf/*.urdf` 와 `isaac/tools/*.py` 가 `../waffle_4wd_description/` 를 상대 경로로 쓴다 (USD 는 메시가 안에 들어 있어 USD 폴더만 옮겨도 됨).
- 큰 파일: `waffle_4wd_description/meshes/b/body_keep.stl` 31 MB, USD `geometries.usd` 각 24 MB.
- **stow 자세**: zero → stow 로 갈 때 위팔이 팔레트 칸에 걸린다 (Isaac 오차 0.21 rad). J1 을 먼저 다 돌린 뒤 접을 것 (`CHANGES_B.md` 확인할 것 1).
- 강화 학습 (2026-11-04 정리: 팔 = `policy/manipulator/`, 주행 · 도킹 = `isaac/rl/`): 팔레트 칸에는 **위에서 집기**로 (옆에서 쥐면 3열 4칸만). 집기 = `Waffle-TopPick-Direct` (상자를 로봇 앞 0 ~ 5 cm 에,
  `policies/toppick_latest.pt` 실제 과제 86 % = 기울임 92 % · 수직 80 %, `./run_rl.sh toppick_eval`),
  놓기 = 고정 궤적 (바깥 열부터 채움). Isaac 놓기 확인 **24/24**: 뒤 프로파일 기둥 옆 ±147° · ±155.6° 4칸은 **집게를 45° 기울여** 놓음 (퍽은 세운 채, joint5 한계 그대로).
  이 4칸 퍽은 상자에서 집을 때부터 45° 로 쥠 (TopPick 기울임 모드).
  남은 것: 손가락 폭 26 vs 홈 24 mm, 팔 처짐 보정 (호스트 적분), 퍽 · 팔레트 마찰 실측 (`isaac/rl/정책_학습기록.md` 4.15 절).
  옛 통 (C · P · N) 기준 과제 (SidePick · Store · Insert) 는 쓰지 않는다.
