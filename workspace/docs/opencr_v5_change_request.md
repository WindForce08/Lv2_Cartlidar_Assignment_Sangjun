# interfaces.md "USB 패킷" 절 변경 요청 — v3 → v5 (OpenCR 2대 · 팔 HOLD/REARM) (Firmware 팀, 2026-10-26)

받는 사람: Controller 팀장 (interfaces.md 승인). 상태: **승인 대기** — `feature/firmware` 브랜치에만 있고, 승인 전에는 dev/main 에 병합하지 않는다.
제안 내용은 이 브랜치의 `interfaces.md` "USB 패킷" 절에 반영해 두었다 (PR diff 로 검토). 구현 · 시험: `opencr_firmware/`, `opencr_hardware/`
(colcon test 77 건 실패 0, 실제 펌웨어 소스를 PC 에서 돌리는 SIL 포함), 팔 보드 실기: `arm_bench_test_2026-10-26.md`.

## 승인본 (dev, v3) 과 제안 (v5) 비교

| 항목 | 승인본 v3 | 제안 v5 | 바뀌는 이유 |
|---|---|---|---|
| OpenCR | 1대 | **2대** — 주행 보드 (역할 1) · 팔 보드 (역할 2) | 하드웨어 구성 결정: 주행 모터 전용 OpenCR + 팔 전용 OpenCR |
| CMD 0x01 | 54 B: `wheel_vel[6]` · `arm_pos[6]` · `gripper_pos` · `flags` · `seq` | **55 B**: 맨 앞에 `role` 추가 | 받은 보드가 자기 몫인지 확인 (아니면 버리고 0x20) |
| STATE 0x81 | 191 B | **192 B**: 맨 앞에 `role` 추가 | 같은 USB ID 인 두 보드를 호스트가 구별 |
| CMD flags | bit0 TORQUE_ON · bit1 ESTOP · bit2 GRIPPER_VALID | + **bit3 REARM** | 팔 HOLD 해제 (아래) |
| STATE error | 0x01 ~ 0x10 | + **0x20 역할 불일치** · **0x40 HOLD** · **0x80 FAULT** | |
| `PROTOCOL_VERSION` | 3 | **5** | 버전이 다르면 호스트가 활성화를 거부 |
| 바퀴 | 4륜, ID 1~4, 슬롯 0~3 (4 · 5 는 0) | **같음** (주행 보드에서) | — |
| 팔 | ID 11~16 | ID 11~16 (팔 보드에서), 슬롯 6~11 | — |
| 포트 | (1대) | `ports` 에 후보 2개, STATE.role 로 연결 | |
| 부팅 점검 | ping · 모델 · Homing 0, Return Delay 0 | + 팔 모터 위치 한계가 관절 범위를 덮는지 | 실기에서 모터 한계 때문에 목표가 응답 없이 무시됨 |

프레임 (`FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`), 필드 순서 (role 뒤는 v3 와 같음), 다른 팀이 쓰는 ROS 토픽 · Action 은 바뀌지 않는다.
버전이 다른 조합 (v3/v4 호스트 + v5 펌웨어, 또는 반대) 은 활성화되지 않는다 — 안전한 쪽 실패. 호스트와 두 펌웨어를 함께 바꾼다.

### 팔 HOLD / REARM 이 필요한 이유

v3 에서는 명령이 끊기거나 ESTOP 뒤 다시 움직여도 되는지 표현할 방법이 없고, 다시 켜려면 토크를 껐다 켜야 해서 팔 (무거운 그리퍼) 이 떨어진다.
v5 에서 팔은 토크를 유지한 채 HOLD 하고, 운용자 리셋 버튼 → 호스트가 REARM 펄스를 보낼 때만 다시 움직인다. 바퀴는 지금처럼 0 으로 멈춘다.

> 이력: 이 브랜치는 10/22 에 interfaces.md 를 승인 없이 v4 · 6륜으로 고쳤었다 (`a817e2e`). 2026-10-26 주행이 다시 4륜이 되어 바퀴는 승인본과 같아졌고,
> 브랜치의 interfaces.md 는 위 v5 제안으로 바꿨다.

## 결정 요청 — 팔 6 번째 칸 · 모터 ID (실물 확인 결과)

실물 팔은 **허리 · 어깨 · 팔꿈치 · 손목 굽힘 · 손목 롤 + 평행 그리퍼** 이다 (팀 시뮬레이션 `TeamRobot/Sim3D` 의 `teamrobot.urdf` 와 같음).
저장소 URDF · `control/config/manipulator.yaml` 은 `joint5` + `gripper_roll` (둘 다 x 축 롤) 로 되어 있어 실물과 다르다.

| 슬롯 | 지금 이름 | 실물 | 모터 ID |
|---|---|---|---|
| 6 · 7 · 8 | joint1 · 2 · 3 | 같음 | 11 · 12 · 13 |
| 9 | joint4 | 손목 굽힘 | **16** |
| 10 | joint5 | 손목 롤 | **14** |
| 11 | gripper_roll | **그리퍼 개폐 모터** (회전 → 손가락 벌림) | **15** |

- 그리퍼 개폐를 (a) 6 번째 팔 칸 (관절각 rad) 으로 둘지, (b) `gripper_pos` 필드 + `GRIPPER_VALID` 로 옮기고 6 번째 칸을 비울지.
  지금 펌웨어는 벤치용으로 (a) 를 쓰고, 범위를 그리퍼 끝에서 0.1 rad 안쪽으로 묶어 두었다.
- URDF 관절 이름 (`joint5` · `gripper_roll` → 예: `wrist_roll_joint` · 그리퍼) 정리.

## Controller 팀 확인 요청 — dev 의 새 `manipulator.yaml` 과 펌웨어 · URDF

(값이 모두 TODO 인 것은 알고 있음. 실물에 맞출 때 같이 보면 좋은 것)

| 항목 | dev `manipulator.yaml` | 펌웨어 · URDF (`opencr.ros2_control.xacro`) | 영향 |
|---|---|---|---|
| 관절 한계 | J1 ±3.14 · J2 ±1.57 · J3 ±2.5 · J4 ±2.0 · J5 ±3.14 | (2026-10-27) J1 ±π · J2 −2.14~1.6 · J3 −1.6~1.6 · J4 −1.92~2.0 · J5 ±2.8 · 6 번째 −0.7946~0.9541 (펌웨어 = URDF = 호스트 명령 한계) | 펌웨어는 범위 밖 CMD 를 통째로 거부, 호스트는 이 범위로 자름 |
| STOW | [0, 1.2, **−2.3**, 1.1, 0, 0] | J3 최소 −1.6 | 호스트가 J3 를 −1.6 으로 잘라 다른 자세가 됨 |
| 부호 · 영점 | `joint_signs` · `joint_offsets` TODO | 실물에서 URDF + 방향 · 영점 확인 끝 (실기 기록 2장) | IK 의 부호 · 0점이 URDF 와 같아야 함 |
| 그리퍼 | `enabled: true`, `gripper_joint` 0~0.02 m (ParallelGripperCommand) | `gripper_controller` 없음 (`use_gripper` false), 실물 그리퍼는 6 번째 팔 칸 | pick 의 그리퍼 단계가 실패할 수 있음 |
| 시작 자세 | — | 전원 켜기 전 접어 둔 자세 J2 −2.08 · J3 +1.55 · J4 −1.87 rad — 2026-10-27 범위를 넓혀 (A안) 범위 안 | 이 자세에서 바로 토크 ON 가능 |

## 2026-10-27 팔 이식 (개인 시험 폴더 arm_bench → dev 기준 브랜치) — Controller 확인 요청

실물 기록: [arm_bench_test_2026-10-27.md](arm_bench_test_2026-10-27.md). 프로토콜 (v5) 필드는 그대로. 로봇 URDF · 관절 이름은 dev 그대로 (`joint1~5` · `gripper_roll`).

| 바뀐 것 | 내용 |
|---|---|
| 팔 범위 | 위 표 "관절 한계". 대기 자세 (A안) · J1 ±180° (배선 꼬임 미확인 ★) · 그리퍼 다시 결합 (열림 1550 · 닫힘 2650 tick = −0.76 · +0.92 rad) |
| 위치 게인 | 펌웨어가 부팅 때 `config.h ARM_POS_*_GAIN` 을 쓰고 다시 읽음 (지금 공장값 P 800 · I 0 · D 0, 튜닝 예정 ★) |
| 그리퍼 막힘 보호 | 6 번째 칸이 0.3 s 동안 거의 못 움직이면 (물체를 잡음) 펌웨어가 목표를 "막힌 자리 + 33 tick" 으로 고정해 약하게 쥔다. 열기 명령이면 풀림 |
| 쥔 채 정지 | ESTOP · 토크 OFF 요청 HOLD 에서도 쥐는 중인 그리퍼는 그대로 (퍽을 놓치지 않게). 다른 관절은 현재 위치 HOLD |
| 부팅 점검 | 게인 다시 읽기, 그리퍼 모터 PWM Limit ≤ 400 (45 %) — 아니면 FAULT (토크를 켜지 않음) |

**Controller 쪽에서 볼 것 ★**
- **6 번째 칸 (`gripper_roll`) 은 실물로 그리퍼 개폐 모터다.** `manipulator_node` 는 팔 궤적마다 `ik.gripper_roll_default` (0.0) 를 보내는데,
  0.0 rad = 2048 tick 은 반쯤 열린 자리다 → 퍽을 쥔 채 (약 2377 tick) 팔을 움직이면 그리퍼가 여는 쪽으로 명령되어 펌웨어 보호가 풀리고 퍽을 놓칠 수 있다.
  예: 팔 궤적의 6 번째 값은 지금 그리퍼 값 (쥐는 중이면 닫힘 0.92 rad) 을 유지하거나, 그리퍼를 별도 컨트롤러로 분리 (이름 변경은 Firmware 와 같이).
- 그리퍼는 닫힘 (0.92 rad) 을 명령하면 펌웨어가 막힌 자리에서 약하게 쥔다 — 손가락 간격 (m) 이 아니라 모터 각 (rad). 쥐었는지 알리는 STATE 비트는 없다
  (위치가 명령보다 덜 닫히고 전류가 있으면 쥐는 중).
- `manipulator.yaml` 의 관절 한계를 위 범위 안으로 (명령이 이 범위를 넘으면 호스트가 자르고, 펌웨어 범위 밖이면 CMD 전체가 거부됨).
- 팀 규격 (`interfaces.md`) 에는 이 동작 (그리퍼 ESTOP 예외 · 부팅 점검 추가) 을 아직 넣지 않았다 — 팀장 확인 뒤 반영.
