# Codex 작업 인수인계 기록

> 이 문서는 저장소를 다른 컴퓨터에 clone한 뒤 Codex가 작업 맥락을 이어가도록 기록한 요약입니다. 저장소에 커밋한 뒤 이 파일도 함께 push해야 집에서 같은 기록을 볼 수 있습니다.

## 프로젝트와 현재 목표

- 저장소: `Lv2_Cartlidar_Assignment`
- 진행 작업: TeamRobot 듀얼 OpenCR 프로토콜 v4와 ROS 2 host 연동
- 사용자 역할: 펌웨어 담당자이며, 구현을 이해하면서 단계적으로 진행하기를 원함
- 설명 언어와 방식: 한국어 존댓말, 의미 있는 단계마다 무엇을 확인하는지와 안전한 확인 방법을 짧게 설명

## 설계 요약

Jetson이 USB CDC로 OpenCR 두 대에 연결됩니다. USB 포트 번호에 역할을 고정하지 않고, OpenCR의 `STATE.role` 응답으로 역할을 판별합니다.

| 보드 | 빌드 역할 | Dynamixel | 동작 모드 | 상태 슬롯 |
|---|---|---|---|---|
| 바퀴 | `OPENCR_BOARD_ROLE=1`, `ROLE_WHEEL=1` | ID 1–6 | Velocity Mode | 0–5, IMU와 배터리 제공 |
| 팔 | `OPENCR_BOARD_ROLE=2`, `ROLE_ARM=2` | ID 11–16 | Position Mode | 6–11 |

- 프로토콜 v4 프레임: `FF FD | TYPE | LEN | PAYLOAD | CRC16-LE`
- CMD `0x01`: 페이로드 55바이트. 역할, 바퀴 속도 6개, 팔 위치 6개, 그리퍼 위치, flags, seq.
- STATE `0x81`: 페이로드 192바이트. 역할, 12축 position/velocity/effort, IMU, 배터리, 오류, 펌웨어 버전, seq.
- v3는 페이로드 길이 불일치로 v4 파서에서 거부되어야 함.
- 바퀴 방향 기본값은 `config.h`의 `{+1,+1,+1,-1,-1,-1}`이나 실물 검증값이 아님. 방향 검증 전 모터 구동 금지.
- 안전: 명령 타임아웃 200 ms, 오류/역할 검사, E-stop 처리, 토크 활성화 전 설정 검사, 토크 ON 시 Bus Watchdog.

상세 설계는 [듀얼 OpenCR 프로토콜 문서](../workspace/docs/opencr_dual_board_v4.md)에 있습니다.

## 현재 코드 변경 범위

현재 작업 디렉터리에 아래 변경이 있으며 아직 커밋되지 않았습니다. 일부 변경은 이번 검토보다 앞서 진행된 작업입니다. 다음 Codex 세션은 먼저 `git status`와 전체 `git diff`를 확인하고 작업을 이어가야 합니다.

- 펌웨어: `workspace/ros2_ws/src/opencr_firmware/config.h`, `opencr_firmware.ino`, `opencr_protocol.h`
- host: `workspace/ros2_ws/src/opencr_hardware/include/opencr_hardware/`, `src/opencr_hardware.cpp`
- ROS 설정: bringup launch, OpenCR launch, URDF/Xacro, udev 규칙
- 프로토콜/루프백 시험: `workspace/ros2_ws/src/opencr_hardware/test/`
- 문서: `README.md`, `workspace/docs/interfaces.md`, `workspace/docs/review_robot_nodes.md`, `workspace/docs/opencr_dual_board_v4.md`
- 이번 검토 중 추가한 변경: loopback URDF fixture를 6륜으로 맞추고 슬롯 5 명령 assertion을 추가함.

현재 `git status`에는 위 변경과 함께 추적되지 않은 `CLAUDE.md`도 표시되어 있습니다. 그 파일은 임의로 수정하거나 삭제하지 말고 사용자가 만든 변경으로 취급합니다.

## 검토·검증 현황

- 프로토콜 호스트/펌웨어 헤더가 동일함을 `cmp`로 확인함.
- C++ 프로토콜 헤더에 대해 `g++ -fsyntax-only` 통과.
- 수정된 Python launch 파일의 AST 파싱 통과.
- `opencr_hardware.xml`, `package.xml` 파싱 통과.
- `git diff --check` 통과.
- ROS host 패키지 빌드는 시도했으나 `/opt/ros/lyrical`에 `hardware_interface`가 없어 CMake 구성에서 중단됨.
- loopback/gtest는 실행하지 않음. Arduino CLI와 PlatformIO는 사용할 수 있는 것으로 확인되지 않았고 펌웨어 빌드도 검증되지 않음.
- 코드 읽기에서 host가 각 상태의 역할로 링크를 매핑하고, 두 역할과 v4를 요구하며, 링크 이상 때 연결 가능한 보드로 E-stop을 보내려는 흐름을 확인함. 전체 컴파일 및 런타임 검증은 미완료.

## 실물 장비·안전 제약

- 사용자 보고: Jetson과 OpenCR 두 대를 연결했고 Jetson에는 SSH 접속만 해둔 상태.
- 현재 모터 등 주변 하드웨어는 준비되어 있지 않음.
- 지금까지 실물 플래시나 모터 구동은 하지 않음.
- 토크 활성화/모터 구동은 주변 하드웨어와 배선, 비상정지, 방향 설정을 확인하기 전에는 진행하지 않음.
- 초기 실물 확인은 장치 인식과 토크 OFF 상태의 역할별 v4 상태 수신부터 계획.
- v3 비호환 동작과 검증 전 바퀴 방향값은 유지.
- 커밋은 사용자가 명시적으로 요청하기 전에는 하지 않음.

## 다음 진행 순서

1. Jetson에서 OpenCR 두 장치가 보이는지 확인하고 실제 장치 경로를 기록합니다. 포트 번호만으로 역할을 단정하지 않습니다.
2. 저장소의 전체 변경을 검토해 역할별 배열 경계, 포트 오류/중복 역할 처리, 활성화·E-stop 흐름을 확인합니다.
3. Jetson ROS 설치에서 `hardware_interface` 등 의존성을 해결하거나 동일한 ROS 배포판의 빌드 환경을 준비한 뒤 host를 빌드합니다.
4. host loopback 시험을 실행하고, 실패 시 원인을 수정한 뒤 재실행합니다. 6개 바퀴 슬롯과 양쪽 보드 역할을 모두 확인합니다.
5. OpenCR Arduino 빌드 환경을 확인하고 바퀴/팔 이미지를 각각 컴파일합니다. 빌드 성공 뒤에야 토크 OFF 통신 확인용 업로드를 검토합니다.
6. 주변 하드웨어가 마련된 뒤 방향·E-stop 배선·안전 정지부터 점검하고, 그 전에는 모터 명령을 실행하지 않습니다.

## 다음 Codex 세션 시작 시

1. 저장소 루트에서 이 문서를 읽습니다.
2. `git status --short`와 `git diff --`를 확인하고 기존 변경을 보존합니다.
3. 사용자에게 변경된 파일 요약과 현재 가능한 안전한 다음 단계를 한국어로 보고합니다.
4. 빌드/시험 환경에 변화가 있는지 확인하되, 실물 플래시·토크 활성화·모터 구동은 별도 맥락을 확인하고 신중히 다룹니다.
