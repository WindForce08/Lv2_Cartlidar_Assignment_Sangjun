# 듀얼 OpenCR 펌웨어 작업 — 진행 상황 점검 (2026-10-23)

작성: 2026-10-23, Claude Code (읽기 전용 점검). 이 점검에서는 git 작업이나 소스 코드 수정을 하지 않았다.
기준 커밋: `af6defe` (`feature/firmware`). 줄 번호는 이 커밋 기준이며, 이후 바뀌면 함수 이름으로 찾는다.
상세 분석 · 구현 계획 · 결정 사항 원문: [firmware_safety_analysis.md](firmware_safety_analysis.md)

## 요약

이전 세션에서 **1단계(읽기 전용 분석)까지 끝났습니다.** 그 뒤로 펌웨어·호스트 코드는 바뀌지 않았습니다. 2단계 계획으로 넘어가려면 분석 문서 §9 의 결정 사항 D1~D9 에 대한 답이 필요합니다.

## 1. 저장소 상태 (2026-10-23)

- 브랜치는 `feature/firmware` 이고 작업 트리는 깨끗합니다. HEAD 는 `af6defe` 이며 `backup/feature/firmware` 와 같습니다 (앞섬·뒤처짐 0). 팀 원격 `origin` 에는 아직 이 브랜치가 없습니다.

| 커밋 | 내용 |
|---|---|
| `0f00432` | 팀 PR #1 병합. 분석을 시작한 시점의 HEAD |
| `a817e2e` (2026-10-22 20:33) | 듀얼 OpenCR v4 전환 작업 (파일 17개), 분석 문서, `CLAUDE.md`, `opencr_dual_board_v4.md` |
| `af6defe` (2026-10-22 20:39) | `AGENTS.md`, `codex/SESSION_HANDOFF.md` (Codex 인수인계용, 문서만) |

## 2. 지금까지 한 작업

### (가) Claude 작업 이전: 단일 OpenCR v3 → 듀얼 OpenCR v4 전환 (사용자 · Codex)

- 펌웨어: `OPENCR_BOARD_ROLE` 값 (1/2) 에 따라 ID 와 모드를 나눠 빌드합니다. 다른 역할로 온 CMD 는 버리고 `ERR_ROLE_MISMATCH` 를 보고합니다.
- 프로토콜 v4: role 필드를 추가했습니다 (CMD 55 B, STATE 192 B). 펌웨어 · 호스트 헤더 두 사본은 동일합니다.
- 호스트: 후보 포트 2개를 열어 STATE.role 로 보드를 구분합니다. WHEEL · ARM 이 하나씩 있고 둘 다 v4 일 때만 활성화하며, 한쪽 링크가 끊기면 남은 보드에 ESTOP 을 보냅니다.

### (나) Claude 1단계: 안전 정책과 현재 코드 비교 → [firmware_safety_analysis.md](firmware_safety_analysis.md)

- ✅ **이미 구현됨:** 역할별 빌드 분기, 다른 역할 CMD 거부, WHEEL 타임아웃 시 바퀴 0, ARM 타임아웃 시 마지막 목표 유지
- 🔴 **실물에서 위험한 문제:**
  1. HOLD 가 저절로 풀립니다. CMD 가 다시 들어오면 바로 다음 주기에 새 목표가 적용됩니다 (`applySafety()`, [opencr_firmware.ino:264-266](../ros2_ws/src/opencr_firmware/opencr_firmware.ino#L264-L266)).
  2. NaN · Inf · 범위 초과 값을 거부하지 않고 clamp 합니다. 그 결과 관절 한계 위치로 이동 명령이 나갑니다.
  3. SyncRead 에 응답하지 않은 모터는 이전 위치값 (초기값 0) 을 그대로 씁니다. 그래서 토크를 켜는 순간 −π 위치가 목표가 될 수 있습니다.
  4. 모터 HW 오류 폴링에 마스크 버그가 있고, 읽기에 실패하면 정상으로 취급합니다.
  5. 토크를 무조건 끄는 경로가 2곳 있습니다. 부팅 시 `setupMotor()` 의 `torqueOff` ([opencr_firmware.ino:88](../ros2_ws/src/opencr_firmware/opencr_firmware.ino#L88)) 와 호스트 `wait_first_state()` 의 `send_cmd(false)` ([opencr_hardware.cpp:294](../ros2_ws/src/opencr_hardware/src/opencr_hardware.cpp#L294)) 입니다. 어느 쪽이든 팔이 떨어질 수 있습니다.
- ❌ **아직 없는 기능:** 명시적 재활성화 절차, FAULT 래치, 값 검증을 통과한 CMD 에만 타이머 갱신, 루프 진단, STATE seq 검사
- **설정 · 문서 불일치:** 물리 E-stop 이 없는데 `config.h` 가 `ESTOP_PIN_ENABLE 1` 이고, 주석에는 "물리 E-stop 필수" 라고 적혀 있습니다.
- **v4 필드 부족:** 재활성화와 HOLD/FAULT 를 표현할 비트가 모자랍니다. 그래서 선택지 A/B/C 를 정리했고 (권장: B 로 v5 상향, C 로 DIAG 프레임 추가), 분석 문서 §7 에 구현 계획 초안 (`safety_core.h` 신설 등) 도 적어 두었습니다.

## 3. 2026-10-23 재확인 결과

| 항목 | 결과 |
|---|---|
| 분석 내용과 현재 코드 | 핵심 지점을 대조해 보니 그대로 일치합니다 |
| 프로토콜 헤더 두 사본 | `cmp` 결과 동일 |
| g++ 단독 테스트 (Protocol 5, EstopLatch 4) | **9개 모두 통과.** 빌드 출력은 저장소 밖에 두었습니다 (명령은 `CLAUDE.md` "빌드 · 테스트" 참고) |
| colcon 빌드 · 루프백 테스트 | **실행하지 못했습니다.** 이 머신에 ros2_control 과 xacro 가 없습니다 |
| 펌웨어 역할별 컴파일 | **실행하지 못했습니다.** arduino-cli 와 OpenCR 코어가 없습니다 |
| 점검한 머신 환경 | x86_64 Ubuntu 24.04.5, ROS 2 lyrical, `/dev/ttyACM*` 없음. `CLAUDE.md` 에 기록된 회사 노트북 환경과 같습니다 |

## 4. 문서와 실제 상태가 어긋난 부분 (이 점검에서는 수정하지 않음)

- 분석 문서 §2 에는 "HEAD `0f00432`, 미커밋 변경" 이라고 되어 있지만, 그 변경은 이미 `a817e2e` 로 커밋되었습니다. 그래서 **D9 (구현 전 커밋 여부) 는 사실상 처리됐는데**, §9 표에는 아직 "미결" 로 남아 있습니다.
- [codex/SESSION_HANDOFF.md](../../codex/SESSION_HANDOFF.md) 의 "미커밋, CLAUDE.md 미추적" 기록도 지난 내용입니다.
- 같은 인수인계 문서에 "Jetson + OpenCR 2대 연결, SSH 접속만, 모터 없음" 이라는 기록이 있습니다. D8 (어느 머신에 OpenCR 이 연결됐는지) 의 단서인데, Claude 분석 문서에는 반영되어 있지 않습니다.

## 5. 다음 단계: 결정이 필요한 사항

| ID | 질문 | 권장 |
|---|---|---|
| D1 | 재활성화와 HOLD/FAULT 를 어떻게 표현할지: A (기존 필드만 사용) / B (최소한의 비트 추가). B 라면 v4 를 유지할지 v5 로 올릴지 | B + v5 |
| D2 | DIAG 프레임 (0x82) 을 추가할지 | 추가 |
| D3 | WHEEL 타임아웃 뒤 자동으로 재개할지 (현재 동작), 재활성화를 요구할지 | — |
| D4 | 오류별 대응 초안 (분석 문서 §7.3) 과 임계값: 연속 누락 횟수 K, 위치 여유, 연속성 허용오차 (제안 0.05 rad) | — |
| D5 | 토크 OFF 경로 2곳을 고칠지 (호스트 변경 포함) | 수정 |
| D6 | 바퀴 속도 한계 충돌: 펌웨어 한계는 바퀴 약 11.9 rad/s 이고 diff_drive 가 요구하는 최대는 약 19.9 rad/s 입니다. 범위 초과를 거부하게 바꾸면 회전하며 주행하는 명령이 거부됩니다 | — |
| D7 | 도구 설치 승인 (arduino-cli · OpenCR 코어 / ros2_control · xacro) | — |
| D8 | OpenCR 2대가 연결된 머신이 Jetson 이 맞는지 확인 | — |
| D9 | 이미 커밋되었으니 해결된 것으로 처리할지 확인 | — |

D1~D8 에 답이 나오면 그에 맞춰 2단계 구현 계획을 확정합니다. 그때 분석 문서 §2 · §9 와 `CLAUDE.md` 의 "현재 진행 상황" 도 함께 갱신할지 정합니다.
