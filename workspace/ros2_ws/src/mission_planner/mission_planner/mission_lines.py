"""plan (planner.plan 결과) -> mission_node 단계 문장 목록 (mission_table.yaml 문법 + plan 모드 단계).

mission_node 가 쓰는 단계 (src/mission/src/mission_node.cpp 머리말):
  goto <x> <y> <yaw>                    지도 자세로 주행 + 팔 STOW (navigate 와 같고 위치 이름 대신 좌표)
  approach <d>                          STOW -> COARSE -> READY -> FINE (TFmini 렌즈 -> 박스 앞면 d, >= 0.13)
  pick_target <x> <y> <z>               /manipulator/pick, target = 지도 좌표 퍽 중심 (grasped 확인)
  skill manipulator <이름> [timeout] [mode=…]   이름 규칙 (manipulator_node, PR #94).
                                        집기 · 놓기는 모두 집게 45° (_tilt):
        target_look:base_footprint:<x>:<y>:<z>   집기 전 팔 카메라로 퍽을 내려다보고 재서 붙잡음 (#82, 45° 집는 중에는 못 봄).
                                        x · y = 회전 뒤 퍽 예상 중심 (poses.puck_after_turn), z = 바닥 위 퍽 중심
        toppick_tilt                    45° 기울여 위에서 집기 (TopPick 기울임 모드). 목표 = 직전 target_look 이 잰 위치
        pallet_load_<열>_<칸>_tilt       쥔 퍽을 팔레트 칸에 놓기 (pallet_place_traj.json 의 칸 궤적, 속도 규칙 #75)
        pallet_unload_<열>_<칸>_tilt     팔레트 칸의 퍽을 집어 들기 (위 궤적의 반대)
        place_<L>_<dx>_<dy>_tilt         쥔 퍽을 장소 L 의 3 x 3 칸 (dx, dy = m1 · 0 · p1, 지도 +Y 가 위) 에 놓기
  skill drive turn_<p|m><deg>          drive_node (#77): FINE 정렬 방향에서 로봇 중심 둘레로 deg 제자리 회전 (p = 왼쪽 +, m = 오른쪽).
        turn_0                          정렬 방향으로 되돌림. 상자 퍽을 45° 로 집기 전 · 후 (퍽 면을 J1→퍽 방향에 맞춤)
  verify <L> <cx> <cy> <z> <expect9> <yellow_shape>   /perception/check_dice (칸 번호 = 위 줄부터 행 우선, 왼쪽 -X 부터)
  wait <s>                              제자리 대기 (복귀 후 3 s 정지)
"""
from __future__ import annotations

from .poses import DICE, FINE_DISTANCE, FINE_GAP, puck_after_turn

PUCK_CENTER_Z = 0.07 + 0.03  # [m] 박스 (70) 위 퍽 (60) 중심, 지도 z
GROUP_CODE = {"R": 1, "G": 2, "Y": 3}  # CheckDice.RED · GREEN · YELLOW
SHAPE_CODE = {"cylinder": 1, "box": 2}  # Puck.SHAPE_CYLINDER · SHAPE_CUBOID


def _n(v):
    return f"{v:.4f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def _idx(v):
    return {-1: "m1", 0: "0", 1: "p1"}[v]


def turn_name(deg):
    """17.06 -> 'turn_p17', -17.06 -> 'turn_m17', 0 -> 'turn_0'."""
    n = round(abs(deg))
    return "turn_0" if n == 0 else f"turn_{'p' if deg > 0 else 'm'}{n}"


def expect_cells(count, group):
    """주사위 count, 그룹 -> CheckDice expect 9 개 (행 우선: 위 줄 (+Y) 부터, 줄 안은 -X 부터)."""
    e = [0] * 9
    for dx, dy in DICE[count]:
        r, c = 1 - dy, dx + 1
        e[r * 3 + c] = GROUP_CODE[group]
    return "".join(str(v) for v in e)


def to_lines(world, plan, fine_distance=0.13, skill_timeout=40.0, return_wait=3.0):
    """-> (문장 목록, 메모 목록)."""
    if fine_distance < 0.13:
        raise ValueError("fine_distance >= 0.13 (mission_node approach 하한)")
    A = world.approaches
    lines, notes = [], []
    cur = "HOME"
    approached = False
    T = _n(float(skill_timeout))
    for s in plan["steps"]:
        t = s["type"]
        if t == "observe":
            continue
        if t == "navigate":
            x, y, yaw = s["stage"]
            lines.append(f"goto {_n(x)} {_n(y)} {_n(yaw)}")
            cur, approached = s["to"], False
        elif t == "pick":
            if not approached:
                lines.append(f"approach {_n(fine_distance)}")
                approached = True
            r, i = s["slot"]
            turn = turn_name(s.get("turn_deg", 0.0))
            if turn != "turn_0":
                lines.append(f"skill drive {turn} {T} mode=OFF")
            pk = A[s["at"]].extra["pucks"][s["side"]]
            lx, ly = puck_after_turn(pk["fwd"], pk["lat"], s.get("turn_deg", 0.0))
            lines.append(f"skill manipulator target_look:base_footprint:{lx:.4f}:{ly:.4f}:{PUCK_CENTER_Z:.3f} {T} "
                         "mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG")
            lines.append(f"skill manipulator toppick_tilt {T} mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG")
            lines.append(f"skill manipulator pallet_load_{r}_{i}_tilt {T} mode=OFF")
            if turn != "turn_0":
                lines.append(f"skill drive turn_0 {T} mode=OFF")  # 다음 퍽 · 주행 전 정렬 방향으로
        elif t == "place":
            if not approached:
                lines.append(f"approach {_n(fine_distance)}")
                approached = True
            r, i = s["from_slot"]
            dx, dy = (int(v) for v in s["cell"].split(","))
            lines.append(f"skill manipulator pallet_unload_{r}_{i}_tilt {T} mode=OFF")
            lines.append(f"skill manipulator place_{s['at']}_{_idx(dx)}_{_idx(dy)}_tilt {T} mode=SEARCH_FRONT+ARM_ALIGN")
        elif t == "verify":
            L = s["at"]
            g = plan["groups"][L]
            cx, cy = A[L].extra["cells"]["0,0"]["x"], A[L].extra["cells"]["0,0"]["y"]
            ys = SHAPE_CODE.get(plan["allocation"][L].get("y_shape") or "", 0) if g["group"] == "Y" else 0
            lines.append(f"verify {L} {_n(cx)} {_n(cy)} {_n(PUCK_CENTER_Z - 0.03)} {expect_cells(g['count'], g['group'])} {ys}")
        elif t == "return":
            lines.append(f"wait {_n(float(return_wait))}")
    notes.append(f"approach FINE {fine_distance} m (TFmini 렌즈 -> 면) 자리 = 접근 자세표 final (앞 끝 -> 면 {FINE_GAP:.4f} m, #64 안 A)")
    if abs(fine_distance - FINE_DISTANCE) > 1e-9:
        notes.append(f"★ fine_distance {fine_distance} 가 접근 자세표 FINE_DISTANCE {FINE_DISTANCE} 와 다름 -> 집기 · 놓기 칸 거리가 어긋남")
    return lines, notes
