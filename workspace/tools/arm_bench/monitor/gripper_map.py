"""그리퍼 모터 각 (6 번째 칸, ID 15) -> URDF 손가락 위치 (gripper_left/right_joint, m) 변환. 모터 tick 을 직선으로 나눔 (가정 ★).

- 2026-10-27 그리퍼를 다시 결합한 뒤 실측 (사용자): 완전히 닫힘 CLOSED_TICK = 2650 -> 손가락 -0.011 m (ROBOTIS 닫힘),
  끝까지 열림 OPEN_TICK = 1550 -> 손가락 0.020 m. 닫을수록 tick 증가, 이동량 1100. 한 바퀴 (0 ~ 4095) 안이라 0 을 지나지 않음.
- unwrap_tick 은 예전 결합 (0 을 지나던) 때 것으로, 지금은 값이 그대로 나옴.
"""
import math
import os
import re

CLOSED_TICK = 2650
OPEN_TICK = 1550
FINGER_CLOSED, FINGER_OPEN = -0.011, 0.020   # [m] URDF gripper_left/right_joint 한계 (ROBOTIS)
RAD_PER_TICK = 2 * math.pi / 4096
CONFIG_H = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "..", "..", "ros2_ws", "src", "opencr_firmware", "config.h")   # Lv2 펌웨어


def firmware_grip():
    """펌웨어 그리퍼 막힘 보호 값 (config.h GRIP_*, 2026-10-27): stall · progress · stall_ms · squeeze [tick/ms] · pwm_max"""
    text = open(CONFIG_H, encoding="utf-8").read()

    def val(name):
        return int(re.search(name + r"\s*=\s*(\d+)", text).group(1))
    return dict(stall=val("GRIP_STALL_TICKS"), progress=val("GRIP_PROGRESS_TICKS"), stall_ms=val("GRIP_STALL_MS"),
                squeeze=val("GRIP_SQUEEZE_TICKS"), pwm_max=val("GRIP_PWM_LIMIT_MAX"))


def unwrap_tick(tick, closed=CLOSED_TICK, opened=OPEN_TICK):
    mid = (closed + opened) / 2
    return min((tick + k * 4096 for k in (-1, 0, 1)), key=lambda t: abs(t - mid))


def finger_from_tick(tick, closed=CLOSED_TICK, opened=OPEN_TICK):
    """그리퍼 모터 tick -> 손가락 위치 [m] (닫힘 -0.011 ~ 열림 0.020 으로 자름)"""
    s = (unwrap_tick(tick, closed, opened) - closed) / (opened - closed)
    return FINGER_CLOSED + min(1.0, max(0.0, s)) * (FINGER_OPEN - FINGER_CLOSED)


def rad_from_finger(m, closed=CLOSED_TICK, opened=OPEN_TICK):
    """손가락 위치 [m] -> 그리퍼 모터 목표 (펌웨어 6 번째 칸 rad). 닫힘 ~ 열림 밖은 자름"""
    s = min(1.0, max(0.0, (m - FINGER_CLOSED) / (FINGER_OPEN - FINGER_CLOSED)))
    return (closed + s * (opened - closed) - 2048) * RAD_PER_TICK


def finger_from_rad(rad, closed=CLOSED_TICK, opened=OPEN_TICK):
    """펌웨어 STATE 의 6 번째 칸 (rad = (tick - 2048) x 2π/4096) -> 손가락 위치 [m]"""
    return finger_from_tick(2048 + rad / RAD_PER_TICK, closed, opened)
