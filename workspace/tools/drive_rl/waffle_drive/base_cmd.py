"""정책 행동 → 바퀴 속도. 실물 명령이 지나가는 길의 한계를 흉내 낸다 (torch 만 씀, Isaac 없이 시험 가능).

  정책 출력 [-1, 1]² × drive_node skill 한계 (0.3 m/s, 1.5 rad/s)     ros2_ws/src/control/src/drive_node.cpp skill.max_*
  → diff_drive: |vx| ≤ 0.34, |wz| ≤ 2.0, vx 가속 · 감속 ≤ 1.5 m/s²    opencr_hardware/config/controllers_4wd.yaml · drive_belt.yaml
    (각속도 가속 한계는 설정에 없음)
  → 바퀴 ω = (vx ∓ wz · 좌우 간격 / 2) / 반지름
  → 한 바퀴라도 한계를 넘으면 좌우를 같은 비율로 줄임                  펌웨어 비례 축소 (PR #10). controller_manager 는 바퀴를
                                                                        따로 자르지 않음 (opencr.ros2_control.xacro limits enable false)
바퀴 한계는 시뮬 바퀴 한계 (팀 robot_params.json 10.996 rad/s) 를 쓴다. 실물 펌웨어 한계 (모터 7.91 rad/s × 벨트 1.5 = 11.87) 보다
작아서, 시뮬 관절이 바퀴마다 따로 잘려 회전 반경이 바뀌는 일이 없다.
"""

from __future__ import annotations

import torch

SKILL_VX = 0.3  # drive_node skill.max_linear [m/s] = 정책 vx 출력 범위
SKILL_WZ = 1.5  # drive_node skill.max_angular [rad/s] = 정책 wz 출력 범위
DD_VX = 0.34  # diff_drive linear.x.max_velocity (drive_belt.yaml) [m/s]
DD_WZ = 2.0  # diff_drive angular.z.max_velocity [rad/s]
DD_AX = 1.5  # diff_drive linear.x.max_acceleration = -max_deceleration [m/s²]


class BaseCmd:
    """환경마다 diff_drive 가속 한계 상태 (직전 vx 명령) 를 가진다. step() 은 물리 스텝마다 부른다.

    mode "twist" (기본): 위 사슬. "wheels": 행동 = 왼쪽 · 오른쪽 바퀴 속도 × 바퀴 한계 (예전 D1 방식, 비교용).
    """

    def __init__(self, num_envs: int, device, wheel_radius: float, wheel_separation: float, wheel_speed_max: float,
                 dt: float, mode: str = "twist"):
        if mode not in ("twist", "wheels"):
            raise ValueError(f"action_mode 는 twist 또는 wheels: {mode}")
        self.mode = mode
        self.r, self.sep, self.wheel_max, self.dt = wheel_radius, wheel_separation, wheel_speed_max, dt
        self.vx = torch.zeros(num_envs, device=device)
        """한계를 적용한 뒤의 vx 명령 [m/s] (관측에도 씀 — 실물에서는 호스트가 아는 값)"""

    def reset(self, env_ids) -> None:
        self.vx[env_ids] = 0.0

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """actions (N, 2) → (왼쪽, 오른쪽) 바퀴 각속도 [rad/s], 각각 (N,)"""
        a = actions.clamp(-1.0, 1.0)
        if self.mode == "wheels":
            wl, wr = a[:, 0] * self.wheel_max, a[:, 1] * self.wheel_max
            self.vx = (wl + wr) * 0.5 * self.r
            return wl, wr
        vx_target = (a[:, 0] * SKILL_VX).clamp(-DD_VX, DD_VX)
        wz = (a[:, 1] * SKILL_WZ).clamp(-DD_WZ, DD_WZ)
        dv = DD_AX * self.dt
        self.vx = self.vx + (vx_target - self.vx).clamp(-dv, dv)
        half = wz * self.sep * 0.5
        wl, wr = (self.vx - half) / self.r, (self.vx + half) / self.r
        s = (torch.maximum(wl.abs(), wr.abs()) / self.wheel_max).clamp(min=1.0)
        return wl / s, wr / s
