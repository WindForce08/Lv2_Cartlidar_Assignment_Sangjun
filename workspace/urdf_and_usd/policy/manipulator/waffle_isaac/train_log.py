"""학습 기록 공통 (보상 항목별 에피소드 합 · 종료 원인 · 행동 포화). 학습 동작 (관측 · 행동 · 보상 값 · 종료) 에는 영향 없음.

TensorBoard 그룹:
  Episode_Reward/<항목>        끝난 에피소드의 항목별 보상 합 (평균)
  Episode_Termination/<원인>   끝난 에피소드 중 그 원인 비율 (여러 개 동시 가능, 성공해도 시간 초과까지 가면 time_out)
  Saturation/*                 행동 포화 (PPO 는 자르기 전 표본으로 확률을 계산 -> 자주 넘으면 학습 왜곡)

쓰는 법 (DirectRLEnv):
  __init__           self.tlog = TrainLog(self, ("tipped", "fell", ...))
  _pre_physics_step  self.tlog.actions(actions, lead_err=raw - q, lead=..., target=t, lo=self._lo, hi=self._hi)
  _get_rewards       rew = self.tlog.reward({"reach": ..., ...})
  _get_dones         self.tlog.dones(tipped=..., ..., time_out=time_out)
  _reset_idx         self.tlog.reset(env_ids)  (super()._reset_idx 전, 에피소드 길이를 읽으므로)
행동 마지막 열 = 집게, 나머지 = 움직임 (네 환경 공통).
"""
from __future__ import annotations

import torch


class TrainLog:
    def __init__(self, env, term_keys):
        n, dev = env.num_envs, env.device
        self.env = env
        self.keys = tuple(term_keys) + ("time_out",)
        self.ep: dict[str, torch.Tensor] = {}
        self.flags = {k: torch.zeros(n, dtype=torch.bool, device=dev) for k in self.keys}
        self.sat_steps = torch.zeros(n, device=dev)  # 움직임 행동이 [-1, 1] 넘은 스텝 수 (에피소드)

    def _log(self):
        return self.env.extras.setdefault("log", {})

    def actions(self, raw, lead_err=None, lead=None, target=None, lo=None, hi=None):
        """raw = 자르기 전 행동. lead_err = (목표 + 변화) - 현재 관절, lead = 앞섬 한계, target = 앞섬으로 자른 뒤 관절 목표."""
        mv = raw[:, :-1]
        over = mv.abs() > 1.0
        self.sat_steps += over.any(1).float()
        log = self._log()
        log["Saturation/action_move"] = over.float().mean().item()  # 움직임 행동 중 넘은 비율
        log["Saturation/action_abs_mean"] = mv.abs().mean().item()  # 자르기 전 크기 (1 근처면 포화 직전)
        log["Saturation/gripper_margin"] = (raw[:, -1].abs() < 0.1).float().mean().item()  # 집게 열기·닫기 경계 근처 (흔들림)
        if lead_err is not None:
            log["Saturation/lead_clamp"] = (lead_err.abs() > lead).float().mean().item()  # 관절 앞섬 한계에 걸린 비율
        if target is not None:
            log["Saturation/joint_limit"] = ((target <= lo) | (target >= hi)).float().mean().item()  # 관절 한계에 걸린 비율

    def reward(self, terms):
        """terms = {이름: (N,) 텐서 또는 상수} -> 합 (N,). 항목별로 에피소드 합 누적."""
        n, dev = self.env.num_envs, self.env.device
        vals = {k: torch.broadcast_to(torch.as_tensor(v, dtype=torch.float32, device=dev), (n,)) for k, v in terms.items()}
        for k, v in vals.items():
            self.ep[k] = self.ep.get(k, torch.zeros(n, device=dev)) + v
        return torch.stack(list(vals.values()), dim=0).sum(0)

    def dones(self, **flags):
        self.flags = flags

    def reset(self, env_ids):
        log = self._log()
        for k, v in self.ep.items():
            log[f"Episode_Reward/{k}"] = v[env_ids].mean().item()
            v[env_ids] = 0.0
        for k in self.keys:
            if k in self.flags:
                log[f"Episode_Termination/{k}"] = self.flags[k][env_ids].float().mean().item()
        eplen = self.env.episode_length_buf[env_ids].float().clamp(min=1.0)
        log["Saturation/episode_ratio"] = (self.sat_steps[env_ids] / eplen).mean().item()  # 에피소드 중 포화 스텝 비율
        self.sat_steps[env_ids] = 0.0
