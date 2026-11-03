"""주행 과제 RSL-RL PPO 설정. 팀 rl/waffle_isaac/agents 의 WaffleDrivePPORunnerCfg (작은 MLP, 저사양 GPU 용) 와 같고 실험 이름 · 반복만 다르다.
미로 과제는 라이다 36 개가 관측에 들어가 MLP 를 128-128 로 키움."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class WaffleDriveFlatPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """단계 A (관측 11, 행동 2)."""

    num_steps_per_env = 32
    max_iterations = 300
    save_interval = 50
    experiment_name = "waffle_drive_flat"
    actor = RslRlMLPModelCfg(
        hidden_dims=[64, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.6),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[64, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive", gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class WaffleDriveMazePPORunnerCfg(WaffleDriveFlatPPORunnerCfg):
    """미로 경로 따라가기 (관측 53 = 17 + 라이다 36, 행동 2)."""

    max_iterations = 1000
    experiment_name = "waffle_drive_maze"
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.6),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[128, 128], activation="elu", obs_normalization=True)
