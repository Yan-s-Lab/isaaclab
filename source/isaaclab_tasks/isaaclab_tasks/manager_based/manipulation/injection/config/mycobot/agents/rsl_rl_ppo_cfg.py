# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class MyCobotInjectionPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 1500
    save_interval = 50
    experiment_name = "mycobot_injection"
    run_name = ""
    resume = False
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8, # 初始噪声 0.5 ，改为0.8
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.02, # 0.005, # 
        num_learning_epochs=8,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.02, # 0.01,
        max_grad_norm=1.0,
    )



# # -----1 号志国 为了防止抖动而让 gpt
# #推荐的修改参数,试试他如何-- # Copyright (c) 2022-2025, The Isaac Lab Project Developer

# # Copyright (c) 2022-2025, The Isaac Lab Project Developers
# # SPDX-License-Identifier: BSD-3-Clause

# from isaaclab.utils import configclass
# from isaaclab_rl.rsl_rl import (
#     RslRlOnPolicyRunnerCfg,
#     RslRlPpoActorCriticCfg,
#     RslRlPpoAlgorithmCfg,
# )

# @configclass
# class MyCobotInjectionPPORunnerCfg(RslRlOnPolicyRunnerCfg):
#     # ===== Runner =====
#     num_steps_per_env = 32          # 每个 env 采样步数
#     max_iterations =   600          # 训练迭代上限 原来1500
#     save_interval = 50
#     experiment_name = "mycobot_injection"
#     run_name = ""
#     resume = False
#     empirical_normalization = False

#     # ===== Policy (Actor-Critic 网络与策略分布) =====
#     policy = RslRlPpoActorCriticCfg(
#         # 动作分布初始标准差 (越小=初始噪声越低；建议 0.25~0.6 之间试)
#         init_noise_std=0.35,        # 原 0.8 → 0.35
#         actor_hidden_dims=[64, 64],
#         critic_hidden_dims=[64, 64],
#         activation="elu",
#     )

#     # ===== Algorithm (PPO 超参) =====
#     algorithm = RslRlPpoAlgorithmCfg(
#         value_loss_coef=1.0,
#         use_clipped_value_loss=True,
#         clip_param=0.2,
#         # 策略熵系数 (越小=越少鼓励随机探索；能显著降低抖动)
#         entropy_coef=0.003,         # 原 0.02 → 0.003
#         num_learning_epochs=8,
#         num_mini_batches=4,
#         # 学习率适当降低以稳住 std/熵 的波动
#         learning_rate=3.0e-4,       # 原 1.0e-3 → 3.0e-4
#         # 使用自适应日程，以下 KL 目标控制策略步幅（含 std 的变化幅度）
#         schedule="adaptive",
#         desired_kl=0.01,            # 原 0.02 → 0.01
#         gamma=0.99,
#         lam=0.95,
#         max_grad_norm=1.0,
#     )
