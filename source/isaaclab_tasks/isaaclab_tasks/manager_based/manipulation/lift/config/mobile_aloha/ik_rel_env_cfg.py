# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
# 这里换成你的 mobile aloha HIGH_PD cfg
# 你如果还没有 HIGH_PD 版本，也可以先用普通版本，但 IK tracking 会更抖一点
from isaaclab_assets.robots.maloha import MOBILE_ALOHA_FULL_CFG  # <- 按你的真实路径改


@configclass
class MalohaCubeLiftEnvCfg(joint_pos_env_cfg.MalohaCubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Mobile Aloha as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = MOBILE_ALOHA_FULL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (mobile aloha)
        # 只控制左臂：joint_names 只写左臂 6DoF 的关节
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",

            # ✅ 改这里：左臂关节名（推荐用正则 / 或者写全名列表）
            # 例1：如果你的关节名就叫 fl_joint1..6：
            joint_names=[r"fl_joint[1-6]"],
            # 例2：如果实际带路径/前缀（更常见），用更稳的：
            # joint_names=[r".*fl_joint[1-6]$"],

            # ✅ 改这里：末端 link 名（用你面板里看到的左臂末端 link）
            # 常见：腕部最后一节、夹爪基座 link
            body_name="fl_link6",

            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="dls",
            ),

            scale=0.5,

            # ✅ 改这里：末端偏移
            # Franka 用 0.107 是把控制点从 hand link 原点移到抓取点
            # 你这里要按 mobile aloha 夹爪/工具的几何来
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                pos=[0.0, 0.0, 0.0]
                # e.g. pos=[0.0, 0.0, 0.12]
            ),
        )


@configclass
class MalohaCubeLiftEnvCfg_PLAY(MalohaCubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
