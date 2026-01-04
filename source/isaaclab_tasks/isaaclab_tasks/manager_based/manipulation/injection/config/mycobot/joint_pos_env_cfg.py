# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.managers import ActionTermCfg as ActionTerm

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
# from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
from isaaclab_tasks.manager_based.manipulation.injection.injection_env_cfg import InjectionEnvCfg
##
# Pre-defined configs
##
from isaaclab_assets import FRANKA_PANDA_CFG  # isort: skip
from isaaclab_assets import MYCOBOT_280_M5_HIGH_PD_CFG,MYCOMBOT_280_M5_CFG # high pd & normal one.


##
# Environment configuration
##

@configclass
class MyCobotInjectionEnvCfg(InjectionEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # ✅ 用高 PD 配置替换
        # self.scene.robot = MYCOBOT_280_M5_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = MYCOMBOT_280_M5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # ✅ 替换控制末端名
        # self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["joint6_flange"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["joint6_flange"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["joint6_flange"]

        # ✅ 替换动作的关节名
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
                "joint6output_to_joint6"
            ],
            scale=0.5,
            # scale=1.2,

            use_default_offset=True
        )

        # ✅ 替换 command 目标末端
        self.commands.ee_pose.body_name = "joint6_flange"
        # self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)

        # todo，等待验证。lzg 专门给 b→c 路径筛选用的 IK 控制器
        # 2) 新增：专门给 FilteredPoseCommand 做路径可达性检查的 IK 控制器
        # self.actions.ik_filter_action = DifferentialInverseKinematicsActionCfg(
        #     asset_name="robot",
        #     joint_names=[
        #         "joint2_to_joint1",
        #         "joint3_to_joint2",
        #         "joint4_to_joint3",
        #         "joint5_to_joint4",
        #         "joint6_to_joint5",
        #         "joint6output_to_joint6",
        #         ],
        #     body_name="joint6_flange",
        #     controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
        #     body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.05]),
        # )


        # gripper_action: ActionTerm | None = None


@configclass
class MyCobotInjectionEnvCfg_PLAY(MyCobotInjectionEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.0
        self.observations.policy.enable_corruption = False

