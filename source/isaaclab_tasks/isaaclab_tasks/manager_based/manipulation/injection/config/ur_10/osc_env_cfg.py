# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause
import math

from isaaclab.controllers.operational_space_cfg import OperationalSpaceControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import OperationalSpaceControllerActionCfg
from isaaclab.utils import configclass

from . import joint_pos_env_cfg

from isaaclab_assets import UR10_CFG  # Pre-defined UR10 config


@configclass
class UR10InjectionEnvCfg(joint_pos_env_cfg.InjectionEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Use UR10 robot with OSC controller
        self.scene.robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 关闭重力影响
        self.scene.robot.spawn.rigid_props.disable_gravity = True


        # 在 scene.robot 设置部分，添加：
        # self.scene.robot.actuators["shoulder_joint"].stiffness = 0.0
        # self.scene.robot.actuators["shoulder_joint"].damping = 0.0
        # self.scene.robot.actuators["elbow_joint"].stiffness = 0.0
        # self.scene.robot.actuators["elbow_joint"].damping = 0.0

        # 适配 ee_link 为末端执行器
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["ee_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["ee_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["ee_link"]

        # 使用 OSC 控制器
        self.actions.arm_action = OperationalSpaceControllerActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            body_name="ee_link",
            controller_cfg=OperationalSpaceControllerCfg(
                target_types=["pose_abs"],
                impedance_mode="variable_kp",
                inertial_dynamics_decoupling=True,
                partial_inertial_dynamics_decoupling=False,
                gravity_compensation=False,
                motion_stiffness_task=100.0,
                motion_damping_ratio_task=1.0,
                motion_stiffness_limits_task=(50.0, 200.0),
                nullspace_control="none",  # 可选："none" 或 "position"（UR10 有冗余）
            ),
            position_scale=1.0,
            orientation_scale=1.0,
            stiffness_scale=100.0,
        )

        # 命令生成器目标为 ee_link，并指定姿态
        self.commands.ee_pose.body_name = "ee_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # 移除冗余 joint obs，OSC 可选
        self.observations.policy.joint_pos = None
        self.observations.policy.joint_vel = None


@configclass
class UR10InjectionEnvCfg_PLAY(UR10InjectionEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
