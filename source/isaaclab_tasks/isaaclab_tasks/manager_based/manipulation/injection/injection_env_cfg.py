# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
import torch
# from isaaclab_tasks.manager_based.manipulation.injection.stab_path_filter import SimpleIKHelper

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.manager_based.manipulation.injection.mdp as mdp

##
# Scene definition
##


@configclass
class InjectionSceneCfg(InteractiveSceneCfg):
    """Configuration for the scene with a robotic arm."""

    # world
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)),
    )

    # robots
    robot: ArticulationCfg = MISSING

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""
    ee_pose = mdp.UniformPoseCommandCfg(
        # ik_helper_path="actions.ik_filter_action.controller",
        asset_name="robot",
        body_name=MISSING,
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            # 这是一次点位中b点位置
            roll=(1.57,1.57),
            pitch=(1.57,1.57),
            yaw=(1.57,1.57),
            pos_x=(0.2,0.2),
            pos_y=(0.0,0.0),
            pos_z=(0.3,0.3),
            # 下面唯一的位置下，c的位置
            # roll=(1.57,1.57),
            # pitch=(1.57,1.57),
            # yaw=(1.57,1.57),
            # pos_x=(0.24,0.24),
            # pos_y=(0.0,0.0),
            # pos_z=(0.3,0.3),
            # roll=(-1.435,-1.435),
            # pitch=(3.142,3.142),
            # yaw=(-1.629,-1.629),
            # pos_x=(-0.246,-0.246),
            # pos_y=(-0.031,-0.031),
            # pos_z=(0.037,0.037),
            # 把下面的 min,max 都设成同一个数
            # roll=(-2,2),
            # pitch=(-1,1),
            # yaw=(-2,2),
            # pos_x=(-0.3,0.3),
            # pos_y=(-0.3,0.3),
            # pos_z=(-0.3,0.3),
            #-----上面是撞库的全范围
            # roll=(-16.57,-16.57),
            # pitch=(-43.26,-43.26 ),
            # yaw=(75.64, 75.64),
            # pos_x=(0.54, 0.54),
            # pos_y=(0.20, 0.20),
            # pos_z=(0.638, 0.638),
            # --------
            # roll=(-24.0, -24.0),
            # pitch=(0, 0),
            # yaw=(450.0, 450.0),
            # pos_x=(0.15, 0.15),
            # pos_y=(0.00, 0.00),
            # pos_z=(0.2, 0.2),
            # roll=(-1.5708, -1.5708),
            # pitch=(0.0, 0.0),
            # yaw=(1.5708, 1.5708),

            # 更小逻辑， 为了测试ik的可行性，抛弃b点的随机生成，采用固定生成一个b点。固定死一个位置。
                # pos_x=(0.10, 0.25),
                # pos_y=(-0.12, 0.12),
                # pos_z=(0.05, 0.22),
                # roll=(0.0, 0.0),
                # pitch=(0.0, 0.0),
                # yaw=(-3.14, 3.14),
            # 修改为更小范围，逻辑上，这个空间是一个长方体，具体这个数值为什么，没有理论，只有gpt分析。
            # pos_x=(0.2,0.29),
            # pos_y=(-0.12,0.12),
            # pos_z=(0.1,0.22),
            # roll=(0.0,0.0),
            # pitch=(0.0, 0.0),
            # yaw=(-3.14, 3.14),
            # 修改为mycombot的数据。但是这不应该是最终数据，最终应冗余出 准备打针前的位置空间。
            # pos_x=(0.2, 0.32),
            # pos_y=(-0.12, 0.12),
            # pos_z=(0.1, 0.25),
            # roll=(0.0, 0.0),
            # pitch=(0.0, 0.0),
            # yaw=(-3.14, 3.14),

            # pos_x=(0.35, 0.65),
            # pos_y=(-0.2, 0.2),
            # pos_z=(0.15, 0.5),
            # roll=(0.0, 0.0),
            # pitch=MISSING,  # depends on end-effector axis
            # yaw=(-3.14, 3.14),
        ),
    )



    # ee_pose = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name=MISSING,
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #             pos_x=(0.10, 0.25),
    #             pos_y=(-0.12, 0.12),
    #             pos_z=(0.05, 0.22),
    #             roll=(0.0, 0.0),
    #             pitch=(0.0, 0.0),
    #             yaw=(-3.14, 3.14),
    #         # 修改为更小范围，逻辑上，这个空间是一个长方体，具体这个数值为什么，没有理论，只有gpt分析。
    #         # pos_x=(0.2,0.29),
    #         # pos_y=(-0.12,0.12),
    #         # pos_z=(0.1,0.22),
    #         # roll=(0.0,0.0),
    #         # pitch=(0.0, 0.0),
    #         # yaw=(-3.14, 3.14),
    #         # 修改为mycombot的数据。但是这不应该是最终数据，最终应冗余出 准备打针前的位置空间。
    #         # pos_x=(0.2, 0.32),
    #         # pos_y=(-0.12, 0.12),
    #         # pos_z=(0.1, 0.25),
    #         # roll=(0.0, 0.0),
    #         # pitch=(0.0, 0.0),
    #         # yaw=(-3.14, 3.14),

    #         # pos_x=(0.35, 0.65),
    #         # pos_y=(-0.2, 0.2),
    #         # pos_z=(0.15, 0.5),
    #         # roll=(0.0, 0.0),
    #         # pitch=MISSING,  # depends on end-effector axis
    #         # yaw=(-3.14, 3.14),
    #     ),
    # )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = MISSING
    gripper_action: ActionTerm | None = None


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "ee_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # task terms
    end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    )
    end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.1,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "std": 0.1, "command_name": "ee_pose"},
    )
    end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=MISSING), "command_name": "ee_pose"},
    )
    # demo版本b-》c
    bc_line_reachability = RewTerm(
        func=mdp.bc_line_reachability_reward,
        weight=1.0,
        params={
        "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
        "command_name_b": "ee_pose",   # 先用现有这个
        "command_name_c": "ee_pose",   # 先用同一个占位
    },
        # params={
        #     "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
        #     "command_name_b": "b_pose",
        #     "command_name_c": "c_pose",
        # },\
    )
    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    # ------------后面是自己新建的两个reward 项目。-------
    # Add this in RewardsCfg:
    # path_deviation_reward = RewTerm(
    #     func=mdp.path_deviation,
    #     weight=-0.1,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
    #         "point_a": torch.tensor([0.10, 0.0, 0.1]),
    #         "point_b": torch.tensor([0.25, 0.0, 0.1]),
    #     },
    # )
    # motion_smoothness_reward = RewTerm(
    # func=mdp.motion_smoothness,
    # weight=-0.001,
    # params={"asset_cfg": SceneEntityCfg("robot")},
# )



@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -0.005, "num_steps": 4500}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 4500}
    )


##
# Environment configuration
##


@configclass
class InjectionEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Injection end-effector pose tracking environment."""

    # Scene settings
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=4096, env_spacing=2.5)
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=2048, env_spacing=2.5)
    # ik_filter_action: ActionsCfg=ActionsCfg()

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0

    
@configclass
class InjectionEnvPlayCfg(InjectionEnvCfg):
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=1, env_spacing=2.0)


@configclass
class InjectionEnvB2CCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Injection end-effector pose tracking environment."""

    # Scene settings
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=4096, env_spacing=2.5)
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=2048, env_spacing=2.5)
    # ik_filter_action: ActionsCfg=ActionsCfg()

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 12.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0

    
@configclass
class InjectionEnvB2CPlayCfg(InjectionEnvCfg):
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=1, env_spacing=2.0)