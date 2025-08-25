# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
import torch
import math

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

    # 神奇的bug， ee_pose 和b pose，c pose使用的参数一样，但是结果不一样。
    # 结果不一样是指，eepose的结果，
    # 从日志输出看来。是这样的，不管原因了，先解决这个bug吧。
    # ee_pose 的四元数
    # quat=[0.4996, -0.5000, 0.5004, -0.5000]
    # 这对应的 RPY≈(1.57, π, 1.57)（roll=90°，pitch=180°，yaw=90°）。

    # b_pose / c_pose 的四元数
    # quat=[0.7071, 0.00028, 0.7071, 0.00028]
    # 这对应的 RPY≈(1.57, 1.57, 1.57)（三个轴都是≈90°）。

    # 也就是说：ee_pose 实际用了 pitch=π（180°），而 b/c 用的是 pitch=1.57（90°），所以三者姿态不一致，这不是“旋转乱了”，而是配置生效的不一致。
    ee_pose = mdp.UniformPoseCommandCfg(
        # ik_helper_path="actions.ik_filter_action.controller",
        asset_name="robot",
        body_name="joint6_flange",
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            # 这是一次点位中b点位置
            roll=(1.57,1.57),
            pitch=(1.57,1.57),
            yaw=(1.57,1.57),
            pos_x=(-0.2,0.2),
            pos_y=(-0.2,0.2),
            pos_z=(0.14,0.34),
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

    # 注意把 body_name 填成你的 EEF（比如 joint6_flange）
   
    b_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="joint6_flange",
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            roll=(1.57, 1.57),
            pitch=(math.pi, math.pi),
            yaw=(1.57, 1.57),
            pos_x=(0.25, 0.25),
            pos_y=(0.0, 0.0),
            pos_z=(0.3, 0.3),
        ),
    )

    c_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="joint6_flange",
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            roll=(1.57, 1.57),
            pitch=(math.pi, math.pi),
            yaw=(1.57, 1.57),
            pos_x=(0.27, 0.27),
            pos_y=(0.0, 0.0),
            pos_z=(0.3, 0.3),
        ),
    )
    # 先完全一样；确认流程后，再改成有“直线方向”的偏移


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

    # # 新增 / 修正：初始化 IK helper —— 注意不要传 asset_cfg
    # init_bc_ik_helper = EventTerm(
    #     func=mdp.init_bc_ik_helper,   # 就是你 mdp 里那函数
    #     mode="reset",
    #     params={"damping": 1e-3, "max_iters": 128, "tol": 1e-3},
    # )


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
    # bc_line_reachability = RewTerm(
    #     func=mdp.bc_line_reachability_reward,
    #     weight=1.0,
    #     params={
    #     "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
    #     "command_name_b": "b_pose",   # 替换
    #     "command_name_c": "c_pose",   # 替换实际
    #     "reach_pos_tol": 0.03,  # 先放宽到 3cm 验证触发
    # },
        # params={
        #     "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
        #     "command_name_b": "b_pose",
        #     "command_name_c": "c_pose",
        # },\
    # )

    
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
    bc_line_reach_cmd = RewTerm(
    func=mdp.bc_line_reach_term_cmd,   # 奖励函数：评估从 B→C 的直线可达性（外部运动学，不改仿真）
    weight=1.0,
    params={
        # ===== 机器人与末端设置 =====
        "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
        # ^ 场景里机器人在 env.scene 的名字；body_names[0] 是用来判定“到达B”的末端连杆名

        # ===== B/C 目标来源（命令名）=====
        "b_command_name": "b_pose",    # 从 command_manager 读取名为 "b_pose" 的目标姿态作为 B
        "c_command_name": "c_pose",    # 若为 None，则不用命令而是用下面的 offset 从 B 推出 C

        # ===== 无 c 命令时，如何由 B 推出 C =====
        "c_offset_in_tool": 0.04,      # 单位 m，沿“B 的工具坐标系 +X 方向”前推这段距离得到 C 的位置
        "keep_c_orientation": True,    # True: C 的朝向 = B 的朝向；False: 用当前 EEF 实际朝向作为 C 的朝向
        # 注：当 c_command_name 非 None 时，上面两项不会生效（优先使用 c 命令）

        # ===== Pinocchio 运动学模型（仅 backend="pin" 时有效）=====
        # "urdf_path": "/home/lzg/codes/ros2_package/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange.urdf",
        "urdf_path": "/home/lzg/codes/sim_usd_assets/robot/mycombot/mycobot_280_m5/mycobot_280_m5/mycobot_280_m5",
        "urdf_eef_frame": "joint6_flange",  # URDF 中末端 frame 名（应与 body_names[0] 对应）
        # 解析 URDF 里 package:// 的资源路径（网格/碰撞几何）。有需要再打开：
        # "pin_package_dirs": [
        #     "/home/lzg/codes/ros2_package/mycobot_ros2"
        # ],

        # ===== 自碰撞检查（可选，开启会更慢但更真实）=====
        "enable_collision": False,     # True 则在影子 IK 过程中做纯运动学自碰撞检查
        # "disable_adjacent_pairs": True, # 忽略父子/同一关节相邻连杆对，减少伪碰撞

        # ===== 评估触发策略（推荐一次性奖励）=====
        "debug_force_now": False,      # False：不每步评估；配合下一项只在“到达B”时触发一次
        "trigger_once_after_b": True,  # True：首次满足“到达B”阈值的那一帧触发评估并给一次性奖励

        # “到达 B”判定阈值（仅一次性触发模式使用；不传则用默认：1cm / 5°）
        "b_reach_pos_tol": 0.01,                     # m，EEF 到 B 的位置误差阈值
        "b_reach_rot_tol": math.radians(5.0),        # rad，EEF 到 B 的姿态误差阈值

        # ===== B→C 直线路径可达性求解参数 =====
        "steps": 4,                 # 航点数：1=只评估终点 C；>1=插值评估整条路径（更严格更慢）
        "max_iters_per_wp": 20,    # 每个航点 DLS 迭代上限
        "pos_tol": 0.2,             # 航点位姿误差阈值（m）；建议调参：先宽松，收敛后逐步收紧
        "rot_tol": math.radians(5.0), # 航点姿态误差阈值（rad）；最终可收紧到 ~0.5°（math.radians(0.5)）
        "dls_lambda2": 1e-4,        # DLS 阻尼；大一点更稳但可能慢些
        "joint_limit": True,        # 影子 IK 迭代时夹在 URDF 关节限位内
        "vel_limit": False,         # 若 True，会限制每步 dq 幅度（更稳、更慢）

        # ===== 后端选择 =====
        "backend": "none",          # "pin"：启用 Pin 评估并产生实际得分；"none"：直接返回 0（便于 GUI/无 Pin 环境调试）
    },
)


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
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=4096, env_spacing=2.5)
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=2048, env_spacing=2.5)
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=8192, env_spacing=2.5)

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