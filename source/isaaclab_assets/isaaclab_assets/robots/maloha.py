# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Mobile ALOHA robot (arms actuated, wheels passive, stable PD)."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg


# -----------------------------
# 更稳的 PD 建议值（先跑起来，再细调）
# -----------------------------
# 6DoF 手臂（joint1-6）：中等硬度 + 非零阻尼，避免抖动/漂 ARM_EFFORT = 60.0 ARM_VEL = 3.0 ARM_STIFFNESS = 200.0 ARM_DAMPING = 20.0
# 6DoF 手臂（joint1-6）：先保证能抗重力
ARM_EFFORT = 2000.0
ARM_STIFFNESS = 30000.0
ARM_DAMPING = 800.0
ARM_VEL = 6.0



# 夹爪（joint7-8）：更硬一些，否则 Binary position 很可能不动
GRIPPER_EFFORT = 80.0
GRIPPER_VEL = 5.0
GRIPPER_STIFFNESS = 2000.0
GRIPPER_DAMPING = 100.0


# -----------------------------
# 只给“手臂+夹爪”加 actuator
# wheels/castors 不加 actuator -> 纯被动（推荐用于先做手臂 teleop）
# -----------------------------
_ACTUATORS = {
    # back-left
    "bl_arm": ImplicitActuatorCfg(
        joint_names_expr=["bl_joint[1-6]"],
        effort_limit_sim=ARM_EFFORT,
        velocity_limit_sim=ARM_VEL,
        stiffness=ARM_STIFFNESS,
        damping=ARM_DAMPING,
    ),
    "bl_gripper": ImplicitActuatorCfg(
        joint_names_expr=["bl_joint[7-8]"],
        effort_limit_sim=GRIPPER_EFFORT,
        velocity_limit_sim=GRIPPER_VEL,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
    ),

    # back-right
    "br_arm": ImplicitActuatorCfg(
        joint_names_expr=["br_joint[1-6]"],
        effort_limit_sim=ARM_EFFORT,
        velocity_limit_sim=ARM_VEL,
        stiffness=ARM_STIFFNESS,
        damping=ARM_DAMPING,
    ),
    "br_gripper": ImplicitActuatorCfg(
        joint_names_expr=["br_joint[7-8]"],
        effort_limit_sim=GRIPPER_EFFORT,
        velocity_limit_sim=GRIPPER_VEL,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
    ),

    # front-left
    "fl_arm": ImplicitActuatorCfg(
        joint_names_expr=["fl_joint[1-6]"],
        effort_limit_sim=ARM_EFFORT,
        velocity_limit_sim=ARM_VEL,
        stiffness=ARM_STIFFNESS,
        damping=ARM_DAMPING,
    ),
    "fl_gripper": ImplicitActuatorCfg(
        joint_names_expr=["fl_joint[7-8]"],
        effort_limit_sim=GRIPPER_EFFORT,
        velocity_limit_sim=GRIPPER_VEL,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
    ),

    # front-right
    "fr_arm": ImplicitActuatorCfg(
        joint_names_expr=["fr_joint[1-6]"],
        effort_limit_sim=ARM_EFFORT,
        velocity_limit_sim=ARM_VEL,
        stiffness=ARM_STIFFNESS,
        damping=ARM_DAMPING,
    ),
    "fr_gripper": ImplicitActuatorCfg(
        joint_names_expr=["fr_joint[7-8]"],
        effort_limit_sim=GRIPPER_EFFORT,
        velocity_limit_sim=GRIPPER_VEL,
        stiffness=GRIPPER_STIFFNESS,
        damping=GRIPPER_DAMPING,
    ),
}


# -----------------------------
# FULL robot cfg（但 wheels 不驱动）
# -----------------------------
MOBILE_ALOHA_FULL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/lzg/IsaacLab/source/isaaclab_assets/data/maloha.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            # 原来 5.0 较激进；降低可减少“解穿透把轮子顶飞/滚走”
            max_depenetration_velocity=1.5,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # ✅ 关键：先关掉 self-collision 来排除轮子/底盘互相顶导致的“自走”
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
        # 如果你仍有接触抖动，可再加 contact_offset/rest_offset（按需再开）
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),

    # 初始关节位置：这里只初始化手臂+夹爪（轮子/脚轮保持 USD 默认）
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            **{f"{p}joint{i}": 0.0 for p in ("bl_", "br_", "fl_", "fr_") for i in range(1, 9)},
        },
    ),

    actuators=_ACTUATORS,
    soft_joint_pos_limit_factor=1.0,
)


# -----------------------------
# HIGH PD（用于 IK/teleop 更跟手）
# -----------------------------
MOBILE_ALOHA_FULL_HIGH_PD_CFG = MOBILE_ALOHA_FULL_CFG.copy()

for group_name in ("bl_arm", "br_arm", "fl_arm", "fr_arm"):
    a = MOBILE_ALOHA_FULL_HIGH_PD_CFG.actuators[group_name]
    a.stiffness = ARM_STIFFNESS
    a.damping = ARM_DAMPING
    a.effort_limit_sim = ARM_EFFORT



# for group_name in ("bl_arm", "br_arm", "fl_arm", "fr_arm"):
#     MOBILE_ALOHA_FULL_HIGH_PD_CFG.actuators[group_name].stiffness = 400.0
#     MOBILE_ALOHA_FULL_HIGH_PD_CFG.actuators[group_name].damping = 60.0


# # Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# # All rights reserved.
# #
# # SPDX-License-Identifier: BSD-3-Clause

# """Configuration for the MALOHA Emika robots.

# The following configurations are available:

# * :obj:`MALOHA_PANDA_CFG`: MALOHA Emika Panda robot with Panda hand
# * :obj:`MALOHA_PANDA_HIGH_PD_CFG`: MALOHA Emika Panda robot with Panda hand with stiffer PD control

# Reference: https://github.com/MALOHAemika/MALOHA_ros
# """

# import isaaclab.sim as sim_utils
# from isaaclab.actuators import ImplicitActuatorCfg
# from isaaclab.assets.articulation import ArticulationCfg
# from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

# ##
# # Configuration
# ##
# # ------30号---我留下的备注。
# # robot_usd = "kinova_robotiq.usd"
# # usd_dir_path = os.path.join(BASE_DIR, "../usd/")
# # /home/lzg/codes/sim_usd_assets
# # Copyright (c) 2022-2025, The Isaac Lab Project Developers
# # SPDX-License-Identifier: BSD-3-Clause

# """Configuration for the Mobile ALOHA robot (left arm only control).

# This file defines an ArticulationCfg for a Mobile ALOHA USD asset, but
# ONLY sets actuators for the left arm joints (and optionally left gripper).
# Other joints (base, right arm, tray, wheels, etc.) will exist in the articulation
# but will not be directly actuated by this config unless you add actuators for them.
# """

# import isaaclab.sim as sim_utils
# from isaaclab.actuators import ImplicitActuatorCfg
# from isaaclab.assets.articulation import ArticulationCfg


# # -----------------------------------------------------------------------------
# # 1) 你需要确认你的 USD 里左臂关节命名：
# #    你之前提到：fl_joint{1-8}
# #    - fl_joint1..fl_joint6: 6DoF arm
# #    - fl_joint7..fl_joint8: gripper two fingers (如果是这样)
# #
# # 2) 你还需要确认“整机 articulation 根 prim”是否就是这个 usd 的 defaultPrim。
# #    一般来说不用写在这里，EnvCfg 里 replace(prim_path=...) 就行。
# # -----------------------------------------------------------------------------


# MOBILE_ALOHA_LEFTARM_CFG = ArticulationCfg(
#     spawn=sim_utils.UsdFileCfg(
#         # ✅ 改成你的 mobile aloha USD 路径
#         usd_path="/home/lzg/IsaacLab/source/isaaclab_assets/data/maloha.usd",
#         activate_contact_sensors=False,
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             disable_gravity=False,
#             max_depenetration_velocity=5.0,
#         ),
#         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#             enabled_self_collisions=True,
#             solver_position_iteration_count=8,
#             solver_velocity_iteration_count=0,
#         ),
#         # 如果你需要更稳定的接触，可按需打开：
#         # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
#     ),

#     # -------------------------------------------------------------------------
#     # 初始姿态：这里只写“你想设定的关节”，没写的关节会保持 USD/URDF 默认值
#     # -------------------------------------------------------------------------
#     init_state=ArticulationCfg.InitialStateCfg(
#         joint_pos={
#             # ✅ 左臂 6DoF：给一个“中性姿态”，先用 0 也能跑
#             "fl_joint1": 0.0,
#             "fl_joint2": 0.0,
#             "fl_joint3": 0.0,
#             "fl_joint4": 0.0,
#             "fl_joint5": 0.0,
#             "fl_joint6": 0.0,

#             # ✅ 左夹爪：如果你的 gripper 确实是 fl_joint7/8
#             # 如果不是（比如叫 fl_finger_left / fl_finger_right），你要改名字
#             "fl_joint7": 0.00,
#             "fl_joint8": -0.00,
#         },
#     ),

#     # -------------------------------------------------------------------------
#     # actuators：只给左臂的关节配置 PD（ImplicitActuator）
#     # 你可以先用一个组把 1-6 全包起来，然后夹爪单独一组
#     # -------------------------------------------------------------------------
#     actuators={
#         # 左臂（1-6）
#         "left_arm": ImplicitActuatorCfg(
#             joint_names_expr=["fl_joint[1-6]"],
#             # 下面这些数值你需要后续按实际机器人调：
#             # - effort_limit_sim: 最大力矩（或等效 effort）
#             # - velocity_limit_sim: 最大关节速度
#             # - stiffness/damping: PD 参数（越大越硬，越小越软）
#             effort_limit_sim=50.0,
#             velocity_limit_sim=2.0,
#             stiffness=40.0,
#             damping=2.0,
#         ),

#         # 左夹爪（7-8）
#         "left_gripper": ImplicitActuatorCfg(
#             joint_names_expr=["fl_joint[7-8]"],
#             effort_limit_sim=50.0,
#             velocity_limit_sim=0.2,
#             stiffness=2e3,
#             damping=1e2,
#         ),
#     },

#     soft_joint_pos_limit_factor=1.0,
# )

# """Configuration of Mobile ALOHA robot with actuators set for left arm only."""


# # -----------------------------------------------------------------------------
# # 高 PD 版本：用于 IK / teleop 追踪更稳（不是必须）
# # -----------------------------------------------------------------------------
# MOBILE_ALOHA_LEFTARM_HIGH_PD_CFG = MOBILE_ALOHA_LEFTARM_CFG.copy()

# # ⚠️ 一般不建议关重力（除非你只是做纯 IK 演示）
# # MOBILE_ALOHA_LEFTARM_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True

# MOBILE_ALOHA_LEFTARM_HIGH_PD_CFG.actuators["left_arm"].stiffness = 400.0
# MOBILE_ALOHA_LEFTARM_HIGH_PD_CFG.actuators["left_arm"].damping = 80.0
