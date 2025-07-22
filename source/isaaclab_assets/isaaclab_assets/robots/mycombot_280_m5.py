# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the elephant mycombot 280 m5 robots.

The following configurations are available:

* :obj:`MYCOMBOT_280_M5_CFG`: Mycombot 280 m5 robot with camera.
* :obj:`MYCOMBOT_280_M5_HIGH_PD_CFG`: Mycombot 280 m5 robot with camera with stiffer PD control

Reference: https://github.com/elephantrobotics/mycobot_ros
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import USER_BASE_DIR

##
# Configuration
##
# ------30号---我留下的备注。
# robot_usd = "kinova_robotiq.usd"
# usd_dir_path = os.path.join(BASE_DIR, "../usd/")
# /home/lzg/codes/sim_usd_assets

MYCOMBOT_280_M5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{USER_BASE_DIR}/robot/mycombot/m280_camera_flage_6_7_2025.usd",
        # approximation_shape="convexHull",  # 或 "capsule"、"box" # gpt的回复是低精度的任务重，推荐使用这个。 而在高精度任务重不需要。 那其实按照老师说的，打针动作是交给已有算法来完成的。那么其实机械臂rl只需要做定位就好了。
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # 下面这部分内容，是我为了解决phix错误后来增加的。如果后续需要精确任务，则需要注释掉，or改为mesh网格的复杂格式。
        # collision_props=sim_utils.CollisionPropertiesCfg(
        #     contact_offset=0.005,
        #     rest_offset=0.0,
        # ),

        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint2_to_joint1": 0.0,
            "joint3_to_joint2": 0.0,
            "joint4_to_joint3": 0.0,
            "joint5_to_joint4": 0.0,
            "joint6_to_joint5": 0.0,
            "joint6output_to_joint6": 0.0,
        },
    ),
    actuators={
        "mycombot_arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[2-6]_to_joint[1-6]","joint6output_to_joint6"],
            effort_limit_sim=5.0,
            velocity_limit_sim=1.5,
            stiffness=500.0,
            damping=80.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Mycombot 280 m5 robot."""


# FRANKA_PANDA_HIGH_PD_CFG = FRANKA_PANDA_CFG.copy()
# FRANKA_PANDA_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_shoulder"].damping = 80.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].stiffness = 400.0
# FRANKA_PANDA_HIGH_PD_CFG.actuators["panda_forearm"].damping = 80.0
# """Configuration of Franka Emika Panda robot with stiffer PD control.

# This configuration is useful for task-space control using differential IK.
# """

# ✅ 高刚度 PD 控制版本（用于插针/任务空间控制等高精度任务）
# 为什么下面数值是800和120 ，我也不知道，gpt给出的结论
MYCOBOT_280_M5_HIGH_PD_CFG = MYCOMBOT_280_M5_CFG.copy()
MYCOBOT_280_M5_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
MYCOBOT_280_M5_HIGH_PD_CFG.actuators["mycombot_arm"].stiffness = 800.0
MYCOBOT_280_M5_HIGH_PD_CFG.actuators["mycombot_arm"].damping = 120.0

"""Configuration of MyCobot 280 M5 robot with stiffer PD control.

This configuration is useful for task-space control (e.g., OSC, IK) or tasks that require precise and rigid end-effector behavior, such as injection or insertion.
"""
