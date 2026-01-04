# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from dataclasses import MISSING

# from isaaclab_tasks.manager_based.manipulation.injection.stab_path_filter import SimpleIKHelper
import isaaclab.sim as sim_utils
import torch
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
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)
        ),
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
#
#  先注释了,我要测试一个点加成 8 点把
candidate_points = [(0.16, 0.0, 0.29, -1.57, 0, -1.57)]


# candidate_points = [(0.16, 0.0, 0.29, -1.57, 0, -1.57), (0.16, 0.003, 0.29, -1.57, 0, -1.553), (0.16, 0.006, 0.29, -1.57, 0, -1.535), (0.16, 0.008, 0.29, -1.57, 0, -1.518), (0.16, 0.011, 0.29, -1.57, 0, -1.5), (0.159, 0.014, 0.29, -1.57, 0, -1.483), (0.159, 0.017, 0.29, -1.57, 0, -1.465), (0.159, 0.019, 0.29, -1.57, 0, -1.448), (0.158, 0.022, 0.29, -1.57, 0, -1.43), (0.158, 0.025, 0.29, -1.57, 0, -1.413), (0.158, 0.028, 0.29, -1.57, 0, -1.395), (0.157, 0.031, 0.29, -1.57, 0, -1.378), (0.157, 0.033, 0.29, -1.57, 0, -1.361), (0.156, 0.036, 0.29, -1.57, 0, -1.343), (0.155, 0.039, 0.29, -1.57, 0, -1.326), (0.155, 0.041, 0.29, -1.57, 0, -1.308), (0.154, 0.044, 0.29, -1.57, 0, -1.291), (0.153, 0.047, 0.29, -1.57, 0, -1.273), (0.152, 0.049, 0.29, -1.57, 0, -1.256), (0.151, 0.052, 0.29, -1.57, 0, -1.238), (0.15, 0.055, 0.29, -1.57, 0, -1.221), (0.149, 0.057, 0.29, -1.57, 0, -1.203), (0.148, 0.06, 0.29, -1.57, 0, -1.186), (0.147, 0.063, 0.29, -1.57, 0, -1.169), (0.146, 0.065, 0.29, -1.57, 0, -1.151), (0.145, 0.068, 0.29, -1.57, 0, -1.134), (0.144, 0.07, 0.29, -1.57, 0, -1.116), (0.143, 0.073, 0.29, -1.57, 0, -1.099), (0.141, 0.075, 0.29, -1.57, 0, -1.081), (0.14, 0.078, 0.29, -1.57, 0, -1.064), (0.139, 0.08, 0.29, -1.57, 0, -1.046), (0.137, 0.082, 0.29, -1.57, 0, -1.029), (0.136, 0.085, 0.29, -1.57, 0, -1.011), (0.134, 0.087, 0.29, -1.57, 0, -0.994), (0.133, 0.089, 0.29, -1.57, 0, -0.977), (0.131, 0.092, 0.29, -1.57, 0, -0.959), (0.129, 0.094, 0.29, -1.57, 0, -0.942), (0.128, 0.096, 0.29, -1.57, 0, -0.924), (0.126, 0.099, 0.29, -1.57, 0, -0.907), (0.124, 0.101, 0.29, -1.57, 0, -0.889), (0.123, 0.103, 0.29, -1.57, 0, -0.872), (0.121, 0.105, 0.29, -1.57, 0, -0.854), (0.119, 0.107, 0.29, -1.57, 0, -0.837), (0.117, 0.109, 0.29, -1.57, 0, -0.82), (0.115, 0.111, 0.29, -1.57, 0, -0.802), (0.113, 0.113, 0.29, -1.57, 0, -0.785), (0.111, 0.115, 0.29, -1.57, 0, -0.767), (0.109, 0.117, 0.29, -1.57, 0, -0.75), (0.107, 0.119, 0.29, -1.57, 0, -0.732), (0.105, 0.121, 0.29, -1.57, 0, -0.715), (0.103, 0.123, 0.29, -1.57, 0, -0.697), (0.101, 0.124, 0.29, -1.57, 0, -0.68), (0.099, 0.126, 0.29, -1.57, 0, -0.662), (0.096, 0.128, 0.29, -1.57, 0, -0.645), (0.094, 0.129, 0.29, -1.57, 0, -0.628), (0.092, 0.131, 0.29, -1.57, 0, -0.61), (0.089, 0.133, 0.29, -1.57, 0, -0.593), (0.087, 0.134, 0.29, -1.57, 0, -0.575), (0.085, 0.136, 0.29, -1.57, 0, -0.558), (0.082, 0.137, 0.29, -1.57, 0, -0.54), (0.08, 0.139, 0.29, -1.57, 0, -0.523), (0.078, 0.14, 0.29, -1.57, 0, -0.505), (0.075, 0.141, 0.29, -1.57, 0, -0.488), (0.073, 0.143, 0.29, -1.57, 0, -0.47), (0.07, 0.144, 0.29, -1.57, 0, -0.453), (0.068, 0.145, 0.29, -1.57, 0, -0.436), (0.065, 0.146, 0.29, -1.57, 0, -0.418), (0.063, 0.147, 0.29, -1.57, 0, -0.401), (0.06, 0.148, 0.29, -1.57, 0, -0.383), (0.057, 0.149, 0.29, -1.57, 0, -0.366), (0.055, 0.15, 0.29, -1.57, 0, -0.348), (0.052, 0.151, 0.29, -1.57, 0, -0.331), (0.049, 0.152, 0.29, -1.57, 0, -0.313), (0.047, 0.153, 0.29, -1.57, 0, -0.296), (0.044, 0.154, 0.29, -1.57, 0, -0.278), (0.041, 0.155, 0.29, -1.57, 0, -0.261), (0.039, 0.155, 0.29, -1.57, 0, -0.244), (0.036, 0.156, 0.29, -1.57, 0, -0.226), (0.033, 0.157, 0.29, -1.57, 0, -0.209), (0.031, 0.157, 0.29, -1.57, 0, -0.191), (0.028, 0.158, 0.29, -1.57, 0, -0.174), (0.025, 0.158, 0.29, -1.57, 0, -0.156), (0.022, 0.158, 0.29, -1.57, 0, -0.139), (0.019, 0.159, 0.29, -1.57, 0, -0.121), (0.017, 0.159, 0.29, -1.57, 0, -0.104), (0.014, 0.159, 0.29, -1.57, 0, -0.086), (0.011, 0.16, 0.29, -1.57, 0, -0.069), (0.008, 0.16, 0.29, -1.57, 0, -0.052), (0.006, 0.16, 0.29, -1.57, 0, -0.034), (0.003, 0.16, 0.29, -1.57, 0, -0.017), (0.0, 0.16, 0.29, -1.57, 0, 0.001), (-0.003, 0.16, 0.29, -1.57, 0, 0.018), (-0.006, 0.16, 0.29, -1.57, 0, 0.036), (-0.008, 0.16, 0.29, -1.57, 0, 0.053), (-0.011, 0.16, 0.29, -1.57, 0, 0.071), (-0.014, 0.159, 0.29, -1.57, 0, 0.088), (-0.017, 0.159, 0.29, -1.57, 0, 0.106), (-0.019, 0.159, 0.29, -1.57, 0, 0.123), (-0.022, 0.158, 0.29, -1.57, 0, 0.14), (-0.025, 0.158, 0.29, -1.57, 0, 0.158), (-0.028, 0.158, 0.29, -1.57, 0, 0.175), (-0.031, 0.157, 0.29, -1.57, 0, 0.193), (-0.033, 0.157, 0.29, -1.57, 0, 0.21), (-0.036, 0.156, 0.29, -1.57, 0, 0.228), (-0.039, 0.155, 0.29, -1.57, 0, 0.245), (-0.041, 0.155, 0.29, -1.57, 0, 0.263), (-0.044, 0.154, 0.29, -1.57, 0, 0.28), (-0.047, 0.153, 0.29, -1.57, 0, 0.298), (-0.049, 0.152, 0.29, -1.57, 0, 0.315), (-0.052, 0.151, 0.29, -1.57, 0, 0.332), (-0.055, 0.15, 0.29, -1.57, 0, 0.35), (-0.057, 0.149, 0.29, -1.57, 0, 0.367), (-0.06, 0.148, 0.29, -1.57, 0, 0.385), (-0.063, 0.147, 0.29, -1.57, 0, 0.402), (-0.065, 0.146, 0.29, -1.57, 0, 0.42), (-0.068, 0.145, 0.29, -1.57, 0, 0.437), (-0.07, 0.144, 0.29, -1.57, 0, 0.455), (-0.073, 0.143, 0.29, -1.57, 0, 0.472), (-0.075, 0.141, 0.29, -1.57, 0, 0.489), (-0.078, 0.14, 0.29, -1.57, 0, 0.507), (-0.08, 0.139, 0.29, -1.57, 0, 0.524), (-0.082, 0.137, 0.29, -1.57, 0, 0.542), (-0.085, 0.136, 0.29, -1.57, 0, 0.559), (-0.087, 0.134, 0.29, -1.57, 0, 0.577), (-0.089, 0.133, 0.29, -1.57, 0, 0.594), (-0.092, 0.131, 0.29, -1.57, 0, 0.612), (-0.094, 0.129, 0.29, -1.57, 0, 0.629), (-0.096, 0.128, 0.29, -1.57, 0, 0.647), (-0.099, 0.126, 0.29, -1.57, 0, 0.664), (-0.101, 0.124, 0.29, -1.57, 0, 0.681), (-0.103, 0.123, 0.29, -1.57, 0, 0.699), (-0.105, 0.121, 0.29, -1.57, 0, 0.716), (-0.107, 0.119, 0.29, -1.57, 0, 0.734), (-0.109, 0.117, 0.29, -1.57, 0, 0.751), (-0.111, 0.115, 0.29, -1.57, 0, 0.769), (-0.113, 0.113, 0.29, -1.57, 0, 0.786), (-0.115, 0.111, 0.29, -1.57, 0, 0.804), (-0.117, 0.109, 0.29, -1.57, 0, 0.821), (-0.119, 0.107, 0.29, -1.57, 0, 0.839), (-0.121, 0.105, 0.29, -1.57, 0, 0.856), (-0.123, 0.103, 0.29, -1.57, 0, 0.873), (-0.124, 0.101, 0.29, -1.57, 0, 0.891), (-0.126, 0.099, 0.29, -1.57, 0, 0.908), (-0.128, 0.096, 0.29, -1.57, 0, 0.926), (-0.129, 0.094, 0.29, -1.57, 0, 0.943), (-0.131, 0.092, 0.29, -1.57, 0, 0.961), (-0.133, 0.089, 0.29, -1.57, 0, 0.978), (-0.134, 0.087, 0.29, -1.57, 0, 0.996), (-0.136, 0.085, 0.29, -1.57, 0, 1.013), (-0.137, 0.082, 0.29, -1.57, 0, 1.031), (-0.139, 0.08, 0.29, -1.57, 0, 1.048), (-0.14, 0.078, 0.29, -1.57, 0, 1.065), (-0.141, 0.075, 0.29, -1.57, 0, 1.083), (-0.143, 0.073, 0.29, -1.57, 0, 1.1), (-0.144, 0.07, 0.29, -1.57, 0, 1.118), (-0.145, 0.068, 0.29, -1.57, 0, 1.135), (-0.146, 0.065, 0.29, -1.57, 0, 1.153), (-0.147, 0.063, 0.29, -1.57, 0, 1.17), (-0.148, 0.06, 0.29, -1.57, 0, 1.188), (-0.149, 0.057, 0.29, -1.57, 0, 1.205), (-0.15, 0.055, 0.29, -1.57, 0, 1.223), (-0.151, 0.052, 0.29, -1.57, 0, 1.24), (-0.152, 0.049, 0.29, -1.57, 0, 1.257), (-0.153, 0.047, 0.29, -1.57, 0, 1.275), (-0.154, 0.044, 0.29, -1.57, 0, 1.292), (-0.155, 0.041, 0.29, -1.57, 0, 1.31), (-0.155, 0.039, 0.29, -1.57, 0, 1.327), (-0.156, 0.036, 0.29, -1.57, 0, 1.345), (-0.157, 0.033, 0.29, -1.57, 0, 1.362), (-0.157, 0.031, 0.29, -1.57, 0, 1.38), (-0.158, 0.028, 0.29, -1.57, 0, 1.397), (-0.158, 0.025, 0.29, -1.57, 0, 1.415), (-0.158, 0.022, 0.29, -1.57, 0, 1.432), (-0.159, 0.019, 0.29, -1.57, 0, 1.449), (-0.159, 0.017, 0.29, -1.57, 0, 1.467), (-0.159, 0.014, 0.29, -1.57, 0, 1.484), (-0.16, 0.011, 0.29, -1.57, 0, 1.502), (-0.16, 0.008, 0.29, -1.57, 0, 1.519), (-0.16, 0.006, 0.29, -1.57, 0, 1.537), (-0.16, 0.003, 0.29, -1.57, 0, 1.554), (-0.16, 0.0, 0.29, -1.57, 0, 1.572), (-0.16, -0.003, 0.29, -1.57, 0, 1.589), (-0.16, -0.006, 0.29, -1.57, 0, 1.606), (-0.16, -0.008, 0.29, -1.57, 0, 1.624), (-0.16, -0.011, 0.29, -1.57, 0, 1.641), (-0.159, -0.014, 0.29, -1.57, 0, 1.659), (-0.159, -0.017, 0.29, -1.57, 0, 1.676), (-0.159, -0.019, 0.29, -1.57, 0, 1.694), (-0.158, -0.022, 0.29, -1.57, 0, 1.711), (-0.158, -0.025, 0.29, -1.57, 0, 1.729), (-0.158, -0.028, 0.29, -1.57, 0, 1.746), (-0.157, -0.031, 0.29, -1.57, 0, 1.764), (-0.157, -0.033, 0.29, -1.57, 0, 1.781), (-0.156, -0.036, 0.29, -1.57, 0, 1.798), (-0.155, -0.039, 0.29, -1.57, 0, 1.816), (-0.155, -0.041, 0.29, -1.57, 0, 1.833), (-0.154, -0.044, 0.29, -1.57, 0, 1.851), (-0.153, -0.047, 0.29, -1.57, 0, 1.868), (-0.152, -0.049, 0.29, -1.57, 0, 1.886), (-0.151, -0.052, 0.29, -1.57, 0, 1.903), (-0.15, -0.055, 0.29, -1.57, 0, 1.921), (-0.149, -0.057, 0.29, -1.57, 0, 1.938), (-0.148, -0.06, 0.29, -1.57, 0, 1.956), (-0.147, -0.063, 0.29, -1.57, 0, 1.973), (-0.146, -0.065, 0.29, -1.57, 0, 1.99), (-0.145, -0.068, 0.29, -1.57, 0, 2.008), (-0.144, -0.07, 0.29, -1.57, 0, 2.025), (-0.143, -0.073, 0.29, -1.57, 0, 2.043), (-0.141, -0.075, 0.29, -1.57, 0, 2.06), (-0.14, -0.078, 0.29, -1.57, 0, 2.078), (-0.139, -0.08, 0.29, -1.57, 0, 2.095), (-0.137, -0.082, 0.29, -1.57, 0, 2.113), (-0.136, -0.085, 0.29, -1.57, 0, 2.13), (-0.134, -0.087, 0.29, -1.57, 0, 2.148), (-0.133, -0.089, 0.29, -1.57, 0, 2.165), (-0.131, -0.092, 0.29, -1.57, 0, 2.182), (-0.129, -0.094, 0.29, -1.57, 0, 2.2), (-0.128, -0.096, 0.29, -1.57, 0, 2.217), (-0.126, -0.099, 0.29, -1.57, 0, 2.235), (-0.124, -0.101, 0.29, -1.57, 0, 2.252), (-0.123, -0.103, 0.29, -1.57, 0, 2.27), (-0.121, -0.105, 0.29, -1.57, 0, 2.287), (-0.119, -0.107, 0.29, -1.57, 0, 2.305), (-0.117, -0.109, 0.29, -1.57, 0, 2.322), (-0.115, -0.111, 0.29, -1.57, 0, 2.34), (-0.113, -0.113, 0.29, -1.57, 0, 2.357), (-0.111, -0.115, 0.29, -1.57, 0, 2.374), (-0.109, -0.117, 0.29, -1.57, 0, 2.392), (-0.107, -0.119, 0.29, -1.57, 0, 2.409), (-0.105, -0.121, 0.29, -1.57, 0, 2.427), (-0.103, -0.123, 0.29, -1.57, 0, 2.444), (-0.101, -0.124, 0.29, -1.57, 0, 2.462), (-0.099, -0.126, 0.29, -1.57, 0, 2.479), (-0.096, -0.128, 0.29, -1.57, 0, 2.497), (-0.094, -0.129, 0.29, -1.57, 0, 2.514), (-0.092, -0.131, 0.29, -1.57, 0, 2.532), (-0.089, -0.133, 0.29, -1.57, 0, 2.549), (-0.087, -0.134, 0.29, -1.57, 0, 2.566), (-0.085, -0.136, 0.29, -1.57, 0, 2.584), (-0.082, -0.137, 0.29, -1.57, 0, 2.601), (-0.08, -0.139, 0.29, -1.57, 0, 2.619), (-0.078, -0.14, 0.29, -1.57, 0, 2.636), (-0.075, -0.141, 0.29, -1.57, 0, 2.654), (-0.073, -0.143, 0.29, -1.57, 0, 2.671), (-0.07, -0.144, 0.29, -1.57, 0, 2.689), (-0.068, -0.145, 0.29, -1.57, 0, 2.706), (-0.065, -0.146, 0.29, -1.57, 0, 2.724), (-0.063, -0.147, 0.29, -1.57, 0, 2.741), (-0.06, -0.148, 0.29, -1.57, 0, 2.758), (-0.057, -0.149, 0.29, -1.57, 0, 2.776), (-0.055, -0.15, 0.29, -1.57, 0, 2.793), (-0.052, -0.151, 0.29, -1.57, 0, 2.811), (-0.049, -0.152, 0.29, -1.57, 0, 2.828), (-0.047, -0.153, 0.29, -1.57, 0, 2.846), (-0.044, -0.154, 0.29, -1.57, 0, 2.863), (-0.041, -0.155, 0.29, -1.57, 0, 2.881), (-0.039, -0.155, 0.29, -1.57, 0, 2.898), (-0.036, -0.156, 0.29, -1.57, 0, 2.915), (-0.033, -0.157, 0.29, -1.57, 0, 2.933), (-0.031, -0.157, 0.29, -1.57, 0, 2.95), (-0.028, -0.158, 0.29, -1.57, 0, 2.968), (-0.025, -0.158, 0.29, -1.57, 0, 2.985), (-0.022, -0.158, 0.29, -1.57, 0, 3.003), (-0.019, -0.159, 0.29, -1.57, 0, 3.02), (-0.017, -0.159, 0.29, -1.57, 0, 3.038), (-0.014, -0.159, 0.29, -1.57, 0, 3.055), (-0.011, -0.16, 0.29, -1.57, 0, 3.073), (-0.008, -0.16, 0.29, -1.57, 0, 3.09), (-0.006, -0.16, 0.29, -1.57, 0, 3.107), (-0.003, -0.16, 0.29, -1.57, 0, 3.125), (-0.0, -0.16, 0.29, -1.57, 0, 3.142), (0.003, -0.16, 0.29, -1.57, 0, 3.16), (0.006, -0.16, 0.29, -1.57, 0, 3.177), (0.008, -0.16, 0.29, -1.57, 0, 3.195), (0.011, -0.16, 0.29, -1.57, 0, 3.212), (0.014, -0.159, 0.29, -1.57, 0, 3.23), (0.017, -0.159, 0.29, -1.57, 0, 3.247), (0.019, -0.159, 0.29, -1.57, 0, 3.265), (0.022, -0.158, 0.29, -1.57, 0, 3.282), (0.025, -0.158, 0.29, -1.57, 0, 3.299), (0.028, -0.158, 0.29, -1.57, 0, 3.317), (0.031, -0.157, 0.29, -1.57, 0, 3.334), (0.033, -0.157, 0.29, -1.57, 0, 3.352), (0.036, -0.156, 0.29, -1.57, 0, 3.369), (0.039, -0.155, 0.29, -1.57, 0, 3.387), (0.041, -0.155, 0.29, -1.57, 0, 3.404), (0.044, -0.154, 0.29, -1.57, 0, 3.422), (0.047, -0.153, 0.29, -1.57, 0, 3.439), (0.049, -0.152, 0.29, -1.57, 0, 3.457), (0.052, -0.151, 0.29, -1.57, 0, 3.474), (0.055, -0.15, 0.29, -1.57, 0, 3.491), (0.057, -0.149, 0.29, -1.57, 0, 3.509), (0.06, -0.148, 0.29, -1.57, 0, 3.526), (0.063, -0.147, 0.29, -1.57, 0, 3.544), (0.065, -0.146, 0.29, -1.57, 0, 3.561), (0.068, -0.145, 0.29, -1.57, 0, 3.579), (0.07, -0.144, 0.29, -1.57, 0, 3.596), (0.073, -0.143, 0.29, -1.57, 0, 3.614), (0.075, -0.141, 0.29, -1.57, 0, 3.631), (0.078, -0.14, 0.29, -1.57, 0, 3.649), (0.08, -0.139, 0.29, -1.57, 0, 3.666), (0.082, -0.137, 0.29, -1.57, 0, 3.683), (0.085, -0.136, 0.29, -1.57, 0, 3.701), (0.087, -0.134, 0.29, -1.57, 0, 3.718), (0.089, -0.133, 0.29, -1.57, 0, 3.736), (0.092, -0.131, 0.29, -1.57, 0, 3.753), (0.094, -0.129, 0.29, -1.57, 0, 3.771), (0.096, -0.128, 0.29, -1.57, 0, 3.788), (0.099, -0.126, 0.29, -1.57, 0, 3.806), (0.101, -0.124, 0.29, -1.57, 0, 3.823), (0.103, -0.123, 0.29, -1.57, 0, 3.841), (0.105, -0.121, 0.29, -1.57, 0, 3.858), (0.107, -0.119, 0.29, -1.57, 0, 3.875), (0.109, -0.117, 0.29, -1.57, 0, 3.893), (0.111, -0.115, 0.29, -1.57, 0, 3.91), (0.113, -0.113, 0.29, -1.57, 0, 3.928), (0.115, -0.111, 0.29, -1.57, 0, 3.945), (0.117, -0.109, 0.29, -1.57, 0, 3.963), (0.119, -0.107, 0.29, -1.57, 0, 3.98), (0.121, -0.105, 0.29, -1.57, 0, 3.998), (0.123, -0.103, 0.29, -1.57, 0, 4.015), (0.124, -0.101, 0.29, -1.57, 0, 4.033), (0.126, -0.099, 0.29, -1.57, 0, 4.05), (0.128, -0.096, 0.29, -1.57, 0, 4.067), (0.129, -0.094, 0.29, -1.57, 0, 4.085), (0.131, -0.092, 0.29, -1.57, 0, 4.102), (0.133, -0.089, 0.29, -1.57, 0, 4.12), (0.134, -0.087, 0.29, -1.57, 0, 4.137), (0.136, -0.085, 0.29, -1.57, 0, 4.155), (0.137, -0.082, 0.29, -1.57, 0, 4.172), (0.139, -0.08, 0.29, -1.57, 0, 4.19), (0.14, -0.078, 0.29, -1.57, 0, 4.207), (0.141, -0.075, 0.29, -1.57, 0, 4.224), (0.143, -0.073, 0.29, -1.57, 0, 4.242), (0.144, -0.07, 0.29, -1.57, 0, 4.259), (0.145, -0.068, 0.29, -1.57, 0, 4.277), (0.146, -0.065, 0.29, -1.57, 0, 4.294), (0.147, -0.063, 0.29, -1.57, 0, 4.312), (0.148, -0.06, 0.29, -1.57, 0, 4.329), (0.149, -0.057, 0.29, -1.57, 0, 4.347), (0.15, -0.055, 0.29, -1.57, 0, 4.364), (0.151, -0.052, 0.29, -1.57, 0, 4.382), (0.152, -0.049, 0.29, -1.57, 0, 4.399), (0.153, -0.047, 0.29, -1.57, 0, 4.416), (0.154, -0.044, 0.29, -1.57, 0, 4.434), (0.155, -0.041, 0.29, -1.57, 0, 4.451), (0.155, -0.039, 0.29, -1.57, 0, 4.469), (0.156, -0.036, 0.29, -1.57, 0, 4.486), (0.157, -0.033, 0.29, -1.57, 0, 4.504), (0.157, -0.031, 0.29, -1.57, 0, 4.521), (0.158, -0.028, 0.29, -1.57, 0, 4.539), (0.158, -0.025, 0.29, -1.57, 0, 4.556), (0.158, -0.022, 0.29, -1.57, 0, 4.574), (0.159, -0.019, 0.29, -1.57, 0, 4.591), (0.159, -0.017, 0.29, -1.57, 0, 4.608), (0.159, -0.014, 0.29, -1.57, 0, 4.626), (0.16, -0.011, 0.29, -1.57, 0, 4.643), (0.16, -0.008, 0.29, -1.57, 0, 4.661), (0.16, -0.006, 0.29, -1.57, 0, 4.678), (0.16, -0.003, 0.29, -1.57, 0, 4.696)]
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
    # 定义候选点列表

    # 初始化时传入 list
    ee_pose = mdp.ListPoseCommandCfg(
        candidate_poses=candidate_points,
        asset_name="robot",
        body_name="joint6_flange",
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
    )

    # ee_pose = mdp.UniformPoseCommandCfg(
    # ik_helper_path="actions.ik_filter_action.controller",
    # asset_name="robot",
    # body_name="joint6_flange",
    # resampling_time_range=(4.0, 4.0),

    # debug_vis=True,
    # ranges=mdp.UniformPoseCommandCfg.Ranges(
    #       # 这是一次点位中b点位置
    #       # 这是一次点位中b点位置
    #     roll=(1.57,1.57), # 转换四元数的代码固定了转换顺序,先 roll绕 x 轴

    #     pitch=(1.57,1.57), #  第二,绕 y 轴

    #     yaw=(3.14,3.14), # 第三,绕z 轴 + 90 度 , 1.57 + 1.57 = 3.14

    #     pos_y=(0.16,0.16),

    #     pos_x=(0.0,0.0),

    #     pos_z=(0.29,0.29),

    #     #  ------
    # 下面std是0.04
    # pos_x=(0.23,0.27),
    # pos_y=(0.0,0.0),
    # pos_z=(0.28,0.32),
    # -------
    # pos_x=(-0.2,0.2),
    # pos_y=(-0.2,0.2),
    # pos_z=(0.14,0.34),
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
    # -----上面是撞库的全范围
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
    #     ),
    # )

    # b_pose = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="joint6_flange",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #          roll=(-1.57,-1.57), # 转换四元数的代码固定了转换顺序,先 roll绕 x 轴

    #         pitch=(0,0), #  第二,绕 y 轴

    #         yaw=(0,0), # 第三,绕z 轴

    #         pos_x=(0.3,0.3),

    #         pos_y=(0.0,0.0),

    #         pos_z=(0.29,0.29),
    #     ),
    # )

    # c_pose = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="joint6_flange",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #                     # 这是一次点位中b点位置
    #         roll=(-1.57,-1.57), # 转换四元数的代码固定了转换顺序,先 roll绕 x 轴

    #         pitch=(0,0), #  第二,绕 y 轴

    #         yaw=(0,0), # 第三,绕z 轴

    #         pos_y=(0.8,0.8),

    #         pos_x=(0.0,0.0),

    #         pos_z=(0.29,0.29),
    #     ),
    # )

    # 注意把 body_name 填成你的 EEF（比如 joint6_flange）

    # b_pose = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="joint6_flange",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         roll=(1.57, 1.57),
    #         pitch=(math.pi, math.pi),
    #         yaw=(1.57, 1.57),
    #         pos_x=(25, 25),
    #         pos_y=(0.0, 0.0),
    #         pos_z=(0.3, 0.3),
    #     ),
    # )

    # c_pose = mdp.UniformPoseCommandCfg(
    #     asset_name="robot",
    #     body_name="joint6_flange",
    #     resampling_time_range=(4.0, 4.0),
    #     debug_vis=True,
    #     ranges=mdp.UniformPoseCommandCfg.Ranges(
    #         roll=(1.57, 1.57),
    #         pitch=(math.pi, math.pi),
    #         yaw=(1.57, 1.57),
    #         pos_x=(27, 27),
    #         pos_y=(0.0, 0.0),
    #         pos_z=(0.3, 0.3),
    #     ),
    # )
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
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        pose_command = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "ee_pose"}
        )
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


# -------------下面是 RewardsCfg 的注释保留版本版----------------
@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # task terms
    # end_effector_position_tracking = RewTerm(
    #     func=mdp.position_command_error,
    #     weight=-0.9,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
    #         "command_name": "ee_pose",
    #     },
    # )
    # 
    # 粗位置 tracking：改成“先高度，再平面”
    # end_effector_position_tracking = RewTerm(
    #     func=mdp.position_height_then_xy_error,
    #     weight=-0.9,   # 先沿用你原来的绝对值，有需要再微调
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
    #         "command_name": "ee_pose",
    #         "z_index": 2,     # 如果你的“高度”轴不是 world z，可以改
    #         "z_tol": 0.01,    # 高度 1cm 内认为“高度基本对齐”
    #         "lambda_xy": 1.0, # 平面误差强度，后面可以单独调
    #     },
    # )
    # 
    # 只管高度的惩罚
    end_effector_height_tracking = RewTerm(
        func=mdp.position_z_error,
        weight=-0.9,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
            "command_name": "ee_pose",
            "z_index": 2,
        },
    )

    # 只管 xy 的惩罚
    end_effector_xy_tracking = RewTerm(
        func=mdp.position_xy_error,
        weight=0.0,   # 前期先几乎不管 xy，后面用 Curriculum 拉上来
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
            "command_name": "ee_pose",
        },
    )

    ee_cyl_overshoot_penalty = RewTerm(
        func=mdp.ee_cylindrical_overshoot_penalty,
        # 先用比较温和的权重，防止把其他 reward 淹没
        weight=-20.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
            "command_name": "ee_pose",
            "margin": 0.02,   # 半径适当放宽一点
        },
    )

    end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.03,  # 先从 0.05~0.08 试；与你的 -0.4 粗惩罚做平衡
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "std": 0.05,
            "command_name": "ee_pose",
        },
    )
    end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.4,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "ee_pose",
        },
    )
    end_effector_orientation_tracking_fine_grained = RewTerm(
        func=mdp.orientation_command_error_tanh,
        weight=0.006,  # 先从 0.05~0.08 试；与你的 -0.4 粗惩罚做平衡
        params={
            "command_name": "ee_pose",
            "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
            "std": 0.0872664626,  # = 5° in rad；先松后紧
        },
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.001)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.001,
        params={"asset_cfg": SceneEntityCfg("robot")},
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
# 上面是11 月 1 号注释的, 保留.

# -------------上面是自己新建的两个reward 项目。-------
# 把bc注释掉了， 先不用。md
#     bc_line_reach_cmd = RewTerm(
#     func=mdp.bc_line_reach_term_cmd,   # 奖励函数：评估从 B→C 的直线可达性（外部运动学，不改仿真）
#     weight=1.0,
#     params={
#         # ===== 机器人与末端设置 =====
#         "asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"]),
#         # ^ 场景里机器人在 env.scene 的名字；body_names[0] 是用来判定“到达B”的末端连杆名

#         # ===== B/C 目标来源（命令名）=====
#         "b_command_name": "b_pose",    # 从 command_manager 读取名为 "b_pose" 的目标姿态作为 B
#         "c_command_name": "c_pose",    # 若为 None，则不用命令而是用下面的 offset 从 B 推出 C

#         # ===== 无 c 命令时，如何由 B 推出 C =====
#         "c_offset_in_tool": 0.04,      # 单位 m，沿“B 的工具坐标系 +X 方向”前推这段距离得到 C 的位置
#         "keep_c_orientation": True,    # True: C 的朝向 = B 的朝向；False: 用当前 EEF 实际朝向作为 C 的朝向
#         # 注：当 c_command_name 非 None 时，上面两项不会生效（优先使用 c 命令）

#         # ===== Pinocchio 运动学模型（仅 backend="pin" 时有效）=====
#         # "urdf_path": "/home/lzg/codes/ros2_package/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_with_camera_flange.urdf",
#         "urdf_path": "/home/lzg/codes/sim_usd_assets/robot/mycombot/mycobot_280_m5/mycobot_280_m5/mycobot_280_m5",
#         "urdf_eef_frame": "joint6_flange",  # URDF 中末端 frame 名（应与 body_names[0] 对应）
#         # 解析 URDF 里 package:// 的资源路径（网格/碰撞几何）。有需要再打开：
#         # "pin_package_dirs": [
#         #     "/home/lzg/codes/ros2_package/mycobot_ros2"
#         # ],

#         # ===== 自碰撞检查（可选，开启会更慢但更真实）=====
#         "enable_collision": False,     # True 则在影子 IK 过程中做纯运动学自碰撞检查
#         # "disable_adjacent_pairs": True, # 忽略父子/同一关节相邻连杆对，减少伪碰撞

#         # ===== 评估触发策略（推荐一次性奖励）=====
#         "debug_force_now": False,      # False：不每步评估；配合下一项只在“到达B”时触发一次
#         "trigger_once_after_b": True,  # True：首次满足“到达B”阈值的那一帧触发评估并给一次性奖励

#         # “到达 B”判定阈值（仅一次性触发模式使用；不传则用默认：1cm / 5°）
#         "b_reach_pos_tol": 0.01,                     # m，EEF 到 B 的位置误差阈值
#         "b_reach_rot_tol": math.radians(5.0),        # rad，EEF 到 B 的姿态误差阈值

#         # ===== B→C 直线路径可达性求解参数 =====
#         "steps": 4,                 # 航点数：1=只评估终点 C；>1=插值评估整条路径（更严格更慢）
#         "max_iters_per_wp": 20,    # 每个航点 DLS 迭代上限
#         "pos_tol": 0.2,             # 航点位姿误差阈值（m）；建议调参：先宽松，收敛后逐步收紧
#         "rot_tol": math.radians(5.0), # 航点姿态误差阈值（rad）；最终可收紧到 ~0.5°（math.radians(0.5)）
#         "dls_lambda2": 1e-4,        # DLS 阻尼；大一点更稳但可能慢些
#         "joint_limit": True,        # 影子 IK 迭代时夹在 URDF 关节限位内
#         "vel_limit": False,         # 若 True，会限制每步 dq 幅度（更稳、更慢）

#         # ===== 后端选择 =====
#         "backend": "none",          # "pin"：启用 Pin 评估并产生实际得分；"none"：直接返回 0（便于 GUI/无 Pin 环境调试）
#     },
# )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.05, "num_steps": 4500},
    )
    
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -0.001, "num_steps": 4500},
    )
    # 新增：逐渐“打开” xy 惩罚
    xy_tracking = CurrTerm(
        func=mdp.modify_reward_weight,
        params={
            "term_name": "end_effector_xy_tracking",  # 对应上面的 reward term 名字
            "weight": -0.9,      # 目标权重（后期想要的 xy 惩罚强度）
            "num_steps": 2000,   # 在多少个环境步之内从当前 weight 线性过渡到 -0.6
        },
    )

##
# Environment configuration
##


@configclass
class InjectionEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Injection end-effector pose tracking environment."""

    # Scene settings
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=4096, env_spacing=2.5)
    # scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=2048, env_spacing=2.5)
    scene: InjectionSceneCfg = InjectionSceneCfg(num_envs=8192, env_spacing=2.5)

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
