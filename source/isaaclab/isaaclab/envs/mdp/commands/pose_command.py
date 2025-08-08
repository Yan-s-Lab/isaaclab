# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Sub-module containing command generators for pose tracking."""
from __future__ import annotations

import torch
import math
from isaaclab.utils.math import combine_frame_transforms

from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import combine_frame_transforms, compute_pose_error, quat_from_euler_xyz, quat_unique

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import UniformPoseCommandCfg


class UniformPoseCommand(CommandTerm):
    """Command generator for generating pose commands uniformly.

    The command generator generates poses by sampling positions uniformly within specified
    regions in cartesian space. For orientation, it samples uniformly the euler angles
    (roll-pitch-yaw) and converts them into quaternion representation (w, x, y, z).

    The position and orientation commands are generated in the base frame of the robot, and not the
    simulation world frame. This means that users need to handle the transformation from the
    base frame to the simulation world frame themselves.

    .. caution::

        Sampling orientations uniformly is not strictly the same as sampling euler angles uniformly.
        This is because rotations are defined by 3D non-Euclidean space, and the mapping
        from euler angles to rotations is not one-to-one.

    """

    cfg: UniformPoseCommandCfg
    """Configuration for the command generator."""

    def __init__(self, cfg: UniformPoseCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator class.

        Args:
            cfg: The configuration parameters for the command generator.
            env: The environment object.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # extract the robot and body index for which the command is generated
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        # create buffers
        # -- commands: (x, y, z, qw, qx, qy, qz) in root frame
        self.pose_command_b = torch.zeros(self.num_envs, 7, device=self.device)
        self.pose_command_b[:, 3] = 1.0
        self.pose_command_w = torch.zeros_like(self.pose_command_b)
        # -- metrics
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        msg = "UniformPoseCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired pose command. Shape is (num_envs, 7).

        The first three elements correspond to the position, followed by the quaternion orientation in (w, x, y, z).
        """
        return self.pose_command_b

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        # transform command from base frame to simulation world frame
        self.pose_command_w[:, :3], self.pose_command_w[:, 3:] = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pose_command_b[:, :3],
            self.pose_command_b[:, 3:],
        )
        # compute the error
        pos_error, rot_error = compute_pose_error(
            self.pose_command_w[:, :3],
            self.pose_command_w[:, 3:],
            self.robot.data.body_pos_w[:, self.body_idx],
            self.robot.data.body_quat_w[:, self.body_idx],
        )
        self.metrics["position_error"] = torch.norm(pos_error, dim=-1)
        self.metrics["orientation_error"] = torch.norm(rot_error, dim=-1)

    def _resample_command(self, env_ids: Sequence[int]):
        # sample new pose targets
        # -- position
        r = torch.empty(len(env_ids), device=self.device)
        self.pose_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        self.pose_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        self.pose_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)
        # -- orientation
        euler_angles = torch.zeros_like(self.pose_command_b[env_ids, :3])
        euler_angles[:, 0].uniform_(*self.cfg.ranges.roll)
        euler_angles[:, 1].uniform_(*self.cfg.ranges.pitch)
        euler_angles[:, 2].uniform_(*self.cfg.ranges.yaw)
        quat = quat_from_euler_xyz(euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2])
        # make sure the quaternion has real part as positive
        self.pose_command_b[env_ids, 3:] = quat_unique(quat) if self.cfg.make_quat_unique else quat

    def _update_command(self):
        pass

    def _set_debug_vis_impl(self, debug_vis: bool):
        # create markers if necessary for the first tome
        if debug_vis:
            if not hasattr(self, "goal_pose_visualizer"):
                # -- goal pose
                self.goal_pose_visualizer = VisualizationMarkers(self.cfg.goal_pose_visualizer_cfg)
                # -- current body pose
                self.current_pose_visualizer = VisualizationMarkers(self.cfg.current_pose_visualizer_cfg)
            # set their visibility to true
            self.goal_pose_visualizer.set_visibility(True)
            self.current_pose_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pose_visualizer"):
                self.goal_pose_visualizer.set_visibility(False)
                self.current_pose_visualizer.set_visibility(False)
    # def quat_to_euler(self,w, x, y, z):
    #     # XYZ 顺序的 roll-pitch-yaw
    #     # roll (x-axis rotation)
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x * x + y * y)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)
    #     # pitch (y-axis rotation)
    #     sinp = 2 * (w * y - z * x)
    #     pitch = math.asin(max(-1, min(1, sinp)))
    #     # yaw (z-axis rotation)
    #     siny_cosp = 2 * (w * z + x * y)
    #     cosy_cosp = 1 - 2 * (y * y + z * z)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)
    #     return roll, pitch, yaw

    def _debug_vis_callback(self, event):
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        # update the markers
        # -- goal pose
        self.goal_pose_visualizer.visualize(self.pose_command_w[:, :3], self.pose_command_w[:, 3:])
        # -- current body pose
        body_link_pose_w = self.robot.data.body_link_pose_w[:, self.body_idx]
        self.current_pose_visualizer.visualize(body_link_pose_w[:, :3], body_link_pose_w[:, 3:7])
        # todo，等待验证，可以删除。
        # 1) 世界坐标下的目标位置和四元数
        # world_pos, world_quat = combine_frame_transforms(
        #     self.robot.data.root_pos_w,
        #     self.robot.data.root_quat_w,
        #     self.pose_command_b[:, :3],
        #     self.pose_command_b[:, 3:],
        # )
        # # 只看第 0 号 env
        # pos0 = world_pos[0].cpu().tolist()
        # qw, qx, qy, qz = world_quat[0].cpu().tolist()

        # # 2) 可选：把四元数转成 Euler
        # roll, pitch, yaw = self.quat_to_euler(qw, qx, qy, qz)

        # # 3) 打印到 Omniverse Console
        # print(f"[DEBUG] Env0 target pos: x={pos0[0]:.3f}, y={pos0[1]:.3f}, z={pos0[2]:.3f}")
        # print(f"[DEBUG] Env0 orient quat: w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f}")
        # print(f"[DEBUG] Env0 Euler   : roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
        import torch
import numpy as np
import math
from isaaclab.envs.mdp.commands.pose_command import UniformPoseCommand
from isaaclab.utils.math import combine_frame_transforms

def quat_to_euler(w, x, y, z):
    # XYZ 顺序 roll–pitch–yaw
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll  = math.atan2(sinr_cosp, cosr_cosp)
    sinp    = 2*(w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw   = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class MyPoseCommand(UniformPoseCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        # 用来保存最近一次采样的 Euler
        self._last_euler = torch.zeros((self.num_envs, 3), device=self.device)

    # def _resample_command(self, env_ids):
    #     pass
        # 1. 复制父类的采样位置/四元数逻辑，但在采 Euler 时存下来
        # r = torch.empty(len(env_ids), device=self.device)
        # # 位置
        # self.pose_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.pos_x)
        # self.pose_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.pos_y)
        # self.pose_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.pos_z)
        # # Euler
        # e = torch.zeros((len(env_ids), 3), device=self.device)
        # e[:, 0].uniform_(*self.cfg.ranges.roll)
        # e[:, 1].uniform_(*self.cfg.ranges.pitch)
        # e[:, 2].uniform_(*self.cfg.ranges.yaw)
        # # 存下来
        # self._last_euler[env_ids] = e
        # # 转四元数
        # from isaaclab.utils.math import quat_from_euler_xyz, quat_unique
        # quat = quat_from_euler_xyz(e[:,0], e[:,1], e[:,2])
        # if self.cfg.make_quat_unique:
        #     quat = quat_unique(quat)
        # self.pose_command_b[env_ids, 3:] = quat

    def _debug_vis_callback(self, event):
        super()._debug_vis_callback(event)

        # 取第 0 个 env
        pos_b = self.pose_command_b[0, :3].cpu().tolist()
        qw, qx, qy, qz = self.pose_command_b[0, 3:].cpu().tolist()
        # 这就是 base-frame 下的 Euler
        roll_b, pitch_b, yaw_b = self._last_euler[0].cpu().tolist()
        # 也可以用 quat_to_euler 转 world-frame 下的 Euler
        world_pos, world_quat = combine_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pose_command_b[:, :3],
            self.pose_command_b[:, 3:],
        )
        wp = world_pos[0].cpu().tolist()
        wr = world_quat[0].cpu().tolist()
        w_roll, w_pitch, w_yaw = quat_to_euler(*wr)

         # 取第 0 个 env 的 root pose
        root_pos = self.robot.data.root_pos_w[0].cpu().tolist()    # [x, y, z]
        root_quat = self.robot.data.root_quat_w[0].cpu().tolist() # [w, x, y, z]

        print(f"[ROOT] base-frame orig in world → pos=({root_pos[0]:.3f}, {root_pos[1]:.3f}, {root_pos[2]:.3f}), "
            f"quat=(w={root_quat[0]:.3f}, x={root_quat[1]:.3f}, y={root_quat[2]:.3f}, z={root_quat[3]:.3f})")

        print(f"[BASE]  pos=( {pos_b[0]:.3f}, {pos_b[1]:.3f}, {pos_b[2]:.3f} ), "
              f"quart=(   qw={qw}, qx={qx}, qy={qy}, qz={qz})"
              f"euler=(roll={roll_b:.3f}, pitch={pitch_b:.3f}, yaw={yaw_b:.3f})")
        print(f"[WORLD] pos=( {wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f} ), "
              f"quat=(w={wr[0]:.3f},x={wr[1]:.3f},y={wr[2]:.3f},z={wr[3]:.3f}), "
              f"euler=(roll={w_roll:.3f}, pitch={w_pitch:.3f}, yaw={w_yaw:.3f})")
        
