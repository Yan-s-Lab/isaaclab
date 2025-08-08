# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import numpy as np
# from scipy.spatial.transform import Rotation as R


from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_error_magnitude, quat_mul

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize tracking of the position error using L2-norm.

    The function computes the position error between the desired position (from the command) and the
    current position of the asset's body (in world frame). The position error is computed as the L2-norm
    of the difference between the desired and current positions.
    """
    # extract the asset (to enable type hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    # obtain the desired and current positions
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
    curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]  # type: ignore
    return torch.norm(curr_pos_w - des_pos_w, dim=1)


def position_command_error_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward tracking of the position using the tanh kernel.

    The function computes the position error between the desired position (from the command) and the
    current position of the asset's body (in world frame) and maps it with a tanh kernel.
    """
    # extract the asset (to enable type hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    # obtain the desired and current positions
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
    curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]  # type: ignore
    distance = torch.norm(curr_pos_w - des_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)


def orientation_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize tracking orientation error using shortest path.

    The function computes the orientation error between the desired orientation (from the command) and the
    current orientation of the asset's body (in world frame). The orientation error is computed as the shortest
    path between the desired and current orientations.
    """
    # extract the asset (to enable type hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    # obtain the desired and current orientations
    des_quat_b = command[:, 3:7]
    des_quat_w = quat_mul(asset.data.root_quat_w, des_quat_b)
    curr_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0]]  # type: ignore
    return quat_error_magnitude(curr_quat_w, des_quat_w)

# 这是我为了ik新建的reward项目，具体实现上可能与slides上的不同，但逻辑是OK的，跑不通很正常，调试就好了。
from typing import TYPE_CHECKING
from isaaclab.utils.math import combine_frame_transforms
from isaaclab.managers import SceneEntityCfg
import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def bc_line_reachability_reward(
    env: "ManagerBasedRLEnv",
    command_name_b: str,
    command_name_c: str,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    # ---- 初始化与自动清缓存（每个 episode 开始都清一次）----
    if not hasattr(env, "bc_checked"):
        env.bc_checked = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        env.bc_fraction = torch.zeros(env.num_envs, dtype=torch.float32, device=env.device)
    # 当某些 env 的 progress_buf == 0，表示新 episode：清理对应缓存
    if hasattr(env, "progress_buf"):
        reset_mask = (env.progress_buf == 0)
        if torch.any(reset_mask):
            env.bc_checked[reset_mask] = False
            env.bc_fraction[reset_mask] = 0.0
    # ----------------------------------------------------

    asset = env.scene[asset_cfg.name]

    # ---------- SAFE DEBUG PRINT (runs once) ----------
    if not hasattr(env, "_dbg_once_done"):
        env._dbg_once_done = False
    if not env._dbg_once_done:
        try:
            body_names = list(asset.data.body_names)
            body_id = int(asset_cfg.body_ids[0])
            name_ok = (0 <= body_id < len(body_names))
            curr_pos0 = asset.data.body_pos_w[0, body_id].detach().cpu().numpy() if name_ok else None

            cmd_b = env.command_manager.get_command(command_name_b)
            des_pos_b = cmd_b[:, :3]
            des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
            des_pos_b0 = des_pos_w[0].detach().cpu().numpy()

            print("\n==== DEBUG ONCE ====")
            print(f"num_envs: {env.num_envs}")
            print(f"asset name: {asset_cfg.name}")
            print(f"body_names (first 10): {body_names[:10]}")
            print(f"using body_id: {body_id}, body_name: {body_names[body_id] if name_ok else 'OUT_OF_RANGE'}")
            print(f"EEF curr_pos_w[env0]: {curr_pos0}")
            print(f"B target pos_w[env0] : {des_pos_b0}")
            print("====================\n")
        except Exception as e:
            print(f"[WARN] debug print failed: {e}")
        finally:
            env._dbg_once_done = True
    # ---------- /SAFE DEBUG PRINT ----------

    # 目标 b 位置（世界系）
    cmd_b = env.command_manager.get_command(command_name_b)
    des_pos_b = cmd_b[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)

    # 当前末端位置
    curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]
    reach_mask = torch.norm(curr_pos_w - des_pos_w, dim=1) < 0.005

    # 只在到达 b 且没检查过的 env 做一次“验证”
    todo = reach_mask & (~env.bc_checked)
    idxs = torch.nonzero(todo, as_tuple=False).squeeze(-1)

    if idxs.numel() > 0:
        cmd_c = env.command_manager.get_command(command_name_c)  # 这里只需取一次
        for i in idxs.tolist():
            des_pos_c = cmd_c[i, :3].unsqueeze(0)  # [1,3]（body系）
            des_pos_c_w, _ = combine_frame_transforms(
                asset.data.root_pos_w[i:i+1], asset.data.root_quat_w[i:i+1], des_pos_c
            )

            # ====== 这里先放最简“成功”占位 ======
            success = True
            # 以后你把这里替换为真实 IK 验证：
            # success = run_ik_verify(b_pos=..., c_pos=..., q_b=..., ...)
            # ===================================

            env.bc_fraction[i] = 1.0 if success else 0.0
            env.bc_checked[i] = True

    out = torch.zeros(env.num_envs, device=env.device)
    out[env.bc_checked] = env.bc_fraction[env.bc_checked]
    return out

# def bc_line_reachability_reward(
#     env: ManagerBasedRLEnv,
#     command_name_b: str,          # b 点命令名（得到 b 的末端位姿）
#     command_name_c: str,          # c 点命令名（得到 c 的末端位姿）
#     asset_cfg: SceneEntityCfg,     # 末端所在刚体
#     ee_link: str,
#     reach_pos_tol: float = 0.005,  # 判“到达 b”的阈值（米）
# ) -> torch.Tensor:
#     asset: RigidObject = env.scene[asset_cfg.name]
#     # b 目标位姿（世界系）
#     cmd_b = env.command_manager.get_command(command_name_b)  # [N, 7] or [N, 3/7] 视你实现
#     des_pos_b = cmd_b[:, :3]
#     des_pos_w, des_quat_w = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)

#     # 当前 EEF 位置（世界系）
#     curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]
#     reach_mask = torch.norm(curr_pos_w - des_pos_w, dim=1) < reach_pos_tol
#     # 只对“刚到 b 且未检查”的 env 做 IK
#     todo = reach_mask & (~env.bc_checked)

#     # 逐个 env 调用一次 bc_ik_verify（非 batched）
#     # 备注：你需要能拿到每个 env 的 b/c 姿态四元数（wxyz），以及 q_b
#     idxs = torch.nonzero(todo, as_tuple=False).squeeze(-1).tolist()
#     for i in idxs:
#         # 准备 b/c 位姿（世界系）
#         b_pos = des_pos_w[i].detach().cpu().numpy()
#         b_quat = des_quat_w[i].detach().cpu().numpy()    # 如果 combine 返回了
#         # c 目标位姿（世界系）——从命令管理器或你的缓冲里取
#         cmd_c = env.command_manager.get_command(command_name_c)
#         c_pos_b = cmd_c[i, :3]
#         c_pos_w, c_quat_w = combine_frame_transforms(
#             asset.data.root_pos_w[i:i+1], asset.data.root_quat_w[i:i+1], c_pos_b.unsqueeze(0)
#         )
#         c_pos = c_pos_w[0].detach().cpu().numpy()
#         c_quat = c_quat_w[0].detach().cpu().numpy()

#         # q_b：到达 b 时的关节（直接读当前关节即可）
#         q_b = env.scene[asset_cfg.name].data.joint_pos[i].clone()

#         res = bc_ik_verify(
#             robot=env.scene[asset_cfg.name].impl,   # 你的 Articulation 句柄
#             sim=env.scene.sim,
#             ee_link=ee_link,
#             b_pos=b_pos, b_quat_wxyz=b_quat,
#             c_pos=c_pos, c_quat_wxyz=c_quat,
#             q_b=q_b,
#             check_self_collision=False  # 你现在不考虑环境碰撞；自碰如需再开
#         )
#         env.bc_fraction[i] = float(res["fraction"])
#         env.bc_checked[i] = True

#     # 奖励：没到 b 或还没检查 → 0；检查过的直接用 fraction（或二值）
#     out = torch.zeros(env.num_envs, device=env.device)
#     out[env.bc_checked] = env.bc_fraction[env.bc_checked]
#     return out



# def path_deviation(
#     env: ManagerBasedRLEnv,
#     asset_cfg: SceneEntityCfg,
#     point_a: torch.Tensor,
#     point_b: torch.Tensor,
# ) -> torch.Tensor:
#     """Compute distance from end-effector to the ideal path (A-B line)."""
#     asset: RigidObject = env.scene[asset_cfg.name]
#     ee_pos = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]  # (num_envs, 3)

#     # 向量
#     vec_ab = point_b - point_a             # (3,)
#     vec_ap = ee_pos - point_a.unsqueeze(0)  # (num_envs, 3)

#     cross = torch.cross(vec_ap, vec_ab)    # (num_envs, 3)
#     num = torch.norm(cross, dim=1)         # (num_envs,)
#     denom = torch.norm(vec_ab) + 1e-6      # 标量
#     return num / denom  # 每个环境一个值


# prev_joint_vel = None  # 建议将其缓存为 env-level 变量

# def motion_smoothness(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
#     global prev_joint_vel
#     asset: RigidObject = env.scene[asset_cfg.name]
#     curr_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]  # (num_envs, num_joints)

#     if prev_joint_vel is None:
#         prev_joint_vel = curr_vel.clone()
#         return torch.zeros(curr_vel.shape[0], device=curr_vel.device)

#     diff = curr_vel - prev_joint_vel
#     prev_joint_vel = curr_vel.clone()
#     return torch.sum(diff**2, dim=1)  # L2-norm squared per env

# import numpy as np
# import torch
# from scipy.spatial.transform import Rotation as R
# from isaaclab.utils.math import combine_frame_transforms

# def ik_path_validity_reward(
#     env: ManagerBasedRLEnv,
#     asset_cfg: SceneEntityCfg,
#     command_name: str,
#     ik_solver,
#     needle_length: float = 0.05,
#     num_points: int = 5,
# ) -> torch.Tensor:
#     """
#     检查 b→c 路径上所有采样点是否 IK 可解，并打印 joint 解
#     """

#     # ✅ 获取机器人实体（假设 asset_cfg.name = "robot"）
#     asset: RigidObject = env.scene[asset_cfg.name]
#     articulation = asset._articulation

#     # ✅ 获取当前关节角（默认 env_id=0）
#     joint_positions = articulation.data.joint_pos[0].cpu().numpy().tolist()

#     # ✅ 从 command manager 获取 b 点的相对位姿（以 robot base 为参考）
#     command = env.command_manager.get_command(command_name)
#     b_pos_rel = command[0, :3].cpu().numpy()    # (x, y, z)
#     b_quat_rel = command[0, 3:7].cpu().numpy()  # (w, x, y, z)

#     # ✅ 转为 world 坐标系下的位姿
#     b_pos_world, b_quat_world = combine_frame_transforms(
#         asset.data.root_pos_w[0],
#         asset.data.root_quat_w[0],
#         torch.tensor(b_pos_rel, device=env.device).unsqueeze(0),
#         torch.tensor(b_quat_rel, device=env.device).unsqueeze(0),
#     )
#     b_pos = b_pos_world[0].cpu().numpy()
#     b_quat = b_quat_world[0].cpu().numpy()  # still [w, x, y, z]

#     # ✅ 获取针头方向（假设为局部 -Z）
#     rot = R.from_quat([b_quat[1], b_quat[2], b_quat[3], b_quat[0]])  # 转换为 (x, y, z, w)
#     needle_dir_world = rot.apply([0, 0, -1])

#     # ✅ 计算 c 点
#     c_pos = b_pos + needle_length * needle_dir_world

#     # ✅ 插值采样路径点
#     positions = [b_pos + (i / (num_points - 1)) * (c_pos - b_pos) for i in range(num_points)]

#     # ✅ 逐点调用 IK
#     print("\n[IK Path Validity Reward] 路径采样点：")
#     for i, pos in enumerate(positions):
#         ik_solver.set_start_state(joint_positions)
#         success, joint_angles = ik_solver.solve(pos, b_quat)
#         print(f"  点 {i+1}/{num_points}: {np.round(pos, 3)} → {'✅' if success else '❌'}")
#         if success:
#             print(f"    关节解: {np.round(joint_angles, 3)}")

#     # ✅ 暂时返回固定 reward
#     return torch.zeros(env.num_envs, device=env.device)
