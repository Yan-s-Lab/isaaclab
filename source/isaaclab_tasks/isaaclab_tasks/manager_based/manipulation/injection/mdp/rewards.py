# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING
import numpy as np

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_error_magnitude, quat_mul

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# =========================
# 基础位姿跟踪奖励（保留原有接口）
# =========================
def position_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """L2 位置误差（世界系）。"""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)  # 期望 Nx7 或 Nx3+（只取前3）
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
    curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]  # type: ignore
    return torch.norm(curr_pos_w - des_pos_w, dim=1)


def position_command_error_tanh(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """tanh 位置跟踪奖励（越接近越趋近 1）。"""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
    curr_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids[0]]  # type: ignore
    distance = torch.norm(curr_pos_w - des_pos_w, dim=1)
    return 1 - torch.tanh(distance / std)


def orientation_command_error(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """最短路径（四元数）姿态误差（世界系）。"""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_quat_b = command[:, 3:7]
    des_quat_w = quat_mul(asset.data.root_quat_w, des_quat_b)
    curr_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0]]  # type: ignore
    return quat_error_magnitude(curr_quat_w, des_quat_w)


# =========================
# 通用四元数/误差/关节限位工具
# =========================
def _quat_conj(q):  # [x,y,z,w] -> conj
    return torch.stack([-q[..., 0], -q[..., 1], -q[..., 2], q[..., 3]], -1)

def _quat_mul(q1, q2):  # [x,y,z,w] * [x,y,z,w]
    x1, y1, z1, w1 = q1.unbind(-1); x2, y2, z2, w2 = q2.unbind(-1)
    return torch.stack([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2], -1)

def _quat_normalize(q, eps=1e-9):
    return q / (q.norm(dim=-1, keepdim=True) + eps)

def _quat_to_rotvec(q):
    q = _quat_normalize(q)
    w = q[..., 3].clamp(-1, 1)
    ang = 2 * torch.acos(w)
    s = torch.sqrt(torch.clamp(1 - w*w, min=0)).unsqueeze(-1)
    axis = torch.where(s > 1e-8, q[..., :3] / s,
                       torch.tensor([0., 0., 0.], device=q.device, dtype=q.dtype))
    return axis * ang.unsqueeze(-1)

def _slerp(q0, q1, t):
    q0 = _quat_normalize(q0); q1 = _quat_normalize(q1)
    dot = (q0 * q1).sum(-1, keepdim=True)
    q1 = torch.where(dot < 0, -q1, q1); dot = dot.abs()
    TH = 0.9995
    omega = torch.acos(dot.clamp(-1+1e-8, 1-1e-8))
    s = torch.sin(omega)
    lerp = _quat_normalize((1 - t) * q0 + t * q1)
    s0 = torch.sin((1 - t) * omega) / (s + 1e-9)
    s1 = torch.sin(t * omega) / (s + 1e-9)
    slerp = _quat_normalize(s0*q0 + s1*q1)
    return torch.where(dot > TH, lerp, slerp)

def _pose_err_twist(cur_p, cur_q, tgt_p, tgt_q):
    dp = tgt_p - cur_p
    q_err = _quat_mul(_quat_conj(cur_q), tgt_q)
    drot = _quat_to_rotvec(q_err)
    return torch.cat([dp, drot], -1)  # (N,6)

def _dls_solve(J, err6, lambda2=1e-4):
    JT = J.transpose(-2, -1)
    JJt = J @ JT
    I = torch.eye(6, device=J.device, dtype=J.dtype).expand(JJt.shape)
    inv = torch.linalg.solve(JJt + lambda2 * I, err6.unsqueeze(-1))
    return (JT @ inv).squeeze(-1)

def _clamp_joint(q, lo, hi):
    return torch.max(torch.min(q, hi), lo)

def _extract_joint_limits(data, N, device):
    """返回 (q_lo, q_hi) 形状 (N,dof)。尽量兼容不同字段名/形状。"""
    # 直接 lower/upper
    if hasattr(data, "joint_pos_lower") and hasattr(data, "joint_pos_upper"):
        q_lo = torch.as_tensor(data.joint_pos_lower, device=device).float()
        q_hi = torch.as_tensor(data.joint_pos_upper, device=device).float()
        return (q_lo.unsqueeze(0).expand(N, -1) if q_lo.ndim == 1 else q_lo,
                q_hi.unsqueeze(0).expand(N, -1) if q_hi.ndim == 1 else q_hi)
    # 备用命名
    if hasattr(data, "joint_pos_limit_lower") and hasattr(data, "joint_pos_limit_upper"):
        q_lo = torch.as_tensor(data.joint_pos_limit_lower, device=device).float()
        q_hi = torch.as_tensor(data.joint_pos_limit_upper, device=device).float()
        return (q_lo.unsqueeze(0).expand(N, -1) if q_lo.ndim == 1 else q_lo,
                q_hi.unsqueeze(0).expand(N, -1) if q_hi.ndim == 1 else q_hi)
    # 合并在一个张量里
    for name in ("joint_pos_limits", "joint_limits"):
        if hasattr(data, name):
            lims = torch.as_tensor(getattr(data, name), device=device).float()
            if lims.ndim == 2 and lims.shape[0] == 2:      # (2,dof)
                q_lo, q_hi = lims[0], lims[1]
            elif lims.ndim >= 2 and lims.shape[-1] == 2:   # (...,dof,2)
                q_lo, q_hi = lims[..., 0], lims[..., 1]
            elif lims.ndim >= 2 and lims.shape[-2] == 2:   # (...,2,dof)
                q_lo, q_hi = lims[..., 0, :], lims[..., 1, :]
            else:
                raise RuntimeError(f"Unrecognized shape for {name}: {tuple(lims.shape)}")
            return (q_lo if q_lo.ndim == 2 else q_lo.expand(N, -1),
                    q_hi if q_hi.ndim == 2 else q_hi.expand(N, -1))
    # 兜底：不限位
    dof = torch.as_tensor(getattr(data, "joint_pos"), device=device).shape[-1]
    q_lo = torch.full((N, dof), -1e9, device=device)
    q_hi = torch.full((N, dof), +1e9, device=device)
    return q_lo, q_hi

def _get_body_id(data, body_name: str) -> int:
    names = None
    if hasattr(data, "body_names"):
        names = list(getattr(data, "body_names"))
    elif hasattr(data, "link_names"):
        names = list(getattr(data, "link_names"))
    if not names or body_name not in names:
        raise RuntimeError(f"Body '{body_name}' not found on robot.data.")
    return names.index(body_name)

def _quat_apply(q, v):
    """用四元数 q([x,y,z,w]) 旋转向量 v([3]或[N,3]) -> [N,3]"""
    if v.ndim == 1:
        v = v.unsqueeze(0).expand(q.shape[0], -1)
    v4 = torch.cat([v, torch.zeros_like(v[..., :1])], -1)
    r = _quat_mul(_quat_mul(q, v4), _quat_conj(q))
    return r[..., :3]
import torch
# =========================
# 打印pose的工具
# =========================
def debug_print_bce(env, asset_cfg, print_env_idx: int = 0):
    """打印 ee_pose / b_pose / c_pose 的命令位姿，以及当前 EEF 实际位姿（世界系）。
    print_env_idx: 打第几个子环境（默认第0个）"""
    robot = env.scene[asset_cfg.name]
    rdata = robot.data
    # 取 EEF 名称（若 cfg 里没填则回退到默认）
    eef_name = asset_cfg.body_names[0] if getattr(asset_cfg, "body_names", None) else "joint6_flange"
    eid = _get_body_id(rdata, eef_name)

    # ---- 读取命令位姿（世界系）----
    pos_ee_cmd, quat_ee_cmd = _pose_from_command_world(env, asset_cfg, "ee_pose")
    pos_b_cmd,  quat_b_cmd  = _pose_from_command_world(env, asset_cfg, "b_pose")
    pos_c_cmd,  quat_c_cmd  = _pose_from_command_world(env, asset_cfg, "c_pose")

    # ---- 读取当前实际 EEF 位姿（世界系）----
    pos_eef_now  = rdata.body_pos_w[:, eid]   # (num_envs, 3)
    quat_eef_now = rdata.body_quat_w[:, eid]  # (num_envs, 4) 约定 [w,x,y,z] 或 [x,y,z,w] 取决于你的数据结构

    # 只打印一个 env，避免刷屏
    i = print_env_idx
    tolist = lambda x: x[i].detach().cpu().tolist()

    print(
        "[CMD] ee_pose: pos=", tolist(pos_ee_cmd), " quat=", tolist(quat_ee_cmd), "\n",
        "[CMD] b_pose : pos=", tolist(pos_b_cmd),  " quat=", tolist(quat_b_cmd),  "\n",
        "[CMD] c_pose : pos=", tolist(pos_c_cmd),  " quat=", tolist(quat_c_cmd),  "\n",
        "[EEF] current: pos=", tolist(pos_eef_now), " quat=", tolist(quat_eef_now),
    )


# =========================
# Pinocchio 外部运动学（FK + 世界系 Jacobian）
# =========================
def _mat3_to_quat_xyzw(R, device):
    if not torch.is_tensor(R):
        R = torch.from_numpy(R)
    R = R.to(device).float()
    t = R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2]
    w = torch.sqrt(torch.clamp(1.0 + t, min=0.0)) * 0.5
    x = (R[..., 2, 1] - R[..., 1, 2]) / (4*w + 1e-9)
    y = (R[..., 0, 2] - R[..., 2, 0]) / (4*w + 1e-9)
    z = (R[..., 1, 0] - R[..., 0, 1]) / (4*w + 1e-9)
    return torch.stack([x, y, z, w], dim=-1)

def _get_dof_names_from_data(data):
    for cand in ("dof_names", "joint_names", "joint_name", "names"):
        if hasattr(data, cand):
            try:
                return list(getattr(data, cand))
            except Exception:
                pass
    return None  # 兜底：假定与 URDF 顺序一致
# def _build_pin_ctx(urdf_path: str, eef_frame: str, isaac_dof_names=None):
#     try:
#         import pinocchio as pin
#     except Exception as e:
#         raise RuntimeError("未安装 Pinocchio。请先执行: pip install pin") from e

#     model = pin.buildModelFromUrdf(urdf_path)   # 固定基
#     data  = model.createData()
#     nv    = model.nv
#     try:
#         fid = model.getFrameId(eef_frame)
#     except Exception as e:
#         raise RuntimeError(f"URDF 中找不到末端 frame '{eef_frame}'。") from e

#     # 用 joint.idx_v 作为列号（速度向量的起始列），而不是列表下标
#     # 对于常见的 1-DoF 旋转/平移关节，idx_v 就是唯一那一列
#     name2vidx = {}
#     for jid, jname in enumerate(model.names):
#         j = model.joints[jid]
#         if j.nv > 0:
#             if j.nv != 1:
#                 raise RuntimeError(
#                     f"URDF 关节 '{jname}' 的自由度 nv={j.nv}!=1，当前映射只支持单自由度关节。"
#                     " 请改成按列片段的更通用实现，或在 URDF/Isaac 两侧统一到 1-DoF 关节。"
#                 )
#             name2vidx[jname] = j.idx_v  # 速度向量中的列号，范围 [0..nv-1]

#     if isaac_dof_names is None:
#         # 默认假设 Isaac 的 dof 顺序与 URDF 的 velocity 顺序一致
#         # 构造一个 0..nv-1 的顺序
#         idx_map = torch.arange(nv, dtype=torch.long)
#         isaac_dof_names = [n for n,_ in sorted(name2vidx.items(), key=lambda kv: kv[1])]
#     else:
#         # Isaac -> URDF（按名字找列）
#         idx_vals = []
#         missing = []
#         for n in isaac_dof_names:
#             if n in name2vidx:
#                 idx_vals.append(name2vidx[n])
#             else:
#                 missing.append(n)
#         if missing:
#             raise RuntimeError(
#                 "以下 Isaac 关节名在 URDF 中找不到可映射的 1-DoF 关节：\n  "
#                 + ", ".join(missing) +
#                 "\n请检查命名/前缀，或在创建 env 时只选择手臂的 6 个关节。"
#             )
#         idx_map = torch.as_tensor(idx_vals, dtype=torch.long)

#     # 稳妥性检查
#     if idx_map.numel() == 0:
#         raise RuntimeError("未能构造 idx_map（映射为空）。")
#     if int(idx_map.max()) >= nv or int(idx_map.min()) < 0:
#         raise RuntimeError(f"idx_map 超出范围：nv={nv}, idx_map范围=[{int(idx_map.min())},{int(idx_map.max())}]。")

#     return {
#         "pin": __import__("pinocchio"),
#         "model": model, "data": data,
#         "nv": nv, "fid": fid,
#         "idx_map": idx_map,                 # Isaac索引 i -> URDF列号 k (= idx_v)
#         "isaac_dof_names": isaac_dof_names  # 仅调试用
#     }
# === A) 在 _build_pin_ctx(...) 里增加“碰撞模型” ===


def _pin_min_self_distance(ctx, q_nv, disable_adjacent_pairs: bool, dist_threshold: float = 0.0):
    """
    返回该姿态下的最小自碰距离（若没有碰撞几何或无对，返回 +inf）。
    """
    pin        = ctx["pin"]
    model      = ctx["model"]
    data       = ctx["data"]
    geom_model = ctx.get("geom_model", None)
    geom_data  = ctx.get("geom_data", None)

    if geom_model is None or geom_data is None or len(geom_model.collisionPairs) == 0:
        return float("inf")

    # 更新位姿
    pin.forwardKinematics(model, data, q_nv)
    pin.updateFramePlacements(model, data)
    pin.updateGeometryPlacements(model, data, geom_model, geom_data)

    parents_list = ctx.get("parents_list", None)

    # 逐对计算距离（hints：Pin>=2.7 可用 computeCollisions + distance；老版本用 FCL 的 request/ result）
    try:
        # 新 API（一次性计算 + 读回最小）
        pin.computeCollisions(geom_model, geom_data, False)  # False = 不停止在首个碰撞
        dmin = float("inf")
        for k, pair in enumerate(geom_model.collisionPairs):
            go1 = geom_model.geometryObjects[pair.first]
            go2 = geom_model.geometryObjects[pair.second]
            j1  = int(go1.parentJoint)
            j2  = int(go2.parentJoint)

            if disable_adjacent_pairs and parents_list is not None:
                # 同关节 或 父子关节
                if (j1 == j2) or \
                   (0 <= j1 < len(parents_list) and parents_list[j1] == j2) or \
                   (0 <= j2 < len(parents_list) and parents_list[j2] == j1):
                    continue

            # 读取该对的最小距离
            # 某些版本里存放在 geom_data.distanceResults[k].min_distance
            try:
                d = float(geom_data.distanceResults[k].min_distance)
            except Exception:
                # 兜底：如果 distanceResults 不可用，就动态算一次
                req = pin.collision.DistanceRequest()
                res = pin.collision.DistanceResult()
                d = pin.collision.distance(go1.geometry, go2.geometry,
                                           geom_data.oMg[pair.first].homogeneous,
                                           geom_data.oMg[pair.second].homogeneous,
                                           req, res)
            if d < dmin:
                dmin = d
                if dmin <= dist_threshold:
                    return dmin
        return dmin
    except Exception:
        # 极老的 API：逐对调用 FCL 的 distance
        dmin = float("inf")
        for k, pair in enumerate(geom_model.collisionPairs):
            go1 = geom_model.geometryObjects[pair.first]
            go2 = geom_model.geometryObjects[pair.second]
            j1  = int(go1.parentJoint)
            j2  = int(go2.parentJoint)

            if disable_adjacent_pairs and parents_list is not None:
                if (j1 == j2) or \
                   (0 <= j1 < len(parents_list) and parents_list[j1] == j2) or \
                   (0 <= j2 < len(parents_list) and parents_list[j2] == j1):
                    continue

            req = pin.collision.DistanceRequest()
            res = pin.collision.DistanceResult()
            d = pin.collision.distance(go1.geometry, go2.geometry,
                                       geom_data.oMg[pair.first].homogeneous,
                                       geom_data.oMg[pair.second].homogeneous,
                                       req, res)
            if d < dmin:
                dmin = d
                if dmin <= dist_threshold:
                    return dmin
        return dmin

def _build_pin_ctx(urdf_path: str,
                   eef_frame: str,
                   isaac_dof_names=None,
                   pin_package_dirs=None,
                   enable_collision: bool = False):
    import pinocchio as pin
    import numpy as np
    model = pin.buildModelFromUrdf(urdf_path)   # 固定基
    data  = model.createData()
    nv    = model.nv
    fid   = model.getFrameId(eef_frame)

    # —— 用 joint.idx_v 构造“URDF 速度列号”映射（最稳妥）——
    name2vidx = {}
    for jid, jname in enumerate(model.names):
        j = model.joints[jid]
        if j.nv > 0:
            if j.nv != 1:
                raise RuntimeError(
                    f"URDF 关节 '{jname}' 的自由度 nv={j.nv}!=1；"
                    f"当前映射只支持单自由度关节，请统一到 1-DoF 或扩展映射逻辑。")
            name2vidx[jname] = int(j.idx_v)  # 0..nv-1

    if isaac_dof_names is None:
        # Isaac 顺序默认与 URDF 速度列一致
        idx_map = torch.arange(nv, dtype=torch.long)
        isaac_dof_names = [n for n,_ in sorted(name2vidx.items(), key=lambda kv: kv[1])]
    else:
        idx_vals = []
        missing  = []
        for n in isaac_dof_names:
            if n in name2vidx:
                idx_vals.append(name2vidx[n])
            else:
                missing.append(n)
        if missing:
            avail = ", ".join(sorted(name2vidx.keys()))
            need  = ", ".join(missing)
            raise RuntimeError(
                "以下 Isaac 关节名在 URDF 中找不到对应 1-DoF 关节：\n  "
                + need + "\n可用的 URDF 关节名有：\n  " + avail)
        idx_map = torch.as_tensor(idx_vals, dtype=torch.long)

    # 校验（防止出现你遇到的 'index 6 is out of bounds'）
    if idx_map.numel() == 0:
        raise RuntimeError("idx_map 为空。请检查关节名映射。")
    imax = int(idx_map.max())
    imin = int(idx_map.min())
    if imax >= nv or imin < 0:
        raise RuntimeError(f"idx_map 超范围：min={imin}, max={imax}, nv={nv}。")

    # —— 可选：加载碰撞几何（用于纯运动学自碰撞）——
    geom_model = None
    geom_data  = None
    if enable_collision:
        if pin_package_dirs is None:
            package_dirs = []
        elif isinstance(pin_package_dirs, (list, tuple)):
            package_dirs = list(pin_package_dirs)
        else:
            package_dirs = [str(pin_package_dirs)]

        geom_model = pin.GeometryModel()
        # 兼容不同版本签名
        try:
            pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.COLLISION, geom_model, package_dirs)
        except TypeError:
            pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.COLLISION, package_dirs, geom_model)
        geom_model.addAllCollisionPairs()
        geom_data = pin.GeometryData(geom_model)

    # parents → 纯 Python 列表（有些版本不直接可迭代）
    try:
        parents_list = [int(p) for p in model.parents]
    except Exception:
        try:
            parents_list = list(np.array(model.parents, dtype=np.int64).tolist())
        except Exception:
            parents_list = None

    return {
        "pin": pin,
        "model": model, "data": data,
        "fid": fid, "nv": nv,
        "idx_map": idx_map,                 # Isaac 索引 i → URDF 速度列 k
        "isaac_dof_names": isaac_dof_names,
        "parents_list": parents_list,
        "geom_model": geom_model, "geom_data": geom_data,
    }

# === B) 批量自碰撞检查（q 为 Isaac 顺序；返回 [N] bool）===
@torch.no_grad()
def _is_self_collision_pin(ctx, q_tensor: torch.Tensor, device) -> torch.Tensor:
    pin = ctx["pin"]; model = ctx["model"]; data = ctx["data"]
    geom_model = ctx["geom_model"]; geom_data = ctx["geom_data"]
    idx_map = ctx["idx_map"]

    N, dof = q_tensor.shape
    out = torch.zeros(N, dtype=torch.bool, device=device)

    # 预备：把列映射弄成 python list 以便索引 numpy
    idx_list = idx_map.cpu().tolist()

    for i in range(N):
        q_isaac = q_tensor[i].detach().cpu().double()
        q_nv = q_isaac[idx_list].numpy()  # Isaac 顺序 -> URDF 顺序

        pin.forwardKinematics(model, data, q_nv)
        pin.updateGeometryPlacements(model, data, geom_model, geom_data, q_nv)
        pin.computeCollisions(geom_model, geom_data, True)

        hit = False
        # 遍历当前已注册的碰撞对
        for k in range(len(geom_model.collisionPairs)):
            if geom_data.collisionResults[k].isCollision():
                hit = True
                break
        out[i] = hit
    return out  # True 表示发生自碰撞


# def _pin_fk_jac(ctx, q_tensor: torch.Tensor, device):
    """返回 (pos_w(N,3), quat_w(N,4)[x,y,z,w], J_w(N,6,dof))，不触碰仿真。"""
    pin   = ctx["pin"];   model = ctx["model"]; data = ctx["data"]
    fid   = ctx["fid"];   idx_map = ctx["idx_map"]           # Isaac索引 i -> URDF索引 k
    nv    = ctx["nv"]                                       # URDF 自由度数
    N, dof = q_tensor.shape
    if dof != idx_map.numel():
        raise RuntimeError(f"dof 不匹配: Isaac dof={dof}, URDF映射列数={idx_map.numel()}。")

    # URDF -> Isaac 的反映射：inv[k] = i
    inv = torch.full((nv,), -1, dtype=torch.long)
    inv[idx_map] = torch.arange(dof, dtype=torch.long)
    if (inv < 0).any():
        missing = (inv < 0).nonzero(as_tuple=False).squeeze(-1).tolist()
        raise RuntimeError(f"URDF中 {len(missing)} 个关节未映射到Isaac：URDF索引 {missing}。请核对命名/前缀。")

    inv_list = inv.cpu().tolist()                         # 给 numpy 索引用
    idx_dev  = idx_map.to(device=device, dtype=torch.long)  # 给 torch 重排 J 用

    pos_out  = torch.zeros((N, 3), device=device, dtype=torch.float32)
    quat_out = torch.zeros((N, 4), device=device, dtype=torch.float32)
    J_out    = torch.zeros((N, 6, dof), device=device, dtype=torch.float32)

    for i in range(N):
        # Isaac顺序 q -> URDF顺序 q_nv
        qi   = q_tensor[i].detach().cpu().numpy()
        q_nv = np.asarray(qi[inv_list], dtype=np.float64)   # (nv,)

        pin.forwardKinematics(model, data, q_nv)
        pin.updateFramePlacements(model, data)

        oMf  = data.oMf[fid]
        pos  = torch.from_numpy(oMf.translation.copy()).float().to(device)
        R    = torch.from_numpy(oMf.rotation.copy()).float().to(device)
        quat = _mat3_to_quat_xyzw(R, device)

        # 6 x nv（WORLD）
        J6nv  = pin.computeFrameJacobian(model, data, q_nv, fid, pin.ReferenceFrame.WORLD)
        J6nv  = torch.from_numpy(J6nv).float().to(device)
        J6dof = J6nv[:, idx_dev]                           # URDF -> Isaac 列重排

        pos_out[i]  = pos
        quat_out[i] = quat
        J_out[i]    = J6dof

    return pos_out, quat_out, J_out

def _pin_fk_jac(ctx, q_tensor: torch.Tensor, device):
    """返回 (pos_w(N,3), quat_w(N,4)[x,y,z,w], J_w(N,6,dof))，不触碰仿真。"""
    import numpy as np
    pin   = ctx["pin"];   model = ctx["model"]; data = ctx["data"]
    fid   = ctx["fid"];   idx_map = ctx["idx_map"]
    nv    = ctx["nv"]
    N, dof = q_tensor.shape
    if dof != idx_map.numel():
        raise RuntimeError(f"dof 不匹配: Isaac dof={dof}, URDF列数={idx_map.numel()}。")

    # 再次保护 idx_map 合法（运行时若被污染能第一时间发现）
    imax = int(idx_map.max()); imin = int(idx_map.min())
    if imax >= nv or imin < 0:
        raise RuntimeError(f"运行时 idx_map 越界：min={imin}, max={imax}, nv={nv}。")

    idx_list = idx_map.cpu().tolist()
    idx_dev  = idx_map.to(device=device, dtype=torch.long)

    pos_out  = torch.zeros((N, 3), device=device, dtype=torch.float32)
    quat_out = torch.zeros((N, 4), device=device, dtype=torch.float32)
    J_out    = torch.zeros((N, 6, dof), device=device, dtype=torch.float32)

    for i in range(N):
        # Isaac 顺序 → URDF 速度顺序
        qi   = q_tensor[i].detach().cpu().numpy()         # (dof,)
        q_nv = np.asarray(qi)[idx_list]                   # (nv,) 这里 nv==dof 对 6DoF 手臂成立

        # FK
        pin.forwardKinematics(model, data, q_nv)
        pin.updateFramePlacements(model, data)
        oMf  = data.oMf[fid]
        pos  = torch.from_numpy(oMf.translation.copy()).float().to(device)
        R    = torch.from_numpy(oMf.rotation.copy()).float().to(device)
        quat = _mat3_to_quat_xyzw(R, device)

        # WORLD 雅可比 6×nv，并按 Isaac 顺序重排列
        J6nv  = pin.computeFrameJacobian(model, data, q_nv, fid, pin.ReferenceFrame.WORLD)
        J6nv  = torch.from_numpy(J6nv).float().to(device)
        J6dof = J6nv[:, idx_dev]   # 选取列的顺序 == Isaac dof 顺序

        pos_out[i]  = pos
        quat_out[i] = quat
        J_out[i]    = J6dof

    return pos_out, quat_out, J_out


# =========================
# Command → 世界系姿态
# =========================
def _pose_from_command_world(env, asset_cfg: SceneEntityCfg, command_name: str):
    """从 command_manager 取 Nx7 或 Nx6（rpy+pos），转世界系 (pos_w, quat_w[x,y,z,w])。"""
    asset: RigidObject = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)   # 期望形状 (N,7) 或 (N,6)
    if not torch.is_tensor(cmd) or cmd.shape[-1] not in (6, 7):
        raise RuntimeError(f"指令 '{command_name}' 不是 Nx7 或 Nx6 张量，请检查命令产生器。")
    if cmd.ndim == 1:
        cmd = cmd.unsqueeze(0)

    if cmd.shape[-1] == 7:
        des_pos_b  = cmd[:, :3]
        des_quat_b = cmd[:, 3:7]
    else:
        # Nx6 [rpy(3), pos(3)] -> quat
        rpy = cmd[:, :3]
        des_pos_b = cmd[:, 3:6]
        cr, sr = torch.cos(rpy[:, 0]*0.5), torch.sin(rpy[:, 0]*0.5)
        cp, sp = torch.cos(rpy[:, 1]*0.5), torch.sin(rpy[:, 1]*0.5)
        cy, sy = torch.cos(rpy[:, 2]*0.5), torch.sin(rpy[:, 2]*0.5)
        w = cr*cp*cy - sr*sp*sy
        x = sr*cp*cy + cr*sp*sy
        y = cr*sp*cy - sr*cp*sy
        z = cr*cp*sy + sr*sp*cy
        des_quat_b = torch.stack([x, y, z, w], dim=-1)

    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w, asset.data.root_quat_w, des_pos_b)
    des_quat_w   = quat_mul(asset.data.root_quat_w, des_quat_b)
    return des_pos_w, des_quat_w


# =========================
# b→c 直线路径可达性验证（Pinocchio 版，不改仿真）
# =========================
# @torch.no_grad()
# def _validate_cart_line_reachability_pin(
#     env,
#     pose_b, pose_c,
#     pin_ctx,
#     steps=12, max_iters_per_wp=8,
#     pos_tol=5e-4, rot_tol=0.5*math.pi/180,
#     dls_lambda2=1e-4,
#     joint_limit=True, vel_limit=True,
# ):
#     """完全外部运动学：影子 q 上做 DLS，不写回仿真、不 step。返回 (N,)∈[0,1]。"""
#     device = env.device
#     robot  = env.scene["robot"]; data = robot.data
#     N = getattr(env, "num_envs", data.joint_pos.shape[0])

#     pos_b, quat_b = pose_b
#     pos_c, quat_c = pose_c

#     # —— 用实际生成的 waypoint 数驱动循环，且确保广播正确 ——
#     steps = int(steps)
#     if steps < 1:
#         steps = 1
#     ts = torch.linspace(0, 1, steps + 1, device=device)[1:]  # (num_wp,)
#     ts = ts.view(-1, 1, 1)                                   # (num_wp,1,1) 便于对齐到 (N,3)/(N,4)
#     num_wp = ts.shape[0]

#     # 位置/姿态插值：得到 (num_wp, N, 3) 与 (num_wp, N, 4)
#     pos_wp  = pos_b.unsqueeze(0) * (1 - ts) + pos_c.unsqueeze(0) * ts
#     quat_wp = _slerp(
#         quat_b.unsqueeze(0).expand(num_wp, -1, -1),
#         quat_c.unsqueeze(0).expand(num_wp, -1, -1),
#         ts
#     )

#     # 形状保护
#     assert pos_wp.shape[0] == quat_wp.shape[0] and pos_wp.shape[0] > 0, \
#         f"waypoints empty or mismatched: pos_wp {pos_wp.shape}, quat_wp {quat_wp.shape}, steps={steps}"

#     q0 = data.joint_pos.clone().to(device)
#     q  = q0.clone()
#     q_lo, q_hi = _extract_joint_limits(data, N, device)

#     reached = torch.zeros((num_wp, N), dtype=torch.bool, device=device)
#     for i in range(num_wp):
#         tp, tq = pos_wp[i], quat_wp[i]
#         for _ in range(max_iters_per_wp):
#             cp, cq, J = _pin_fk_jac(pin_ctx, q, device)          # Pinocchio FK + WORLD Jacobian
#             err6 = _pose_err_twist(cp, cq, tp, tq)
#             pos_err = err6[:, :3].norm(-1)
#             rot_err = err6[:, 3:].norm(-1)
#             if torch.all((pos_err <= pos_tol) & (rot_err <= rot_tol)):
#                 reached[i] = True
#                 break
#             dq = _dls_solve(J, err6, lambda2=dls_lambda2)
#             if vel_limit:
#                 dq = dq.clamp(-0.02, 0.02)
#             q = q + dq
#             if joint_limit:
#                 q = _clamp_joint(q, q_lo, q_hi)
                
#         else:
#             reached[i] = False

#     return reached.float().mean(0)  # (N,)
@torch.no_grad()
def _validate_cart_line_reachability_pin(
    env,
    pose_b, pose_c,
    pin_ctx,
    steps=12, max_iters_per_wp=8,
    pos_tol=5e-4, rot_tol=0.5*math.pi/180,
    dls_lambda2=1e-4,
    joint_limit=True, vel_limit=True,
    # 可选：若不传则从 pin_ctx 里取，默认忽略邻接对
    disable_adjacent_pairs: bool | None = None,
):
    """
    用 Pinocchio 在“影子关节”上验证从 b->c 的笛卡尔直线路径可达性（不写回仿真、不 step 物理）。
    返回 (N,) ∈ [0,1]，为每个并行 env 上“达到各航点比例”的均值。
    若 pin_ctx 含碰撞几何，则做纯运动学自碰撞检查，并可忽略邻接连杆对以避免误判。
    """
    device = env.device
    robot  = env.scene["robot"]; data = robot.data
    N = getattr(env, "num_envs", data.joint_pos.shape[0])

    # 1) 解析起止位姿
    pos_b, quat_b = pose_b  # (N,3), (N,4)[x,y,z,w]
    pos_c, quat_c = pose_c

    # 2) 航点插值（S 个，不含起点，含终点） -> (S,N,3) / (S,N,4)
    S = max(1, int(steps))
    t = torch.linspace(1, S, S, device=device) / float(S)   # (S,)
    t3 = t.view(S, 1, 1)                                    # (S,1,1) 便于广播到 (S,N,3/4)

    pos_wp  = pos_b.unsqueeze(0) * (1.0 - t3) + pos_c.unsqueeze(0) * t3          # (S,N,3)
    quat_wp = _slerp(
        quat_b.unsqueeze(0).expand(S, -1, -1),                                   # (S,N,4)
        quat_c.unsqueeze(0).expand(S, -1, -1),                                   # (S,N,4)
        t3,                                                                      # (S,1,1)
    )                                                                            # (S,N,4)
    assert pos_wp.shape == (S, N, 3) and quat_wp.shape == (S, N, 4), \
        f"waypoints bad shapes: pos_wp {pos_wp.shape}, quat_wp {quat_wp.shape}, S={S}, N={N}"

    # 3) 初始影子关节 & 限位
    q  = data.joint_pos.clone().to(device)                  # (N,dof)
    q_lo, q_hi = _extract_joint_limits(data, N, device)     # (N,dof), (N,dof)

    reached = torch.zeros((S, N), dtype=torch.bool, device=device)

    # 4) 纯运动学自碰撞检查（如果 pin_ctx 带了几何）
    pin         = pin_ctx.get("pin", None)
    model_pin   = pin_ctx.get("model", None)
    data_pin    = pin_ctx.get("data", None)
    geom_model  = pin_ctx.get("geom_model", None)
    geom_data   = pin_ctx.get("geom_data", None)
    idx_map     = pin_ctx.get("idx_map", None)
    parents     = pin_ctx.get("parents_list", None)
    use_collision = (pin is not None and model_pin is not None and data_pin is not None
                     and geom_model is not None and geom_data is not None
                     and idx_map is not None)

    if disable_adjacent_pairs is None:
        disable_adjacent_pairs = pin_ctx.get("disable_adjacent_pairs", True)

    if use_collision:
        idx_list = idx_map.cpu().tolist()

        def _check_collision(q_batch: torch.Tensor) -> torch.Tensor:
            """返回 (N,) bool，True 表示该 env 发生‘有效’自碰撞（已忽略邻接连杆对）。"""
            out = torch.zeros(N, dtype=torch.bool, device=device)
            for ii in range(N):
                # Isaac -> URDF
                q_nv = q_batch[ii].detach().cpu().double()[idx_list].numpy()

                # FK + 几何位姿
                pin.forwardKinematics(model_pin, data_pin, q_nv)
                pin.updateFramePlacements(model_pin, data_pin)
                pin.updateGeometryPlacements(model_pin, data_pin, geom_model, geom_data)

                # 碰撞计算（True = 任意一对即返回也会继续填充结果）
                pin.computeCollisions(geom_model, geom_data, True)

                hit = False
                for k, pair in enumerate(geom_model.collisionPairs):
                    # 邻接过滤：同关节或父子关节的对直接跳过
                    if disable_adjacent_pairs and parents is not None:
                        go1 = geom_model.geometryObjects[pair.first]
                        go2 = geom_model.geometryObjects[pair.second]
                        j1, j2 = int(go1.parentJoint), int(go2.parentJoint)
                        if j1 == j2 \
                           or (0 <= j1 < len(parents) and parents[j1] == j2) \
                           or (0 <= j2 < len(parents) and parents[j2] == j1):
                            continue
                    # 有效碰撞
                    if geom_data.collisionResults[k].isCollision():
                        hit = True
                        break
                out[ii] = hit
            return out
    else:
        def _check_collision(q_batch: torch.Tensor) -> torch.Tensor:
            return torch.zeros(N, dtype=torch.bool, device=device)

    # 5) 逐航点 DLS 迭代
    for i in range(S):
        tp, tq = pos_wp[i], quat_wp[i]                    # (N,3), (N,4)
        active = torch.ones(N, dtype=torch.bool, device=device)  # 还需要迭代的 env

        for _ in range(max_iters_per_wp):
            cp, cq, J = _pin_fk_jac(pin_ctx, q, device)          # cp(N,3), cq(N,4), J(N,6,dof)
            err6 = _pose_err_twist(cp, cq, tp, tq)               # (N,6)

            pos_err = err6[:, :3].norm(dim=-1)
            rot_err = err6[:, 3:].norm(dim=-1)
            ok = (pos_err <= pos_tol) & (rot_err <= rot_tol)

            # 标记达标
            reached[i, ok] = True
            active = active & (~ok)
            if not torch.any(active):
                break  # 该航点全部达标

            # 只更新 active 的 env
            dq = _dls_solve(J, err6, lambda2=dls_lambda2)        # (N,dof)
            if vel_limit:
                dq = dq.clamp(-0.02, 0.02)
            dq = dq * active.unsqueeze(-1)                       # 其他 env 不更新
            q = q + dq
            if joint_limit:
                q = _clamp_joint(q, q_lo, q_hi)

            # 碰撞：命中则该航点失败，并从 active 中移除
            coll = _check_collision(q) & active
            if torch.any(coll):
                reached[i, coll] = False
                active = active & (~coll)
                if not torch.any(active):
                    break
        # 未达标且未碰撞的 env 维持 False（默认）

    # 6) 每个 env 的达标比例作为得分
    return reached.float().mean(dim=0)  # (N,)

@torch.no_grad()
def bc_line_reach_term_cmd(
    env,
    asset_cfg: SceneEntityCfg,     # 例：SceneEntityCfg("robot", body_names=["joint6_flange"])
    b_command_name: str,
    c_command_name: str | None = None,
    c_offset_in_tool: float = 0.04,
    keep_c_orientation: bool = True,

    # —— Pinocchio 必需（仅 backend=="pin" 时使用）——
    urdf_path: str = "",
    urdf_eef_frame: str = "joint6_flange",
    
    # —— 路径/IK 参数（用于 b→c 线路评估）——
    steps=12, max_iters_per_wp=8,
    pos_tol=5e-4, rot_tol=0.5*math.pi/180,
    dls_lambda2=1e-4,
    joint_limit=True, vel_limit=True,

    # —— 运行策略 —— 
    debug_force_now: bool = False,           # True=每步评估；False 配合 trigger_once_after_b 做“一次性奖励”
    trigger_once_after_b: bool = True,     # True=到 B 附近时触发一次

    # —— “到达B”的判定阈值（仅一次性触发模式使用）——
    b_reach_pos_tol: float = 0.01,          # 1 cm
    b_reach_rot_tol: float = math.radians(5.0),  # 5°

    # —— 纯运动学自碰撞（仅 pin 后端有用）——
    pin_package_dirs: list[str] | None = None,  # 解析 package:// 的包根目录列表
    enable_collision: bool = False,             # True 才加载碰撞几何
    disable_adjacent_pairs: bool = True,        # 忽略相邻连杆对

    # —— 后端选择 —— 
    backend: str = "pin",                       # "pin" | "none"
):
    # —— 周期性打印，便于核对命令与实际位姿 —— 
    PRINT_EVERY = 60
    env._dbg_tick = getattr(env, "_dbg_tick", 0) + 1
    if env._dbg_tick % PRINT_EVERY == 0:
        debug_print_bce(env, asset_cfg, print_env_idx=0)

    """
    奖励项：读取 b/c 命令，使用“外部运动学后端”验证 b→c 的笛卡尔直线路径可达性。
    - 推荐模式：debug_force_now=False & trigger_once_after_b=True
        到达 B 时仅触发一次评估，并且只在触发步给奖励；其余时间返回 0。
    - 兼容模式：否则与旧逻辑一致（每步评估）。
    """
    # 基本句柄 / 形状
    robot = env.scene[asset_cfg.name]
    rdata = robot.data
    N = getattr(env, "num_envs", rdata.joint_pos.shape[0])
    device = env.device
    dtype = torch.float32

    # ===== 非 pin 后端：直接占位返回 0（GUI 时可规避 pin 依赖）=====
    if backend.lower() != "pin":
        return torch.zeros((N,), device=device, dtype=dtype)

    # ===== pin 后端：准备上下文 =====
    if not urdf_path:
        raise RuntimeError("bc_line_reach_term_cmd 需要参数 urdf_path（URDF 绝对路径）。")

    eef_name = asset_cfg.body_names[0] if getattr(asset_cfg, "body_names", None) else urdf_eef_frame

    need_ctx = (not hasattr(env, "_pin_ctx")) or (env._pin_ctx is None)
    need_geom = enable_collision and (need_ctx or ("geom_model" not in env._pin_ctx))
    if need_ctx or need_geom:
        isaac_names = _get_dof_names_from_data(rdata)  # 取不到则按顺序一致
        env._pin_ctx = _build_pin_ctx(
            urdf_path, urdf_eef_frame,
            isaac_dof_names=isaac_names,
            pin_package_dirs=pin_package_dirs,
            enable_collision=enable_collision,
        )
        env._pin_ctx["disable_adjacent_pairs"] = bool(disable_adjacent_pairs)
        print(f"[PIN] loaded: {urdf_path}, eef={urdf_eef_frame}, nv={env._pin_ctx['nv']}")

    # ===== 读取 b/c 的世界系目标位姿 =====
    pos_b, quat_b = _pose_from_command_world(env, asset_cfg, b_command_name)   # (N,3), (N,4) [w,x,y,z]
    if c_command_name:
        pos_c, quat_c = _pose_from_command_world(env, asset_cfg, c_command_name)
    else:
        # 无 c 命令：沿工具 +X 前进 c_offset_in_tool
        ex = torch.tensor([1.0, 0.0, 0.0], device=device, dtype=pos_b.dtype)
        dir_b = _quat_apply(quat_b[..., [1,2,3,0]], ex)  # _quat_apply 用的是 [x,y,z,w] 约定，因此换位
        pos_c = pos_b + c_offset_in_tool * dir_b
        eid = _get_body_id(rdata, eef_name)
        quat_c = quat_b if keep_c_orientation else rdata.body_quat_w[:, eid]

    # ===== 模式分叉 =====
    # 1) 连续评估（旧逻辑）：debug_force_now=True 或 未开启一次性触发
    if debug_force_now or not trigger_once_after_b:
        return _validate_cart_line_reachability_pin(
            env,
            pose_b=(pos_b, quat_b),
            pose_c=(pos_c, quat_c),
            pin_ctx=env._pin_ctx,
            steps=max(1, int(steps)),
            max_iters_per_wp=int(max_iters_per_wp),
            pos_tol=float(pos_tol),
            rot_tol=float(rot_tol),
            dls_lambda2=float(dls_lambda2),
            joint_limit=bool(joint_limit),
            vel_limit=bool(vel_limit),
            disable_adjacent_pairs=bool(disable_adjacent_pairs),
        )

    # 2) 一次性触发模式：到达 B 时触发一次评估，只在该步给奖励
    # --- 初始化/扩容闩锁缓存 ---
    need_init_latch = (
        (not hasattr(env, "_bc_latch_triggered"))
        or (env._bc_latch_triggered is None)
        or (env._bc_latch_triggered.shape[0] != N)
    )
    if need_init_latch:
        env._bc_latch_triggered = torch.zeros((N,), dtype=torch.bool, device=device)
        env._bc_last_b = torch.empty((N, 7), dtype=dtype, device=device)  # [pos3, quat4(wxyz)]
        env._bc_last_b_valid = torch.zeros((N,), dtype=torch.bool, device=device)

    # --- 若 B 命令改变：解除闩锁（允许再次触发） ---
    changed_mask = ~env._bc_last_b_valid
    if env._bc_last_b_valid.any():
        last_pos_b = env._bc_last_b[:, :3]
        last_quat_b = env._bc_last_b[:, 3:7]
        pos_delta = torch.norm(pos_b - last_pos_b, dim=1)
        # 四元数角差
        rot_delta = quat_error_magnitude(last_quat_b, quat_b)
        changed_mask = changed_mask | (pos_delta > 1e-6) | (rot_delta > 1e-6)

    if changed_mask.any():
        env._bc_latch_triggered[changed_mask] = False
        env._bc_last_b[changed_mask, :3] = pos_b[changed_mask]
        env._bc_last_b[changed_mask, 3:7] = quat_b[changed_mask]
        env._bc_last_b_valid[changed_mask] = True

    # --- 判定是否“到达 B” ---
    eid = _get_body_id(rdata, eef_name)
    pos_now  = rdata.body_pos_w[:, eid]
    quat_now = rdata.body_quat_w[:, eid]  # 与 quat_error_magnitude/quat_mul 同一约定（wxyz）

    near_b = (torch.norm(pos_now - pos_b, dim=1) <= float(b_reach_pos_tol)) & \
             (quat_error_magnitude(quat_now, quat_b) <= float(b_reach_rot_tol))

    # 仅对“刚到达且未触发过”的 env 触发评估
    just_trigger = near_b & (~env._bc_latch_triggered)

    # 默认回报为 0
    reward = torch.zeros((N,), device=device, dtype=dtype)

    if just_trigger.any():
        score = _validate_cart_line_reachability_pin(
            env,
            pose_b=(pos_b, quat_b),
            pose_c=(pos_c, quat_c),
            pin_ctx=env._pin_ctx,
            steps=max(1, int(steps)),
            max_iters_per_wp=int(max_iters_per_wp),
            pos_tol=float(pos_tol),
            rot_tol=float(rot_tol),
            dls_lambda2=float(dls_lambda2),
            joint_limit=bool(joint_limit),
            vel_limit=bool(vel_limit),
            disable_adjacent_pairs=bool(disable_adjacent_pairs),
        )
        # 只在触发步把得分写到回报
        reward[just_trigger] = score[just_trigger]
        # 上闩：本段命令周期内不再触发
        env._bc_latch_triggered[just_trigger] = True

    return reward
