# # -*- coding: utf-8 -*-
# # Pure Torch Damped-Least-Squares IK for Isaac Lab articulations
# # Works without DifferentialIKController. Batch-friendly; CUDA-ready.
# from __future__ import annotations

# import math
# from dataclasses import dataclass
# from typing import Optional, Tuple

# import torch


# @dataclass
# class PureIKCfg:
#     eef_name: str = "joint6_flange"     # 你的末端链接名
#     damping: float = 1e-2               # 阻尼 λ
#     pos_weight: float = 1.0             # 位置误差权重
#     ori_weight: float = 0.5             # 姿态误差权重
#     step_scale: float = 1.0             # Δq 步长缩放
#     max_iters: int = 128                # 迭代次数
#     pos_tol: float = 1e-3               # 位置收敛阈值（米）
#     ori_tol: float = 1e-2               # 姿态收敛阈值（弧度）
#     clamp_vel: float = 0.2              # 每步关节增量上限（弧度）
#     verbose_once: bool = True           # 仅首次打印警告


# class PureIK:
#     """
#     Damped Least Squares IK (batch envs) with PhysX Jacobians.
#     不依赖 Isaac Lab 内置 IK 控制器；直接读 articulation Jacobian。
#     """
#     def __init__(self, robot, num_envs: int, dof: int, device: torch.device, cfg: Optional[PureIKCfg] = None):
#         self.robot = robot
#         self.num_envs = int(num_envs)
#         self.dof = int(dof)
#         self.device = torch.device(device)
#         self.cfg = cfg or PureIKCfg()

#         # 解析 EEF 索引（body/link）
#         eef_name = self.cfg.eef_name
#         body_names = getattr(self.robot.data, "body_names", None)
#         if body_names is None:
#             raise RuntimeError("[PureIK] robot.data.body_names 不可用，无法定位 EEF。")

#         try:
#             self.eef_body_id = int(body_names.index(eef_name))
#         except ValueError:
#             raise RuntimeError(f"[PureIK] 在 robot.data.body_names 中找不到 EEF 名称: '{eef_name}'")

#         self._warned = False

#     # ----------------------- 外部接口 -----------------------

#     @torch.no_grad()
#     def solve(
#         self,
#         q_init: torch.Tensor,                           # (N, dof)
#         target_pos_w: torch.Tensor,                     # (N, 3)
#         target_quat_w: Optional[torch.Tensor] = None,   # (N, 4) wxyz
#         filter_joint_mask: Optional[torch.Tensor] = None # (dof,) 1 可动, 0 冻结
#     ) -> Tuple[torch.Tensor, torch.Tensor, dict]:
#         """
#         返回:
#             q_sol : (N, dof)
#             ok    : (N,) bool
#             info  : dict, 包含最终误差 / 迭代等信息
#         """
#         N = self.num_envs
#         assert q_init.shape == (N, self.dof)
#         assert target_pos_w.shape == (N, 3)
#         if target_quat_w is None:
#             target_quat_w = torch.tensor([1, 0, 0, 0], dtype=q_init.dtype, device=self.device).repeat(N, 1)
#         else:
#             assert target_quat_w.shape == (N, 4)

#         q = q_init.clone()

#         for it in range(self.cfg.max_iters):
#             eef_pos, eef_quat = self._get_eef_pose()              # (N,3), (N,4)
#             pos_err = target_pos_w - eef_pos                      # (N,3)
#             ori_err = self._quat_err(eef_quat, target_quat_w)     # (N,3) 轴角的小角度向量

#             pos_norm = pos_err.norm(dim=-1)                       # (N,)
#             ori_norm = ori_err.norm(dim=-1)                       # (N,)

#             # 收敛判定
#             done_mask = (pos_norm <= self.cfg.pos_tol) & (ori_norm <= self.cfg.ori_tol)

#             if torch.all(done_mask):
#                 break

#             # 组装误差 twist（6x1）：[pos * wp, ori * wo]
#             wp = self.cfg.pos_weight
#             wo = self.cfg.ori_weight
#             twist = torch.cat([wp * pos_err, wo * ori_err], dim=-1)  # (N, 6)

#             # 取 EEF 的 6×dof Jacobian
#             J = self._get_eef_jacobian()  # (N, 6, dof)

#             # 可选冻结部分关节
#             if filter_joint_mask is not None:
#                 # mask: (dof,), 1 keep, 0 zero
#                 J = J * filter_joint_mask.view(1, 1, -1)

#             # DLS: Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ twist
#             lam = self.cfg.damping
#             JT = J.transpose(-2, -1)                          # (N, dof, 6)
#             JJt = J @ JT                                       # (N, 6, 6)
#             I6 = torch.eye(6, dtype=JJt.dtype, device=self.device).unsqueeze(0).expand(N, -1, -1)
#             A = JJt + (lam * lam) * I6                         # (N, 6, 6)
#             # 解 (A x = twist) 然后 Δq = Jᵀ x
#             x = torch.linalg.solve(A, twist.unsqueeze(-1))     # (N, 6, 1)
#             dq = (JT @ x).squeeze(-1)                          # (N, dof)

#             # 步长与关节增量限幅
#             dq = torch.clamp(dq, min=-self.cfg.clamp_vel, max=self.cfg.clamp_vel)
#             dq = dq * self.cfg.step_scale

#             # 冻结关节再次应用
#             if filter_joint_mask is not None:
#                 dq = dq * filter_joint_mask.view(1, -1)

#             q = q + dq
#             self._apply_joint_pos(q)

#         # 最终误差
#         eef_pos, eef_quat = self._get_eef_pose()
#         pos_err = (target_pos_w - eef_pos).norm(dim=-1)          # (N,)
#         ori_err = self._quat_err(eef_quat, target_quat_w).norm(dim=-1)
#         ok = (pos_err <= self.cfg.pos_tol) & (ori_err <= self.cfg.ori_tol)

#         info = {
#             "iters": it + 1,
#             "final_pos_err": pos_err,
#             "final_ori_err": ori_err,
#         }
#         return q, ok, info

#     # ----------------------- 内部：姿态/雅可比/读写 -----------------------

#     def _quat_err(self, q_curr: torch.Tensor, q_target: torch.Tensor) -> torch.Tensor:
#         """
#         计算 q_curr -> q_target 的小角度误差向量（3,）
#         四元数格式 wxyz，返回近似轴角向量（假设角度较小）。
#         """
#         # 旋转误差 q_err = q_target * conj(q_curr)
#         w1, x1, y1, z1 = q_target.unbind(-1)
#         w2, x2, y2, z2 = q_curr.unbind(-1)
#         # conj(curr) = (w, -x, -y, -z)
#         w = w1*w2 + x1*(-x2) + y1*(-y2) + z1*(-z2)
#         x = w1*(-x2) + x1*w2 + y1*(-z2) - z1*(-y2)
#         y = w1*(-y2) - x1*(-z2) + y1*w2 + z1*(-x2)
#         z = w1*(-z2) + x1*(-y2) - y1*(-x2) + z1*w2
#         # 小角度近似：axis*angle ≈ 2 * vec_part (当 w≈1 时)
#         # 为稳定也可用 atan2(‖v‖, w) * v/‖v‖，这里用小角近似即可
#         return 2.0 * torch.stack([x, y, z], dim=-1)

#     def _get_eef_pose(self) -> Tuple[torch.Tensor, torch.Tensor]:
#         """
#         从 robot.data 读 EEF 世界位姿（position (N,3), quat wxyz (N,4)）
#         """
#         data = self.robot.data
#         # body_link_pose_w: (N, num_bodies, 7)  -> [pos(3), quat(x,y,z,w)? or (w,x,y,z)?]
#         # Isaac Lab 通常使用 [pos(3), quat(x,y,z,w)]，我们转成 wxyz。
#         poses = getattr(data, "body_link_pose_w", None)
#         if poses is None:
#             poses = getattr(data, "body_pose_w", None)
#         if poses is None:
#             raise RuntimeError("[PureIK] 无法从 robot.data 读取 body_link_pose_w/body_pose_w。")

#         eef_pose = poses[:, self.eef_body_id, :]                  # (N,7)
#         pos = eef_pose[:, :3]
#         # 假定是 xyzw -> 变成 wxyz
#         q_xyzw = eef_pose[:, 3:7]
#         q_wxyz = torch.stack([q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2]], dim=-1)
#         return pos.to(self.device), q_wxyz.to(self.device)

#     def _get_eef_jacobian(self) -> torch.Tensor:
#         """
#         获取 EEF 的 6×dof Jacobian (N, 6, dof) from PhysX.
#         兼容多种常见入口：root_physx_view / articulation_view / _articulation_view
#         """
#         av = getattr(self.robot, "root_physx_view", None)
#         if av is None:
#             av = getattr(self.robot, "articulation_view", None)
#         if av is None:
#             av = getattr(self.robot, "_articulation_view", None)
#         if av is None:
#             self._warn_once("[PureIK] 无法定位 articulation_view/root_physx_view，取 Jacobian 失败。")
#             raise RuntimeError("No articulation view to fetch jacobians.")

#         # jacobians: (N, num_bodies, 6, dof)
#         try:
#             J_all = av.get_jacobians()
#         except Exception as e:
#             self._warn_once(f"[PureIK] get_jacobians() 调用失败: {e}")
#             # 某些版本需要先刷新/同步
#             try:
#                 if hasattr(av, "update"):
#                     av.update()
#                 J_all = av.get_jacobians()
#             except Exception as e2:
#                 raise RuntimeError(f"[PureIK] 仍无法获取 Jacobians: {e2}")

#         if isinstance(J_all, torch.Tensor):
#             J_all = J_all.to(self.device)
#         else:
#             J_all = torch.as_tensor(J_all, device=self.device)

#         J_eef = J_all[:, self.eef_body_id, :, :]  # (N, 6, dof)
#         return J_eef

#     def _apply_joint_pos(self, q: torch.Tensor):
#         """
#         把 q 写回 robot（目标位置），并刷新内部状态缓存。
#         注意：这里不 step 物理，只更新目标 / 缓存。
#         """
#         data = self.robot.data
#         if hasattr(data, "joint_pos_target"):
#             data.joint_pos_target[:] = q
#         elif hasattr(data, "joint_pos"):
#             data.joint_pos[:] = q
#         else:
#             raise RuntimeError("[PureIK] robot.data 不包含 joint_pos(_target) 字段。")

#         # Isaac Lab 大多有 data.update() 以刷新缓存（FK/Jacobian 依赖）
#         if hasattr(data, "update"):
#             data.update()

#         # 某些版本 articulation_view 也要更新
#         av = getattr(self.robot, "root_physx_view", None)
#         if av is None:
#             av = getattr(self.robot, "articulation_view", None)
#         if av is None:
#             av = getattr(self.robot, "_articulation_view", None)
#         if av is not None and hasattr(av, "update"):
#             av.update()

#     def _warn_once(self, msg: str):
#         if not self._warned and self.cfg.verbose_once:
#             print(msg, flush=True)
#             self._warned = True
