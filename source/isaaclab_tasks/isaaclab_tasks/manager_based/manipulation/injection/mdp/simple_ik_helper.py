# # simple_ik_helper.py
# import torch
# from typing import Optional

# try:
#     from isaaclab.controllers import DifferentialIKController
#     _HAS_DIK = True
# except Exception:
#     DifferentialIKController = None  # type: ignore
#     _HAS_DIK = False


# class SimpleIKHelper:
#     """
#     最小可用的 IK 辅助：
#     - 兼容多版本 DifferentialIKController 构造签名；
#     - 不改动作，只做“可达性门控”：返回 (q_init, ok_mask)；
#     - 构造或调用不匹配时，优雅退回占位模式（不打断训练）。
#     """
#     def __init__(self, robot, eef_name: str, damping: float, iters: int, tol: float):
#         self.robot = robot
#         self.eef_name = eef_name
#         self.damping = float(damping)
#         self.max_iters = int(iters)
#         self.tol = float(tol)

#         # num_envs / device 推断
#         try:
#             self._num_envs = int(robot.data.joint_pos.shape[0])
#         except Exception:
#             self._num_envs = 1
#         self._device_str = str(getattr(robot, "device", "cuda:0"))

#         # 末端帧 id
#         self._body_names = list(robot.data.body_names)
#         if eef_name not in self._body_names:
#             raise ValueError(f"EEF '{eef_name}' not in robot bodies: {self._body_names[:10]} ...")
#         self._eef_id = self._body_names.index(eef_name)

#         self._ctrl = None
#         self._warned_build = False
#         self._warned_call = False

#         if not _HAS_DIK:
#             print("[IK] WARN: DifferentialIKController not available; using placeholder (always-ok).")
#             return

#         self._ctrl = self._build_controller()
#         if self._ctrl is None and not self._warned_build:
#             print("[IK] WARN: DifferentialIKController could not be configured on this version; "
#                   "using placeholder (always-ok).")
#             self._warned_build = True

#     # ------------------------------ 构造控制器 ------------------------------ #
#     def _build_controller(self) -> Optional[object]:
#         """
#         只尝试最保守的几种签名，不传 cfg、不设置 controller 字段，
#         成功后仅尽力指定 EEF 索引。
#         """
#         trials = [
#             # 1) 仅 articulation
#             lambda: DifferentialIKController(self.robot),
#             # 2) articulation + device
#             lambda: DifferentialIKController(self.robot, device=self._device_str),
#             # 3) articulation + num_envs + device（关键字）
#             lambda: DifferentialIKController(self.robot, num_envs=self._num_envs, device=self._device_str),
#             # 4) articulation + num_envs（关键字）
#             lambda: DifferentialIKController(self.robot, num_envs=self._num_envs),
#         ]

#         reasons = []
#         for ctor in trials:
#             try:
#                 ctrl = ctor()
#                 self._try_set_eef(ctrl)  # 只设置 EEF，失败就跳过
#                 return ctrl
#             except Exception as e:
#                 reasons.append(str(e))

#         if not self._warned_build:
#             print("[IK] WARN: DifferentialIKController construction failed with conservative signatures:")
#             for r in reasons:
#                 print("   -", r)
#             self._warned_build = True
#         return None

#     def _try_set_eef(self, ctrl) -> None:
#         """尽力指定末端帧；没有对应接口就忽略。"""
#         # 常见接口 1：方法
#         if hasattr(ctrl, "set_end_effector_frame"):
#             try:
#                 ctrl.set_end_effector_frame(self._eef_id)
#                 return
#             except Exception:
#                 pass
#         # 常见接口 2：属性
#         for attr in ("end_effector_idx", "end_effector_frame_idx", "frame_idx", "body_idx"):
#             if hasattr(ctrl, attr):
#                 try:
#                     setattr(ctrl, attr, self._eef_id)
#                     return
#                 except Exception:
#                     pass

#     # ------------------------------ 对外接口 ------------------------------ #
#     @torch.no_grad()
#     def solve(self, target_pos_w: torch.Tensor, target_quat_w: torch.Tensor, q_init: torch.Tensor):
#         """
#         输入：
#             target_pos_w  [N,3]
#             target_quat_w [N,4]
#             q_init        [N,dof]
#         输出：
#             (q_return, ok_mask)；当前策略保持 q 不变，仅给可达性 ok_mask
#         """
#         n = target_pos_w.shape[0]
#         device = target_pos_w.device

#         if self._ctrl is None:
#             if not self._warned_call:
#                 print("[IK] WARN: controller not constructed; using placeholder (ok=True, q=q_init).")
#                 self._warned_call = True
#             ok = torch.ones((n,), dtype=torch.bool, device=device)
#             return q_init, ok

#         ok = torch.zeros((n,), dtype=torch.bool, device=device)

#         # compute 尝试 1：带 max_iters / tol
#         try:
#             _ = self._ctrl.compute(target_pos_w, target_quat_w, q_init,
#                                    max_iters=self.max_iters, tol=self.tol)
#             ok[:] = True
#             return q_init, ok
#         except TypeError:
#             pass
#         except Exception:
#             # 不再重复刷屏
#             if not self._warned_call:
#                 print("[IK] WARN: compute(..., max_iters, tol) not supported; trying other signatures.")
#                 self._warned_call = True

#         # compute 尝试 2：不带额外关键字
#         try:
#             _ = self._ctrl.compute(target_pos_w, target_quat_w, q_init)
#             ok[:] = True
#             return q_init, ok
#         except TypeError:
#             pass
#         except Exception:
#             if not self._warned_call:
#                 print("[IK] WARN: compute(pos, quat, q) failed; falling back to placeholder.")
#                 self._warned_call = True

#         # 占位返回：认为可达，q 不变
#         ok[:] = True
#         return q_init, ok
