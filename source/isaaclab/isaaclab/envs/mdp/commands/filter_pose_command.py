# filtered_pose_command.py
# 基于 stab_path_filter 中的约束，生成可行的 b 点姿态命令

import torch
import numpy as np
from typing import Sequence
from typing import TYPE_CHECKING

from .pose_command import UniformPoseCommand  # 确保导入路径正确
from .stab_path_filter import is_valid_b_point
from isaaclab.utils.math import quat_from_euler_xyz, quat_unique
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import FilteredPoseCommandCfg

# filtered_pose_command.py
# 基于 FilteredPoseCommandCfg 配置，生成可行的 b 点姿态命令

class FilteredPoseCommand(UniformPoseCommand):
    def __init__(self, cfg, env):
        # cfg: FilteredPoseCommandCfg, env: ManagerBasedEnv
        super().__init__(cfg, env)
        # 动态解析 IK helper 路径
        helper = env
        for attr in cfg.ik_helper_path.split('.'):
            helper = getattr(helper, attr)
        self.ik_helper = helper
        # 从 cfg 中读取参数
        self.needle_length = cfg.needle_length
        self.angle_thresh_deg = cfg.angle_thresh_deg

    def _resample_command(self, env_ids: Sequence[int]):
        max_trials = 50
        for env_id in env_ids:
            for _ in range(max_trials):
                # 1. 随机采样位置
                pos = torch.empty(3, device=self.device)
                pos[0].uniform_(*self.cfg.ranges.pos_x)
                pos[1].uniform_(*self.cfg.ranges.pos_y)
                pos[2].uniform_(*self.cfg.ranges.pos_z)
                # 2. 随机采样欧拉角并转四元数
                roll = torch.empty(1, device=self.device).uniform_(*self.cfg.ranges.roll)
                pitch = torch.empty(1, device=self.device).uniform_(*self.cfg.ranges.pitch)
                yaw = torch.empty(1, device=self.device).uniform_(*self.cfg.ranges.yaw)
                from isaaclab.utils.math import quat_from_euler_xyz, quat_unique
                quat = quat_unique(quat_from_euler_xyz(roll, pitch, yaw))[0]

                # 3. 构造 4x4 pose 矩阵
                quat_np = quat.cpu().numpy()
                rot = torch.tensor(self._quat_to_matrix(quat_np), device=self.device)
                pose_mat = torch.eye(4, device=self.device)
                pose_mat[:3, :3] = rot
                pose_mat[:3, 3] = pos

                # 4. 路径与方向可达性检查
                if is_valid_b_point(pose_mat.cpu().numpy(), self.ik_helper,
                                     needle_length=self.needle_length,
                                     angle_thresh_deg=self.angle_thresh_deg):
                    self.pose_command_b[env_id, :3] = pos
                    self.pose_command_b[env_id, 3:] = quat
                    break
            else:
                # 如果未能采到有效点，回退到默认姿态
                default_pos = torch.tensor([0.2, 0.0, 0.1], device=self.device)
                default_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device)
                self.pose_command_b[env_id, :3] = default_pos
                self.pose_command_b[env_id, 3:] = default_quat

    @staticmethod
    def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
        ])
