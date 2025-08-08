# stab_path_filter.py
# 用于在 Isaac Lab 中筛选可行的 b 点，使其满足 b → c 的直刺路径可行性

import numpy as np


def is_direction_valid(b_pos, b_dir, c_pos, theta_thresh_deg=15.0):
    """判断末端朝向是否对准 c 点方向（夹角小于阈值）"""
    v_bc = c_pos - b_pos
    v_bc /= np.linalg.norm(v_bc)
    b_dir /= np.linalg.norm(b_dir)
    cos_theta = np.dot(v_bc, b_dir)
    return cos_theta >= np.cos(np.deg2rad(theta_thresh_deg))


def compute_orientation_from_z(z_axis):
    """用末端 z 轴方向构造旋转矩阵"""
    z = z_axis / np.linalg.norm(z_axis)
    x_temp = np.array([1.0, 0.0, 0.0])
    if np.abs(np.dot(x_temp, z)) > 0.99:
        x_temp = np.array([0.0, 1.0, 0.0])
    y = np.cross(z, x_temp)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)  # (3x3) rotation matrix


class SimpleIKHelper:
    def __init__(self, robot):
        """
        简化 IK 解算器，需要实现 compute_inverse_kinematics(pose: 4x4 numpy array)
        你可以替换为你自己的 IK 接口
        """
        self.robot = robot

    def compute_inverse_kinematics(self, target_end_effector_pose: np.ndarray):
        """
        假设这里是你实现的 IK 接口逻辑（需替换成实际 IK）
        返回值：若成功，返回 joint angles；若失败，返回 None
        """
        # TODO: 替换为你自己的 IK 逻辑或调用 C++/Python 接口
        # 这里只是一个示例
        return np.zeros((self.robot.num_dof,))


def check_path_reachable(b_pos, b_dir, c_pos, ik_helper, num_steps=20):
    """判断从 b 到 c 的直线路径是否全部点都可 IK 解"""
    orientation = compute_orientation_from_z(b_dir)
    for i in range(num_steps + 1):
        lam = i / num_steps
        pos = b_pos + lam * (c_pos - b_pos)

        # 构造 4x4 变换矩阵
        pose = np.eye(4)
        pose[:3, :3] = orientation
        pose[:3, 3] = pos

        # IK 解算
        result = ik_helper.compute_inverse_kinematics(target_end_effector_pose=pose)
        if result is None:
            return False
    return True


def is_valid_b_point(b_pose: np.ndarray, ik_helper, needle_length=0.01, angle_thresh_deg=15.0):
    """
    主判断函数：是否为合法的 b 点，使得从 b 到 c 的直线刺入路径可行

    :param b_pose: 4x4 pose matrix
    :param ik_helper: ArticulationIKHelper 实例
    :param needle_length: 从 b 点朝末端方向延伸到 c 点的距离（单位：米）
    :param angle_thresh_deg: 最大允许夹角（单位：度）
    """
    b_pos = b_pose[:3, 3]
    b_dir = b_pose[:3, 2]  # z 轴方向
    c_pos = b_pos + needle_length * b_dir

    if not is_direction_valid(b_pos, b_dir, c_pos, theta_thresh_deg=angle_thresh_deg):
        return False

    if not check_path_reachable(b_pos, b_dir, c_pos, ik_helper):
        return False

    return True
