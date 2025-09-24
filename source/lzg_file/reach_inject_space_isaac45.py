"""
Isaac Sim 4.5 | myCobot 280 M5
Reach + 40mm直刺 可行空间筛选 & 最大连贯6D区间提取（含GUI可视化）

运行：
  ./python.sh reach_inject_space_isaac45.py

注意：
- 请先修改 USD_PATH 为你的实际usd路径
- 默认末端工具针轴为局部Z轴，如不同请改 END_EFFECTOR_AXIS
"""
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import os
import json
import numpy as np
from typing import List, Tuple
from omni.isaac.kit import SimulationApp

# --------------------------- 配置区域 ---------------------------
# GUI 可视化
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree

# ====== 你的机械臂 USD 路径（务必修改为你的实际路径）======
USD_PATH = "/home/lzg/codes/sim_usd_assets/robot/mycombot/mycobot_280_m5/mycobot_280_m5/mycobot_280_m5/mycobot_280_m5_not_limited_joint_angle.usd"

# 直刺长度（米）
L = 0.04

# 末端针轴（在工具坐标系下）："x" | "y" | "z" | "-x" | "-y" | "-z"
END_EFFECTOR_AXIS = "z"

# 采样范围（请按你的场景调整；单位：米 / 度）
POS_RANGE = {
    "x": [0.40, 0.60],
    "y": [0.10, 0.30],
    "z": [1.30, 1.50],
}
RPY_RANGE_DEG = {
    "roll": [0.0, 0.0],        # 可固定0°简化；也可 [-180,180]
    "pitch": [-15.0, 15.0],
    "yaw": [75.0, 105.0],
}

# DEMO 可视化的目标（可手动改一个你想看的点）
DEMO_POSE = {
    "x": 0.50, "y": 0.20, "z": 1.40,
    "roll": 0.0, "pitch": 0.0, "yaw": 90.0
}

# 批量采样数量 & 插补检查点数
NUM_SAMPLES = 1500           # 批量筛选候选点数量
N_CHECKS_ALONG_LINE = 7      # 直线插补含端点，建议 5~10

# 聚类阈值（六维欧氏距离），以及最小簇点数
CLUSTER_EPS = 0.06           # 可按你的6D尺度调大/调小（位置米、角度已做归一再量纲平衡）
CLUSTER_MIN_SAMPLES = 15

# 角度归一时的“角度→米”权重（让角度与位置同量纲便于聚类/距离）
ANGLE_SCALE = 0.01           # 1度≈0.01“米”的权重，可按任务敏感度微调

# 盒子角点回检时的最大缩边迭代步数与步长比例
SHRINK_MAX_ITERS = 25
SHRINK_STEP_FRAC = 0.10      # 每次向内收缩10%
# ---------------------------------------------------------------


def axis_vector_local(name: str) -> np.ndarray:
    table = {
        "x":  np.array([1,0,0], dtype=float),
        "y":  np.array([0,1,0], dtype=float),
        "z":  np.array([0,0,1], dtype=float),
        "-x": np.array([-1,0,0], dtype=float),
        "-y": np.array([0,-1,0], dtype=float),
        "-z": np.array([0,0,-1], dtype=float),
    }
    return table[name]

def make_pose_matrix(pos: np.ndarray, rpy_deg: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """生成4x4 SE(3) 与 3x3旋转矩阵"""
    T = np.eye(4)
    Rm = R.from_euler("xyz", np.deg2rad(rpy_deg)).as_matrix()
    T[:3,:3] = Rm
    T[:3, 3] = pos
    return T, Rm

def solve_ik_pose(ik_solver: ArticulationKinematicsSolver, T: np.ndarray):
    """返回 (success: bool, q: np.ndarray or None)"""
    try:
        success, q = ik_solver.compute_inverse_kinematics(target_end_effector_pose=T)
        return bool(success), (np.array(q) if success else None)
    except Exception as e:
        print("[IK] Exception:", e)
        return False, None

def animate_to_joint(world: World, robot: Robot, q_target: np.ndarray, steps=60):
    """简单关节插值动画，便于GUI观察"""
    # 读当前
    cur = robot.get_joint_positions()
    if cur is None:
        # 第一次可能None，尝试step获取
        for _ in range(2):
            world.step(render=True)
        cur = robot.get_joint_positions()

    q0 = np.array(cur)
    qt = np.array(q_target)
    for i in range(1, steps+1):
        alpha = i/steps
        q = (1-alpha)*q0 + alpha*qt
        robot.set_joint_positions(q)
        world.step(render=True)

def check_line_reachability(
    ik_solver: ArticulationKinematicsSolver,
    pos: np.ndarray,
    Rm: np.ndarray,
    n_checks: int,
    axis_local: str,
    L: float
) -> bool:
    """同姿态沿针轴推进 L，离散检查直线上的 IK 可解"""
    d_local = axis_vector_local(axis_local)
    d_world = Rm @ d_local
    pos2 = pos + L * d_world

    # 预先检查两端
    T0, _ = make_pose_matrix(pos, R.from_matrix(Rm).as_euler("xyz", degrees=True))
    T1, _ = make_pose_matrix(pos2, R.from_matrix(Rm).as_euler("xyz", degrees=True))
    ok0, _ = solve_ik_pose(ik_solver, T0)
    if not ok0: return False
    ok1, _ = solve_ik_pose(ik_solver, T1)
    if not ok1: return False

    # 中间插补
    ts = np.linspace(0.0, 1.0, n_checks)
    for t in ts:
        p = (1-t)*pos + t*pos2
        Tt = np.eye(4); Tt[:3,:3] = Rm; Tt[:3,3] = p
        okt, _ = solve_ik_pose(ik_solver, Tt)
        if not okt:
            return False
    return True

def sample_uniform(POS_RANGE, RPY_RANGE_DEG):
    x = np.random.uniform(*POS_RANGE["x"])
    y = np.random.uniform(*POS_RANGE["y"])
    z = np.random.uniform(*POS_RANGE["z"])
    roll  = np.random.uniform(*RPY_RANGE_DEG["roll"])
    pitch = np.random.uniform(*RPY_RANGE_DEG["pitch"])
    yaw   = np.random.uniform(*RPY_RANGE_DEG["yaw"])
    return np.array([x,y,z, roll,pitch,yaw], dtype=float)

def norm6_for_cluster(pts6: np.ndarray) -> np.ndarray:
    """把角度(°)按 ANGLE_SCALE 映射到近似米量纲，便于 KD-Tree 聚类"""
    pts = pts6.copy()
    pts[:, 3:] = pts[:, 3:] * ANGLE_SCALE
    return pts

def connected_components_radius(pts6: np.ndarray, eps: float, min_pts: int) -> List[np.ndarray]:
    """基于半径图的连通分量（简版 DBSCAN），返回每个簇的索引数组"""
    if pts6.shape[0] == 0:
        return []

    X = norm6_for_cluster(pts6)
    tree = cKDTree(X)
    N = X.shape[0]
    visited = np.zeros(N, dtype=bool)
    clusters = []

    for i in range(N):
        if visited[i]:
            continue
        # BFS 以半径 eps 连接
        queue = [i]
        visited[i] = True
        comp = [i]
        while queue:
            j = queue.pop()
            idxs = tree.query_ball_point(X[j], r=eps)
            for k in idxs:
                if not visited[k]:
                    visited[k] = True
                    queue.append(k)
                    comp.append(k)
        if len(comp) >= min_pts:
            clusters.append(np.array(comp, dtype=int))
    return clusters

def validate_box_corners(
    ik_solver: ArticulationKinematicsSolver,
    box: dict,
    n_checks_line: int
) -> bool:
    """对6维盒子八/多角点进行IK+直刺线检查，全部通过才算OK"""
    # 角点组合
    xs = [box["x"][0], box["x"][1]]
    ys = [box["y"][0], box["y"][1]]
    zs = [box["z"][0], box["z"][1]]
    rs = [box["roll"][0], box["roll"][1]]
    ps = [box["pitch"][0], box["pitch"][1]]
    ysaw = [box["yaw"][0], box["yaw"][1]]

    for xi in xs:
        for yi in ys:
            for zi in zs:
                for r in rs:
                    for p in ps:
                        for y in ysaw:
                            pos = np.array([xi, yi, zi], dtype=float)
                            T, Rm = make_pose_matrix(pos, [r,p,y])
                            if not check_line_reachability(ik_solver, pos, Rm, n_checks_line, END_EFFECTOR_AXIS, L):
                                return False
    return True

def shrink_box_until_valid(
    ik_solver: ArticulationKinematicsSolver,
    box_init: dict,
    n_checks_line: int,
    max_iters: int = SHRINK_MAX_ITERS,
    step_frac: float = SHRINK_STEP_FRAC
) -> dict:
    """当直接用 min/max 拟合的盒子过大时，做角点回检并向内等比收缩边界直到通过"""
    box = {k: [float(v[0]), float(v[1])] for k,v in box_init.items()}
    for it in range(max_iters):
        ok = validate_box_corners(ik_solver, box, n_checks_line)
        if ok:
            print(f"[shrink] 盒子角点回检通过，迭代 {it} 次")
            return box
        # 向内收缩
        for key in ["x","y","z","roll","pitch","yaw"]:
            lo, hi = box[key]
            mid = 0.5*(lo+hi)
            half = 0.5*(hi-lo)*(1.0 - step_frac)
            box[key] = [mid - half, mid + half]
        print(f"[shrink] 角点未通过，收缩到: { {k:[round(v[0],4),round(v[1],4)] for k,v in box.items()} }")
    print("[shrink] 达到最大收缩迭代数，返回最后结果（可能仍偏保守）")
    return box

def main():
    # ---------- 创建世界 & 加载机器人 ----------
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # 先把USD引用到Stage
    add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/M280")

    # 用已有Prim创建Robot对象
    m280 = Robot(prim_path="/World/M280", name="m280")
    world.scene.add(m280)

    world.reset()
    world.step(render=True)

    # ---------- IK 求解器 ----------
    ik_solver = ArticulationKinematicsSolver(m280)

    # ---------- DEMO：GUI可视化（到点→直线40mm） ----------
    print("\n[DEMO] 可视化 reach + 40mm 直刺 ...")
    pos_demo = np.array([DEMO_POSE["x"], DEMO_POSE["y"], DEMO_POSE["z"]], dtype=float)
    T_demo, Rm_demo = make_pose_matrix(
        pos_demo, [DEMO_POSE["roll"], DEMO_POSE["pitch"], DEMO_POSE["yaw"]]
    )

    ok_demo0, q_demo0 = solve_ik_pose(ik_solver, T_demo)
    if not ok_demo0:
        print("[DEMO] reach 点 IK 失败，请调整 DEMO_POSE 或USD姿态/基座位置")
    else:
        print("[DEMO] reach 点 IK 成功，开始动画到 reach 位姿")
        animate_to_joint(world, m280, q_demo0, steps=90)

        # 直刺终点
        d_local = axis_vector_local(END_EFFECTOR_AXIS)
        d_world = Rm_demo @ d_local
        pos_demo2 = pos_demo + L * d_world
        T_demo2 = np.eye(4); T_demo2[:3,:3]=Rm_demo; T_demo2[:3,3]=pos_demo2

        ok_demo1, q_demo1 = solve_ik_pose(ik_solver, T_demo2)
        if not ok_demo1:
            print("[DEMO] 直刺终点 IK 失败，请调整 DEMO_POSE 或 姿态/轴向")
        else:
            print("[DEMO] 直刺终点 IK 成功，开始沿线插补动画")
            # 简单直线插补，按N_CHECKS_ALONG_LINE步
            ts = np.linspace(0.0, 1.0, N_CHECKS_ALONG_LINE)
            prev_q = q_demo0.copy()
            for t in ts[1:]:
                p = (1-t)*pos_demo + t*pos_demo2
                Tt = np.eye(4); Tt[:3,:3]=Rm_demo; Tt[:3,3]=p
                okt, qt = solve_ik_pose(ik_solver, Tt)
                if not okt:
                    print("[DEMO] 中途 IK 失败（演示），停止")
                    break
                animate_to_joint(world, m280, qt, steps=24)
                prev_q = qt

    # ---------- 批量筛选 ----------
    print("\n[筛选] 大规模随机采样并检查可行性 ...")
    valid = []
    for i in range(NUM_SAMPLES):
        s = sample_uniform(POS_RANGE, RPY_RANGE_DEG)
        pos = s[:3]
        rpy = s[3:]
        T0, Rm = make_pose_matrix(pos, rpy)
        # 先检查端点IK（起点/终点）
        if not check_line_reachability(ik_solver, pos, Rm, N_CHECKS_ALONG_LINE, END_EFFECTOR_AXIS, L):
            continue
        valid.append(s.tolist())
        if (i+1) % 100 == 0:
            print(f"[筛选] 进度 {i+1}/{NUM_SAMPLES}，已通过 {len(valid)}")

    valid = np.array(valid, dtype=float)
    print(f"[筛选] 完成：通过 {len(valid)} / {NUM_SAMPLES}")

    # ---------- 聚类 & 提取最大连贯子集 ----------
    print("\n[聚类] 基于六维半径连通性，提取连贯子集 ...")
    clusters = connected_components_radius(valid, eps=CLUSTER_EPS, min_pts=CLUSTER_MIN_SAMPLES)
    if len(clusters) == 0:
        print("[聚类] 未找到满足密度的连通簇。可尝试：增大 NUM_SAMPLES、调大 CLUSTER_EPS 或放宽采样范围")
        clusters = [np.arange(valid.shape[0])] if valid.shape[0] > 0 else []

    # 选择最大簇
    largest_idx = None
    max_size = -1
    for ci, idx in enumerate(clusters):
        if len(idx) > max_size:
            max_size = len(idx)
            largest_idx = idx
    print(f"[聚类] 最大簇大小：{max_size} 点")

    if largest_idx is None or max_size <= 0:
        print("[结果] 无可用点，结束。")
        # 仍保存空结果
        with open("valid_targets.json", "w") as f:
            json.dump([], f, indent=2)
        with open("largest_interval.json", "w") as f:
            json.dump({}, f, indent=2)
        simulation_app.close()
        return

    largest_pts = valid[largest_idx]

    # 先取各维 min/max 拟合一个盒子
    box_init = {
        "x":     [float(largest_pts[:,0].min()), float(largest_pts[:,0].max())],
        "y":     [float(largest_pts[:,1].min()), float(largest_pts[:,1].max())],
        "z":     [float(largest_pts[:,2].min()), float(largest_pts[:,2].max())],
        "roll":  [float(largest_pts[:,3].min()), float(largest_pts[:,3].max())],
        "pitch": [float(largest_pts[:,4].min()), float(largest_pts[:,4].max())],
        "yaw":   [float(largest_pts[:,5].min()), float(largest_pts[:,5].max())],
    }
    print("[区间] 初始盒子（按最大簇 min/max）：", {k:[round(v[0],4),round(v[1],4)] for k,v in box_init.items()})

    # 角点回检，不通过则逐步向内收缩
    box_final = shrink_box_until_valid(ik_solver, box_init, N_CHECKS_ALONG_LINE)
    print("[区间] 最终可用盒子：")
    for k,v in box_final.items():
        print(f"  {k}: [{v[0]:.4f}, {v[1]:.4f}]")

    # ---------- 保存结果 ----------
    with open("valid_targets.json", "w") as f:
        json.dump([{
            "x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
            "roll": float(p[3]), "pitch": float(p[4]), "yaw": float(p[5])
        } for p in valid], f, indent=2)

    with open("largest_interval.json", "w") as f:
        json.dump(box_final, f, indent=2)

    print("\n输出：")
    print("  - 可用点列表：valid_targets.json")
    print("  - 最大连贯区间：largest_interval.json")

    # GUI 停留片刻
    for _ in range(120):
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
