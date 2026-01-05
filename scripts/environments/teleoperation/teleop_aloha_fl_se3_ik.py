import argparse
import torch

from isaaclab.app import AppLauncher
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg

from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices import Se3Keyboard
from isaaclab.utils.math import apply_delta_pose, subtract_frame_transforms


def build_exact_regex(names):
    # 生成 "^(name1|name2|...|nameN)$"
    # 保证精确匹配，不会误匹配到别的关节
    inner = "|".join([f"{n}" for n in names])
    return f"^({inner})$"


def main():
    parser = argparse.ArgumentParser()

    # Isaac Lab / Isaac Sim app args
    AppLauncher.add_app_launcher_args(parser)

    # ===== 你需要关心的参数 =====
    parser.add_argument("--usd", required=True, help="Path to mobile aloha USD inside container")
    # 你说 USD 内部的 articulation root 是 /aloha_description/aloha_description/root_joint
    # 我们默认把 USD spawn 到 /World/Robot 下，所以真正路径会变成 /World/Robot + 下面这个相对路径
    parser.add_argument(
        "--art_root_rel",
        default="/aloha_description/aloha_description/root_joint",
        help="Articulation root path INSIDE the USD (relative path). Example: /aloha_description/aloha_description/root_joint",
    )
    parser.add_argument(
        "--spawn_root",
        default="/World/Robot",
        help="Where to spawn the USD in the stage. Default: /World/Robot",
    )

    # 机械臂命名规则：fl_joint1-8 / fl_link1-8
    parser.add_argument("--arm_prefix", default="fl", help="Joint/link prefix, e.g. fl or fr")
    parser.add_argument(
        "--ee_link",
        default="fl_link6",
        help="End-effector link name (e.g., fl_link6 or fl_link8). MUST match exactly.",
    )

    # 手感参数：把键盘输出的 delta 再乘个系数
    parser.add_argument("--pos_scale", type=float, default=1.0, help="Scale translation delta from keyboard")
    parser.add_argument("--rot_scale", type=float, default=1.0, help="Scale rotation delta from keyboard")

    # 夹爪（如果 Se3Keyboard 输出第7维，就用它；否则夹爪保持不动）
    parser.add_argument("--grip_step", type=float, default=0.03, help="Gripper alpha step per tick (when cmd has gripper dim)")

    args = parser.parse_args()

    # ----------------------------
    # Launch app
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # ----------------------------
    # Simulation
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device))
    sim.set_camera_view([2.5, 0.0, 2.0], [0.0, 0.0, 1.0])

    # Ground + light
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg(intensity=3000.0))

    # ----------------------------
    # Spawn USD
    # 注意：这里的 spawn_root 是外层挂载点，但我们要控制的是 articulation root（root_joint 那层）
    robot_root_path = args.spawn_root + args.art_root_rel

    robot_cfg = ArticulationCfg(
        prim_path=robot_root_path,
        spawn=sim_utils.UsdFileCfg(usd_path=args.usd),
    )
    robot = Articulation(cfg=robot_cfg)

    # ----------------------------
    # IK Controller
    ik_cfg = DifferentialIKControllerCfg(
        command_type="pose",      # 目标是 (pos, quat)
        use_relative_mode=False,  # 我们自己维护绝对目标
        ik_method="dls",          # DLS 更稳
    )
    ik = DifferentialIKController(ik_cfg, num_envs=1, device=sim.device)

    # ----------------------------
    # Teleop device
    teleop = Se3Keyboard()

    # Reset + warmup
    sim.reset()
    dt = sim.get_physics_dt()
    sim.step()
    robot.update(dt)

    # ----------------------------
    # Build joint/link names
    arm_prefix = args.arm_prefix
    arm_joints = [f"{arm_prefix}_joint{i}" for i in range(1, 7)]   # 1~6
    grip_joints = [f"{arm_prefix}_joint7", f"{arm_prefix}_joint8"] # 7~8

    # Find joints / bodies
    arm_joint_ids, arm_joint_names = robot.find_joints(build_exact_regex(arm_joints))
    grip_joint_ids, grip_joint_names = robot.find_joints(build_exact_regex(grip_joints))
    ee_body_ids, ee_body_names = robot.find_bodies(f"^({args.ee_link})$")

    if len(arm_joint_ids) != 6:
        raise RuntimeError(f"Arm joints not matched. Expect: {arm_joints}, got: {arm_joint_names}")
    if len(grip_joint_ids) != 2:
        raise RuntimeError(f"Gripper joints not matched. Expect: {grip_joints}, got: {grip_joint_names}")
    if len(ee_body_ids) != 1:
        raise RuntimeError(f"EE link not unique/found. Expect exactly one: {args.ee_link}, got: {ee_body_names}")

    ee_body_id = ee_body_ids[0]
    arm_joint_ids_t = torch.tensor(arm_joint_ids, device=sim.device, dtype=torch.long)
    grip_joint_ids_t = torch.tensor(grip_joint_ids, device=sim.device, dtype=torch.long)

    print("\n================= CONFIG =================")
    print("USD:", args.usd)
    print("Spawn root:", args.spawn_root)
    print("Articulation root inside USD:", args.art_root_rel)
    print("=> Controlled prim_path:", robot_root_path)
    print("Arm joints:", arm_joint_names)
    print("Gripper joints:", grip_joint_names)
    print("EE link:", ee_body_names[0])
    print("pos_scale:", args.pos_scale, "rot_scale:", args.rot_scale)
    print("=========================================\n")

    # ----------------------------
    # Gripper limits -> map alpha in [0,1] to joint positions
    # alpha=0 -> closed (lo), alpha=1 -> open (hi)
    # 注意：不同模型 joint limit 方向可能相反，但这个映射仍然可用（alpha 增大就是朝 hi 走）
    joint_limits = robot.data.joint_limits[0]  # (num_joints, 2) for env0
    grip_lo = joint_limits[grip_joint_ids_t, 0].clone()
    grip_hi = joint_limits[grip_joint_ids_t, 1].clone()
    grip_alpha = torch.tensor([1.0], device=sim.device)  # 默认先开

    # ----------------------------
    # Init EE target = current EE pose in base/root frame
    # root_pose_w: robot root in world
    # ee_pose_w: ee body in world
    root_pose_w = robot.data.root_state_w[:, :7]               # (1,7)
    ee_pose_w = robot.data.body_state_w[:, ee_body_id, :7]     # (1,7)

    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3], ee_pose_w[:, 3:7],
    )

    print("[INFO] 操作前：用鼠标点一下 Isaac Sim viewport，让窗口获得键盘焦点。")
    print("[INFO] 键位（Se3Keyboard 默认）：W/S A/D Q/E 平移；Z/X T/G C/V 旋转；R 重置；K 夹爪（若该版本输出夹爪维度）。\n")

    # ----------------------------
    # Main loop
    while simulation_app.is_running():
        # Read teleop command
        cmd = teleop.advance().to(sim.device)

        # cmd 通常至少 6 维: [dx, dy, dz, dRx, dRy, dRz]
        delta_pose = cmd[0:6].view(1, 6).clone()
        delta_pose[:, 0:3] *= args.pos_scale
        delta_pose[:, 3:6] *= args.rot_scale

        # Update EE target in base frame
        ee_pos_b, ee_quat_b = apply_delta_pose(ee_pos_b, ee_quat_b, delta_pose)

        # Set IK command (absolute pose)
        ik_command = torch.cat([ee_pos_b, ee_quat_b], dim=-1)  # (1,7)
        ik.set_command(ik_command)

        # Jacobian
        jac = robot.root_physx_view.get_jacobians()  # (1, bodies, 6, dof)
        # 重要：fixed-base vs floating-base 索引偏移
        ee_jacobi_idx = ee_body_id - 1 if robot.is_fixed_base else ee_body_id
        jac_ee = jac[:, ee_jacobi_idx, :, arm_joint_ids_t]

        # Current arm joint pos
        joint_pos = robot.data.joint_pos[:, arm_joint_ids_t]

        # Compute desired joint pos
        joint_pos_des = ik.compute(ee_pos_b, ee_quat_b, jac_ee, joint_pos)

        # Apply to arm
        robot.set_joint_position_target(joint_pos_des, joint_ids=arm_joint_ids)

        # Gripper: only if cmd has 7th dim (some versions expose it)
        if cmd.numel() >= 7:
            g = float(cmd[6].item())  # typically -1..1
            grip_alpha = torch.clamp(grip_alpha + args.grip_step * torch.tensor([g], device=sim.device), 0.0, 1.0)

        grip_q = grip_lo + grip_alpha * (grip_hi - grip_lo)     # (2,)
        robot.set_joint_position_target(grip_q.view(1, 2), joint_ids=grip_joint_ids)

        # Write + step
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    simulation_app.close()


if __name__ == "__main__":
    main()
