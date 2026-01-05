from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg

from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.maloha import MOBILE_ALOHA_FULL_CFG  # isort: skip


@configclass
class MalohaCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # 1) Robot
        self.scene.robot = MOBILE_ALOHA_FULL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 2) Actions: 左臂关节位置控制
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["fl_joint[1-6]"],   # ✅ 只控左臂 6DoF
            scale=0.5,
            use_default_offset=True,
        )

        # 3) Actions: 左夹爪（二值开合）
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["fl_joint7", "fl_joint8"],  # 分开写，避免表达式不好匹配
            # 开：两边都回到 0
            open_command_expr={
                "fl_joint7": 0.0,
                "fl_joint8": 0.0,
            },
            # 关：7 -> +0.04, 8 -> -0.04
            close_command_expr={
                "fl_joint7": 0.04,
                "fl_joint8": -0.04,
            },
        )

        # 4) End-effector body name（⚠️ 这里你需要改成你 USD 里的真实末端 link 名）
        # 常见候选： "fl_link6" / "fl_link6_flange" / "fl_link7" / "fl_tool"
        self.commands.object_pose.body_name = "fl_link6"   # <-- 先写一个，你马上按实际改

        # 5) Object（保持不动）
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # 6) FrameTransformer：把 panda_link0/panda_hand 换成你的 base link / ee link
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"

        self.scene.ee_frame = FrameTransformerCfg(
            # ⚠️ base link prim：一般是左臂根部 link，比如 fl_link1（你按实际改）
            prim_path="{ENV_REGEX_NS}/Robot/fl_link1",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    # ⚠️ ee link prim：应当和 body_name 对应，或至少在其附近
                    prim_path="{ENV_REGEX_NS}/Robot/fl_link6",
                    name="end_effector",
                    offset=OffsetCfg(
                        # ⚠️ 这里是 TCP 偏移：先 0，等你确认末端后再调
                        pos=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
        )


@configclass
class MalohaCubeLiftEnvCfg_PLAY(MalohaCubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
