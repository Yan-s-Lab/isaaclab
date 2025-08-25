
from .simple_ik_helper import SimpleIKHelper
# mdp/simple_ik_helper.py 里不变

# mdp/rewards.py 或 mdp/__init__.py 里
def init_bc_ik_helper(env, env_ids, damping: float = 1e-3, max_iters: int = 128, tol: float = 1e-3):
    # env_ids 这次用不到，但必须保留以满足 EventManager 的签名要求
    robot = env.scene["robot"]
    eef_name = "joint6_flange"
    try:
        from .simple_ik_helper import SimpleIKHelper
    except Exception:
        # 如果放在别处，改成正确的导入路径
        from isaaclab_tasks.manager_based.manipulation.injection.mdp.simple_ik_helper import SimpleIKHelper

    env._bc_ik_helper = SimpleIKHelper(robot, eef_name, damping, max_iters, tol)
    print(f"[init_bc_ik_helper] ready (eef='{eef_name}', damping={damping}, iters={max_iters}, tol={tol})")
