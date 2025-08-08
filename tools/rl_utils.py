# rl_utils.py
def get_current_joint_positions(env, env_id=0):
    return env._articulation.data.joint_pos[env_id].cpu().numpy().tolist()
