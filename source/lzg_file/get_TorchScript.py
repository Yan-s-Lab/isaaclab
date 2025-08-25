# extract_obs_stats.py
import torch, numpy as np

CKPT = "/home/lzg/IsaacLab/logs/rsl_rl/mycobot_injection/2025-08-19_19-04-57-new-280-successs/model_1450.pt"

ckpt = torch.load(CKPT, map_location="cpu", weights_only=False)
print("=== ckpt keys ===", ckpt.keys())
infos = ckpt.get("infos", {})
print("=== infos keys ===", list(infos.keys()))

mean = std = None

# 常见几种命名，按顺序尝试
candidates = [
    ("obs_rms", ("mean", "var")),                 # infos["obs_rms"]["mean"/"var"]
    ("ob_rms", ("mean", "var")),                  # infos["ob_rms"]["mean"/"var"]
    ("running_mean_std", ("mean", "var")),        # infos["running_mean_std"]["mean"/"var"]
    ("vecnorm", ("obs_mean", "obs_var")),         # infos["vecnorm"]["obs_mean"/"obs_var"]
    ("obs_norm", ("mean", "var")),                # infos["obs_norm"]["mean"/"var"]
]

for root, (mk, vk) in candidates:
    if root in infos:
        blob = infos[root]
        if isinstance(blob, dict) and mk in blob and vk in blob:
            mean = np.array(blob[mk], dtype=np.float32)
            var  = np.array(blob[vk], dtype=np.float32)
            std  = np.sqrt(np.maximum(var, 1e-12))
            print(f"[FOUND] {root}.{mk}/{vk} -> mean.shape={mean.shape}, std.shape={std.shape}")
            break

# 有些实现直接放在顶层 infos
if mean is None:
    if "obs_mean" in infos and "obs_std" in infos:
        mean = np.array(infos["obs_mean"], dtype=np.float32)
        std  = np.array(infos["obs_std"],  dtype=np.float32)
        print(f"[FOUND] infos.obs_mean/obs_std -> {mean.shape}, {std.shape}")

if mean is None:
    print("!! 没找到 obs 归一化统计（mean/std）。先用“无归一化”跑，或回头查训练脚本/日志里是否有保存。")
else:
    np.savez("obs_stats.npz", mean=mean, std=std)
    print("=> 已保存 obs_stats.npz（包含 mean/std）")
