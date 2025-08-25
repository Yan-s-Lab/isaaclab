
# export_to_torchscript.py
import torch
import torch.nn as nn

CKPT = "/home/lzg/IsaacLab/logs/rsl_rl/mycobot_injection/2025-08-19_19-04-57-new-280-successs/model_1450.pt"   # 训练生成的checkpoint（含 model_state_dict）
OUT  = "/home/lzg/IsaacLab/logs/rsl_rl/mycobot_injection/2025-08-19_19-04-57-new-280-successs/model.ts"        # 导出的 TorchScript 文件


ckpt = torch.load(CKPT, map_location="cpu")
sd = ckpt["model_state_dict"]

# 取 actor 三层的参数（命名与你的权重一致）
w0, b0 = sd["actor.0.weight"], sd["actor.0.bias"]
w2, b2 = sd["actor.2.weight"], sd["actor.2.bias"]
w4, b4 = sd["actor.4.weight"], sd["actor.4.bias"]

obs_dim = w0.shape[1]      # <= 关键：自动读输入维度（你这里就是 25）
h1 = w0.shape[0]
h2 = w2.shape[0]
act_dim = w4.shape[0]

actor = nn.Sequential(
    nn.Linear(obs_dim, h1),
    nn.ELU(),
    nn.Linear(h1, h2),
    nn.ELU(),
    nn.Linear(h2, act_dim),
)
with torch.no_grad():
    actor[0].weight.copy_(w0); actor[0].bias.copy_(b0)
    actor[2].weight.copy_(w2); actor[2].bias.copy_(b2)
    actor[4].weight.copy_(w4); actor[4].bias.copy_(b4)
actor.eval()

dummy = torch.zeros(1, obs_dim)  # <= 不要手写 24，这里会自动用 25
ts = torch.jit.trace(actor, dummy)
ts.save(OUT)

print(f"[OK] Exported TorchScript: {OUT}, obs_dim={obs_dim}, act_dim={act_dim}")