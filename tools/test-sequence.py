import numpy as np
import math

# 目标四元数（Sim 中打印出来的）
q_target = np.array([0.5, -0.5, 0.5, -0.5])

# 你的 Euler 输入
roll,pitch,yaw = 1.57, 1.57, 1.57

# 每轴单独小四元数
def qx(r): return np.array([math.cos(r/2), math.sin(r/2), 0.0, 0.0])
def qy(p): return np.array([math.cos(p/2), 0.0, math.sin(p/2), 0.0])
def qz(y): return np.array([math.cos(y/2), 0.0, 0.0, math.sin(y/2)])

# Hamilton 乘法
def mul(a,b):
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

orders = {
    "XYZ (intrinsic)": lambda: mul(qz(yaw), mul(qy(pitch), qx(roll))),
    "ZYX (intrinsic)": lambda: mul(qx(roll), mul(qy(pitch), qz(yaw))),
    "Z→Y→X (extrinsic)": lambda: mul(qz(yaw), mul(qy(pitch), qx(roll))),
    "X→Y→Z (extrinsic)": lambda: mul(qx(roll), mul(qy(pitch), qz(yaw))),
}

print("target quaternion:", q_target)
for name, fn in orders.items():
    q = fn()
    print(f"{name:>20} →", np.round(q,3), " match?", np.allclose(q, q_target, atol=1e-3))
