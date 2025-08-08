import numpy as np
import math

def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles (XYZ order) to quaternion (w, x, y, z).
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])
def euler_zyx_to_quat(roll, pitch, yaw):
    # roll=p (X), pitch=q (Y), yaw=r (Z)
    # 先绕 X，再绕 Y，最后绕 Z => q = q_z * q_y * q_x
    import math, numpy as np
    # 每轴小四元数
    qx = np.array([math.cos(roll/2), math.sin(roll/2), 0, 0])
    qy = np.array([math.cos(pitch/2), 0, math.sin(pitch/2), 0])
    qz = np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
    # Hamilton 乘法：注意顺序
    def mul(a,b):
        w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
        return np.array([
          w1*w2 - x1*x2 - y1*y2 - z1*z2,
          w1*x2 + x1*w2 + y1*z2 - z1*y2,
          w1*y2 - x1*z2 + y1*w2 + z1*x2,
          w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])
    return mul(qz, mul(qy, qx))

def quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw), XYZ order.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate of quaternion q = [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of quaternions a and b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat_to_rot_mat(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion q = [w,x,y,z] to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y + z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x + y*y)]
    ])

def world_to_base_euler(
    p_root_w: np.ndarray,
    q_root_w: np.ndarray,
    p_w:       np.ndarray,
    roll_w:    float,
    pitch_w:   float,
    yaw_w:     float
) -> tuple[np.ndarray, tuple[float, float, float]]:
    """
    Convert a world-frame Euler pose to base-frame Euler pose.

    Inputs:
      - p_root_w:   root position in world [3]
      - q_root_w:   root quaternion in world [4] (w,x,y,z)
      - p_w:        target position in world [3]
      - roll_w/...: target Euler angles in world (rad)
    Returns:
      - p_b:        position in base-frame [3]
      - euler_b:    Euler angles in base-frame (roll, pitch, yaw)
    """
    # 1) world Euler -> world quaternion
    # q_w = euler_to_quat(roll_w, pitch_w, yaw_w)
    q_w = euler_zyx_to_quat(roll_w, pitch_w, yaw_w)


    # 2) compute R_root from q_root_w
    R_root = quat_to_rot_mat(q_root_w)

    # 3) position: p_b = R_root^T * (p_w - p_root_w)
    p_b = R_root.T.dot(p_w - p_root_w)

    # 4) orientation: q_b = conj(q_root_w) * q_w
    q_b = quat_mul(quat_conjugate(q_root_w), q_w)

    # 5) quaternion -> Euler for base-frame
    euler_b = quat_to_euler(*q_b)

    return p_b, euler_b

if __name__ == "__main__":
    # Example inputs
    p_root_w = np.array([0.0, -0.00, -0.00])      # root pos in world
    q_root_w = np.array([1.0,  -0.0,  0.0,  0.0]) # root quat in world
    # 上面两个是固定的，获取到的基础的机械臂底座的原点的坐标，实际坐标加上四元数坐标。 
    # world 坐标是我想表达的真实坐标，是基于机械臂底座为世界原点，全部都是0,0,0，0,0，的情况下的坐标。
    # 然后输出后的base坐标是基于root的基础上变换过的，是isaacsim 可以理解的坐标
    # 言而总之： p_w是我想要的x，y，z坐标。 roll_w, pitch_w, yaw_w是我真实想要的欧拉角状态。
    # 输出的是我需要给pose生成器配置的坐标。

        
    p_w = np.array([0.2, 0.0, 0.30])      # target pos in world
    # target Euler angles in world (rad)
    # 分别是绕x，y，z轴旋转
    roll_w, pitch_w, yaw_w = 1.572, 1.57, 1.572   # rad表示中180度pi（3.14）90度是1.57, 45度是0.785，
    # 下面是验证用的，验证结果显示，结果与实际相符。验收通过。
    # p_w = np.array([-0.2460, -0.031, 0.0370])      # target pos in world
    # # target Euler angles in world (rad)
    # roll_w, pitch_w, yaw_w = -1.435, 3.14200, -1.629    

    p_b, (roll_b, pitch_b, yaw_b) = world_to_base_euler(
        p_root_w, q_root_w, p_w, roll_w, pitch_w, yaw_w
    )
    print("Base-Frame Position   :", p_b.tolist())
    print(f"Base-Frame Euler (rad): roll={roll_b:.3f}, pitch={pitch_b:.3f}, yaw={yaw_b:.3f}")
    print(f"Base-Frame Euler (°)  : roll={math.degrees(roll_b):.1f}°, "
          f"pitch={math.degrees(pitch_b):.1f}°, yaw={math.degrees(yaw_b):.1f}°")#

