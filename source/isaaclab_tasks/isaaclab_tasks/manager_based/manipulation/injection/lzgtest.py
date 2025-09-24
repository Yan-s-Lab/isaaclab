"""
圆周点生成与位姿扩展的小工具。

提供：
- generate_circle_coords：生成圆周 (x, y, angle_deg)。
- generate_circle_params：基于 0° 原点参数扩展为 6DoF 位姿。
- strip_angle：从 7 元组位姿中去掉角度分量。
"""

import math


def generate_circle_coords(radius, step=1):
    """
    生成圆周上等角步进的二维坐标列表。

    角度约定：
    - 0° 在 (r, 0)（即 x 轴正向上的点）。
    - 角度随 `angle_deg` 增大为**逆时针**方向。
    - 坐标按 3 位小数四舍五入。

    Args:
        radius (float): 圆半径。
        step (int, optional): 角度步长（单位：度）。默认 1。会生成 angle_deg ∈ [0, 360) 的点。

    Returns:
        list[tuple[float, float, int]]: 列表元素为 (x, y, angle_deg)。
    """
    coords = []
    for angle_deg in range(0, 360, step):
        # 关键：0° 在 (r, 0)，角度增大为逆时针
        theta = math.radians(angle_deg)
        x = round(radius * math.cos(theta), 3)
        y = round(radius * math.sin(theta), 3)
        coords.append((x, y, angle_deg))
    return coords


def generate_circle_params(circle_coords, origin_params):
    """
    将圆上每个二维点扩展为 6DoF 位姿。

    给定“原点参数”(x0, y0, z0, roll0, pitch0, yaw0)，认为它对应 0° 时位姿。
    规则：
    - 位置：x、y 用圆坐标替换；z 固定为 z0。
    - 姿态：roll = roll0，pitch = pitch0，yaw = yaw0 + radians(angle_deg)。

    Args:
        circle_coords (list[tuple[float, float, int]]): 由 `generate_circle_coords`
            生成的 (x, y, angle_deg) 列表。
        origin_params (tuple[float, float, float, float, float, float]): 在 0° 时的
            6 维参数 (x0, y0, z0, roll0, pitch0, yaw0)。

    Returns:
        list[tuple[float, float, float, float, float, float, int]]:
            列表元素为 (x, y, z, roll, pitch, yaw, angle_deg)。
    """
    x0, y0, z0, roll0, pitch0, yaw0 = origin_params
    params = []
    for (x, y, angle_deg) in circle_coords:
        roll = roll0
        pitch = pitch0
        yaw = round(yaw0 + math.radians(angle_deg), 3)
        params.append((x, y, z0, roll, pitch, yaw, angle_deg))
    return params


def strip_angle(data):
    """
    裁剪 7 元组 (x, y, z, roll, pitch, yaw, angle_deg) 为前 6 项。

    通常用于去掉角度标注，仅保留 6DoF 参数。

    Args:
        data (list[tuple]): 每个元素为含 7 个分量的元组。

    Returns:
        list[tuple]: 每个元素为 6 分量的元组 (x, y, z, roll, pitch, yaw)。
    """
    return [tuple(item[:6]) for item in data]


# # 示例：半径 0.16，步长 90 度
# if __name__ == "__main__":
#     points = generate_circle_coords(radius=0.16, step=1)
#     print(f"Total points: {len(points)}")
#     for p in points:
#         print(p)

if __name__ == "__main__":
    from pprint import pprint

    # 圆上坐标
    circle = generate_circle_coords(radius=0.16, step=1)
    # print(f"Circle points: {circle}")
    # 原点参数
    origin = (0.16, 0.0, 0.29, -1.57, 0, -1.57)
    # 生成 6 维度坐标
    result = generate_circle_params(circle, origin)
    reulsta =strip_angle(result)
    print(reulsta)
    # pprint(strip_angle(result))
