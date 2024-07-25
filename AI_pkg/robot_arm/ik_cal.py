import numpy as np
from scipy.optimize import minimize
from roboticstoolbox import DHRobot, RevoluteDH


# 定義五軸機械手臂
class FiveAxisArm(DHRobot):
    def __init__(self, lengths):
        links = [
            RevoluteDH(d=lengths[0], a=0, alpha=np.pi / 2),  # 第一個關節繞Z軸旋轉
            RevoluteDH(d=0, a=lengths[1], alpha=0),  # 第二個關節繞X軸旋轉
            RevoluteDH(d=0, a=lengths[2], alpha=0),  # 第三個關節繞X軸旋轉
            RevoluteDH(d=0, a=lengths[3], alpha=np.pi / 2),  # 第四個關節繞Z軸旋轉
            RevoluteDH(d=0, a=lengths[4], alpha=0),  # 第五個關節固定不動
        ]
        super().__init__(links, name="five_axis_arm")

    def fk(self, angles):
        T = np.eye(4)
        for i, angle in enumerate(angles):
            T = T @ self.links[i].A(angle).A
        return T[:3, 3]  # 返回末端執行器的位置


def optimize_arm(lengths, initial_angles, target_coord):
    # 創建機械手臂實例
    robot = FiveAxisArm(lengths)

    # 儲存優化過程中的座標
    optimization_path = []

    # 定義誤差函數
    def error_function(angles):
        # 固定第五軸的角度为0
        angles[4] = 0
        pos = robot.fk(angles)
        optimization_path.append((angles.copy(), pos.copy()))
        return np.linalg.norm(pos - target_coord)

    # 使用scipy.optimize.minimize進行優化
    result = minimize(
        error_function, initial_angles, bounds=[(-np.pi, np.pi)] * 4 + [(0, 0)]
    )

    # 獲取優化結果
    optimized_angles = result.x
    optimized_angles_deg = np.rad2deg(optimized_angles)

    # 打印優化結果
    print(f"Optimized joint angles (radians): {optimized_angles}")
    print(f"Optimized joint angles (degrees): {optimized_angles_deg}")

    # 計算每个关节的坐标
    joint_coords = []
    T = np.eye(4)
    for i, angle in enumerate(optimized_angles):
        T = T @ robot.links[i].A(angle).A
        joint_coords.append(T[:3, 3])

    x_coords = [coord[0] for coord in joint_coords]
    y_coords = [coord[1] for coord in joint_coords]
    z_coords = [coord[2] for coord in joint_coords]

    # 添加機械手臂的底座位置
    x_coords.insert(0, 0)
    y_coords.insert(0, 0)
    z_coords.insert(0, 0)

    # 打印每个关节的坐标
    for idx, (x, y, z) in enumerate(zip(x_coords, y_coords, z_coords)):
        print(f"Joint {idx} coordinates: x={x}, y={y}, z={z}")

    return optimized_angles_deg, joint_coords


# 示例使用
# lengths = [5, 6, 12, 12, 5]  # 根據實際情況設置
# initial_angles = [0, 0, 0, 0, 0]
# target_coord = np.array([30, 10, 20])

# optimized_angles, joint_coords = optimize_arm(lengths, initial_angles, target_coord)
