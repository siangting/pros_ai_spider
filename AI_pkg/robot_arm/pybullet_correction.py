import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import threading


class PybulletRobotController:
    def __init__(self):
        # 初始化 PyBullet 仿真环境
        p.connect(p.GUI)  # 使用 GUI 模式，这样你可以看到仿真界面
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )  # 设置搜索路径以找到 URDF 文件

        # 加载平面
        p.loadURDF("plane.urdf")

        # 加载机器人 URDF 文件
        p.loadURDF("plane.urdf")
        current_directory = os.path.dirname(os.path.abspath(__file__))
        urdf_file_path = os.path.join(
            current_directory, "arm_ver6", "test_abb_4600.urdf"
        )
        initial_position = [-3, -1, 0]
        initial_orientation = initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(
            urdf_file_path, initial_position, initial_orientation, useFixedBase=True
        )

        # 获取机器人所有关节的数量
        self.num_joints = p.getNumJoints(self.robot_id)
        print(f"Number of joints: {self.num_joints}")

        # 初始化目标关节角度
        self.target_joint_angles = [0] * self.num_joints

        # 启动仿真线程
        self.simulation_running = True
        # self.run_simulation()
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

    def set_target_joint_angles(self, angles):
        if len(angles) != self.num_joints:
            raise ValueError(
                f"Number of target angles ({len(angles)}) does not match number of joints ({self.num_joints})"
            )
        self.target_joint_angles = angles

    def apply_joint_angles(self):
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, self.target_joint_angles[i]
            )

    def run_simulation(self):
        print("Simulation running. Press Ctrl+C to exit.")
        try:
            while self.simulation_running:
                self.apply_joint_angles()
                p.stepSimulation()
                time.sleep(1 / 240)  # 控制仿真步长
        except KeyboardInterrupt:
            print("Simulation stopped by user.")
        finally:
            p.disconnect()

    def stop_simulation(self):
        self.simulation_running = False


# 示例如何实例化并使用 `PybulletRobotController`
def main():
    # 确定当前工作目录并设置 URDF 文件路径
    current_directory = os.path.dirname(os.path.abspath(__file__))
    urdf_file_path = os.path.join(current_directory, "arm_ver6", "test_abb_4600.urdf")
    print(f"!!!!!!!!!!!!!Loading URDF from: {urdf_file_path}")

    # 检查 URDF 文件是否存在
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")

    initial_position = [-3, -1, 0]  # 初始位置 (x, y, z)
    initial_orientation = p.getQuaternionFromEuler(
        [0, 0, 0]
    )  # 初始方向 (roll, pitch, yaw)

    # 创建机器人控制器实例
    controller = PybulletRobotController()

    # 示例：持续设置新的目标关节角度
    try:
        while True:
            # 获取用户输入的目标关节角度（度数）
            user_input = input(
                "Enter target joint angles (degrees, separated by spaces): "
            )
            angles_degrees = [float(angle) for angle in user_input.split()]
            angles_radians = [np.deg2rad(angle) for angle in angles_degrees]
            controller.set_target_joint_angles(angles_radians)
    except KeyboardInterrupt:
        print("User stopped the input loop.")
    finally:
        controller.stop_simulation()


if __name__ == "__main__":
    main()
