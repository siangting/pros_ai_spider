import pybullet as p
import pybullet_data
import numpy as np
import os
import threading
import time


class pybulletIK:
    def __init__(self, first_angle):
        self.angle = first_angle
        # 初始化 PyBullet 仿真环境
        p.connect(p.GUI)  # 使用 GUI 模式，这样你可以看到仿真界面
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )  # 设置搜索路径以找到 URDF 文件
        # 加载平面和机器人 URDF 文件
        p.loadURDF("plane.urdf")
        current_directory = os.path.dirname(os.path.abspath(__file__))
        urdf_file_path = os.path.join(
            current_directory, "arm_ver6", "test_abb_4600.urdf"
        )
        self.robot_id = p.loadURDF(urdf_file_path, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.base_link_index = 0  # 假设 base_link 是第一个链接

        # 加载小方块作为目标指示器
        self.target_marker = p.loadURDF("cube_small.urdf", [0, 0, 0])
        self.target_marker_position = [0, 0, 0]
        p.changeVisualShape(
            self.target_marker, -1, rgbaColor=[0, 0, 1, 1]
        )  # 设置目标方块为蓝色

        # 停止碰撞
        for i in range(self.num_joints):
            p.setCollisionFilterPair(self.robot_id, self.target_marker, i, -1, 0)

        # 初始化相机
        self.camera_link_index = None
        self.camera_position = None
        self.camera_orientation = None

        # 将深度相机的方块变红色
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX, rgbaColor=[1, 0, 0, 1], halfExtents=[0.02, 0.02, 0.02]
        )

        # 加载方块作为相机指示器
        self.camera_marker = p.createMultiBody(baseVisualShapeIndex=visual_shape_id)
        p.setCollisionFilterPair(
            self.robot_id, self.camera_marker, -1, -1, 0
        )  # 停止碰撞

        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

    def convert_to_urdf_angles(self, actual_angles):
        # 进行角度转换
        urdf_angles = list(actual_angles)
        urdf_angles[0] = actual_angles[0] + np.deg2rad(90)
        urdf_angles[1] = actual_angles[1] - np.deg2rad(30)  # 第二轴 0度 对应 URDF -60度
        urdf_angles[2] = actual_angles[2] - np.deg2rad(
            -40
        )  # 第三轴 160度 对应 URDF 140度
        urdf_angles[3] = actual_angles[3] - np.deg2rad(0)  # 第4轴 0度 对应 URDF 10度
        return urdf_angles

    def relative_to_world(self, relative_position):
        # 获取 base_link 的位置和方向
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        base_orientation_matrix = p.getMatrixFromQuaternion(base_orientation)
        base_orientation_matrix = np.array(base_orientation_matrix).reshape(3, 3)

        # 将相对坐标转换为世界坐标
        world_position = np.dot(base_orientation_matrix, relative_position) + np.array(
            base_position
        )
        return world_position

    def pybullet_move(self, target, current_joint_angles):
        # 设置目标位置（以相对于 base_link 的坐标表示）
        target_position = target

        # 将相对坐标转换为世界坐标
        world_target_position = self.relative_to_world(target_position)

        # 更新目标指示器的位置（仅在位置改变时更新）
        if not np.allclose(
            world_target_position, self.target_marker_position, atol=1e-3
        ):
            self.target_marker_position = world_target_position
            p.resetBasePositionAndOrientation(
                self.target_marker, world_target_position, [0, 0, 0, 1]
            )

        print(f"Target marker world position: {world_target_position}")

        urdf_joint_angles = current_joint_angles
        ik_solver = 0  # 使用DLS（Damped Least Squares）方法求解

        # 执行逆运动学计算
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            6,  # 末端执行器的链接索引
            world_target_position,
            lowerLimits=[-np.pi] * self.num_joints,
            upperLimits=[np.pi] * self.num_joints,
            jointRanges=[2 * np.pi] * self.num_joints,
            restPoses=urdf_joint_angles,
            solver=ik_solver,
        )

        # 提取关节角度
        joint_angles = joint_angles[: self.num_joints]

        # 将关节角度应用到机器人模型
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, joint_angles[i]
            )

        return joint_angles

    def run_simulation(self):
        while True:
            p.stepSimulation()
            if self.camera_link_index is not None:
                self.update_camera_position()  # 在每个仿真步更新相机位置
            time.sleep(1 / 240)
            p.setTimeStep(1 / 240)
            p.setGravity(0, 0, 0)

    def add_camera_to_link(self, link_index, position, orientation):
        # 将方块作为相机附加到特定的链接上
        self.camera_link_index = link_index
        self.camera_position = position
        self.camera_orientation = orientation
        self.update_camera_position()

    def update_camera_position(self):
        if self.camera_link_index is None:
            return

        # 获取链接的位置和方向
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        link_world_position = link_state[0]
        link_world_orientation = link_state[1]
        link_world_matrix = np.array(
            p.getMatrixFromQuaternion(link_world_orientation)
        ).reshape(3, 3)

        # 计算相机的世界坐标位置
        camera_world_position = np.dot(
            link_world_matrix, self.camera_position
        ) + np.array(link_world_position)
        p.resetBasePositionAndOrientation(
            self.camera_marker, camera_world_position, link_world_orientation
        )

    def transform_camera_to_base(self, camera_position):
        """将相机坐标系中的位置转换为基座坐标系"""
        # 获取相机在世界坐标系中的位置和方向
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        camera_world_position = link_state[0]
        camera_world_orientation = link_state[1]

        # 将相机的四元数转换为旋转矩阵
        rotation_matrix = np.array(
            p.getMatrixFromQuaternion(camera_world_orientation)
        ).reshape(3, 3)

        # 将物体的相机坐标系位置转换到世界坐标系
        world_position = np.dot(rotation_matrix, camera_position) + np.array(
            camera_world_position
        )

        # 获取基座的世界坐标系位置和方向
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        base_rotation_matrix = np.array(
            p.getMatrixFromQuaternion(base_orientation)
        ).reshape(3, 3)

        # 将世界坐标系位置转换为基座坐标系位置
        relative_position = np.dot(
            base_rotation_matrix.T, world_position - np.array(base_position)
        )

        return relative_position


def main():
    ik_solver = pybulletIK([0, 0, 0, 0, 0, 0, 0, 0])

    # 将相机添加到机器人特定的链接上
    ik_solver.add_camera_to_link(
        link_index=6, position=[0, 0.05, -0.05], orientation=[0, 0, 0]
    )  # 确保 link_index 为正确的关节索引

    while True:
        user_input = input("Enter target coordinates relative to base_link (x y z): ")
        target_position = [float(coord) for coord in user_input.split()]
        print(f"Target position relative to base_link: {target_position}")

        current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]  # 根据实际情况设置初始关节角度
        ik_solver.py
