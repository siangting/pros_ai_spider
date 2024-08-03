import pybullet as p
import pybullet_data
import numpy as np
import os
import threading
import time


class pybulletIK:
    def __init__(self, first_angle):
        self.angle = first_angle
        # 初始化 PyBullet
        p.connect(p.GUI)  # 使用 GUI 模式 關閉 GUI 可改用 DIRECTOR
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 找 URDF
        p.loadURDF("plane.urdf")
        current_directory = os.path.dirname(os.path.abspath(__file__))
        urdf_file_path = os.path.join(
            current_directory, "arm_ver6", "test_abb_4600.urdf"
        )
        self.robot_id = p.loadURDF(urdf_file_path, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.base_link_index = 0  # 讓 base_link 是第一個關節

        # 將目標用小方塊顯示
        self.target_marker = p.loadURDF("cube_small.urdf", [0, 0, 0])
        self.target_marker_position = [0, 0, 0]
        # 關閉目標方塊與手臂的碰撞
        for i in range(self.num_joints):
            p.setCollisionFilterPair(self.robot_id, self.target_marker, i, -1, 0)

        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

        # 將相機用方塊表示
        self.camera_marker = p.loadURDF("cube_small.urdf", [0, 0, 0])
        p.setCollisionFilterPair(
            self.robot_id, self.camera_marker, -1, -1, 0
        )  # 停止碰撞

    def convert_to_urdf_angles(self, actual_angles):
        # 角度轉換
        urdf_angles = list(actual_angles)
        urdf_angles[0] = actual_angles[0] + np.deg2rad(90)
        urdf_angles[1] = actual_angles[1] - np.deg2rad(30)  # 第二轴 0度 对应 URDF -60度
        urdf_angles[2] = actual_angles[2] - np.deg2rad(-40)
        urdf_angles[3] = actual_angles[3] - np.deg2rad(0)  # 第4轴 0度 对应 URDF 10度
        return urdf_angles

    def relative_to_world(self, relative_position):
        # 取得 base_link 的位置和方向
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        base_orientation_matrix = p.getMatrixFromQuaternion(base_orientation)
        base_orientation_matrix = np.array(base_orientation_matrix).reshape(3, 3)

        # 相對座標換成世界座標
        world_position = np.dot(base_orientation_matrix, relative_position) + np.array(
            base_position
        )
        return world_position

    # 目前用不到
    def limit_joint_angles(self, joint_angles):
        limited_angles = []
        for angle in joint_angles:
            angle_deg = np.degrees(angle)
            if angle_deg < 0:
                angle_deg = 0
            elif angle_deg > 180:
                angle_deg = 180
            limited_angles.append(np.radians(angle_deg))
        return limited_angles

    # 目前用不到
    def convert_to_servo_angles(self, joint_angles):
        servo_angles = []
        for angle in joint_angles:
            angle_deg = np.degrees(angle)
            if angle_deg < 0:
                angle_deg = 0
            elif angle_deg > 180:
                angle_deg = 180
            servo_angles.append(angle_deg)
        return servo_angles

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

        urdf_joint_angles = current_joint_angles

        ik_solver = 0  # 使用DLS（Damped Least Squares）方法求解

        # 做 ik
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            6,  # 末端的link index
            world_target_position,
            lowerLimits=[-np.pi] * self.num_joints,
            upperLimits=[np.pi] * self.num_joints,
            jointRanges=[2 * np.pi] * self.num_joints,
            restPoses=urdf_joint_angles,
            solver=ik_solver,
        )

        # 取得關節角度
        joint_angles = joint_angles[: self.num_joints]

        # joint_angles = self.limit_joint_angles(joint_angles)

        # 關節角度只顯示小數第一位
        formatted_joint_angles = [round(angle, 1) for angle in joint_angles]
        # print("joint_angles (radians) : ", formatted_joint_angles)

        servo_angles = self.convert_to_servo_angles(joint_angles)
        print("servo_angles (degrees) : ", servo_angles)

        # 將角度套到機器手臂上
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, joint_angles[i]
            )

        return joint_angles

    def run_simulation(self):
        while True:
            p.stepSimulation()
            p.setTimeStep(1 / 240)
            p.setGravity(0, 0, 0)

    def add_camera_to_link(self, link_index, position, orientation):
        # 將方塊當成camera
        self.camera_link_index = link_index
        self.camera_position = position
        self.camera_orientation = orientation
        self.update_camera_position()

    def update_camera_position(self):
        # get link position and direction
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        link_world_position = link_state[0]
        link_world_orientation = link_state[1]
        link_world_matrix = p.getMatrixFromQuaternion(link_world_orientation)
        link_world_matrix = np.array(link_world_matrix).reshape(3, 3)

        # cal camera world position
        camera_world_position = np.dot(
            link_world_matrix, self.camera_position
        ) + np.array(link_world_position)
        p.resetBasePositionAndOrientation(
            self.camera_marker, camera_world_position, link_world_orientation
        )

    def get_camera_image(self, width, height):
        # get camera picture
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=1,
            yaw=self.camera_orientation[0],
            pitch=self.camera_orientation[1],
            roll=self.camera_orientation[2],
            upAxisIndex=2,
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=float(width) / height, nearVal=0.1, farVal=100.0
        )
        _, _, rgba_image, depth_image, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_TINY_RENDERER,
        )
        return rgba_image, depth_image


# test
def main():
    ik_solver = pybulletIK([0, 0, 0, 0, 0, 0, 0, 0])

    # add camera on link
    ik_solver.add_camera_to_link(
        link_index=6, position=[0, 0, 0.1], orientation=[0, -30, 0]
    )

    while True:
        user_input = input("Enter target coordinates relative to base_link (x y z): ")
        target_position = [float(coord) for coord in user_input.split()]
        print(f"Target position relative to base_link: {target_position}")

        current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]
        ik_solver.pybullet_move(target_position, current_joint_angles)
        ik_solver.update_camera_position()  # update camera position
        p.stepSimulation()
        time.sleep(1 / 240)  # 模擬真實時間

        rgba_image, depth_image = ik_solver.get_camera_image(width=640, height=480)


if __name__ == "__main__":
    main()
