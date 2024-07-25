import pybullet as p
import pybullet_data
import numpy as np
import os


class pybulletIK:
    def __init__(self, first_angle):
        self.angle = first_angle
        # 初始化 PyBullet 仿真环境
        p.connect(p.DIRECT)  # 使用 DIRECT 模式，不需要图形界面
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

    def convert_to_urdf_angles(self, actual_angles):
        # 进行角度转换
        urdf_angles = list(actual_angles)
        urdf_angles[1] = actual_angles[1] - np.deg2rad(60)  # 第二轴 0度 对应 URDF -60度
        urdf_angles[2] = actual_angles[2] - np.deg2rad(
            20
        )  # 第三轴 160度 对应 URDF 140度
        urdf_angles[3] = actual_angles[3] - np.deg2rad(-10)  # 第4轴 0度 对应 URDF 10度
        return urdf_angles

    def world_to_base_link(self, world_position):
        # 获取 base_link 的位置和方向
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        base_orientation_matrix = p.getMatrixFromQuaternion(base_orientation)
        base_orientation_matrix = np.array(base_orientation_matrix).reshape(3, 3)

        # 将世界坐标转换为相对于 base_link 的坐标
        relative_position = np.dot(
            np.linalg.inv(base_orientation_matrix),
            np.array(world_position) - np.array(base_position),
        )
        return relative_position

    def pybullet_move(self, target, current_joint_angles):
        # 设置目标位置（以世界坐标系表示）
        target_position = target

        urdf_joint_angles = self.convert_to_urdf_angles(current_joint_angles)

        # 转换目标位置为相对于 base_link 的坐标
        relative_target_position = self.world_to_base_link(target_position)

        # 设置逆运动学的解算参数，包括当前关节角度作为初始值
        ik_solver = 0  # 使用DLS（Damped Least Squares）方法求解

        # 执行逆运动学计算，加入初始关节角度
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            6,  # 末端执行器的链接索引
            relative_target_position,
            lowerLimits=[-np.pi] * self.num_joints,
            upperLimits=[np.pi] * self.num_joints,
            jointRanges=[2 * np.pi] * self.num_joints,
            restPoses=urdf_joint_angles,
            solver=ik_solver,
        )

        # 提取关节角度
        joint_angles = joint_angles[: self.num_joints]

        # 打印优化结果
        print(f"Joint angles (radians): {joint_angles}")
        print(f"Joint angles (degrees): {np.degrees(joint_angles)}")

        # 应用关节角度到机器人模型
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, joint_angles[i]
            )

        return joint_angles

    # def run_simulation(self):
    #     while True:
    #         p.stepSimulation()
    #         # 控制仿真步长
    #         p.setTimeStep(1 / 240)
    #         p.setGravity(0, 0, -9.81)


# if __name__ == "__main__":
#     ik_solver = pybulletIK([0, 0, 0, 0, 0, 0, 0, 0])

#     while True:
#         user_input = input("Enter target coordinates (x y z): ")
#         target_position = [float(coord) for coord in user_input.split()]
#         print(f"Target position: {target_position}")

#         current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]  # 根据实际情况设置初始关节角度
#         ik_solver.pybullet_move(target_position, current_joint_angles)
#         p.stepSimulation()
#         time.sleep(1 / 240)  # 控制仿真步长
