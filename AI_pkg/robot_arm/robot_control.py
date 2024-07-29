import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK

# from robot_arm.pybullet_correction import PybulletRobotController


class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.ik_solver = pybulletIK([0] * 8)  # 假设有8个关节，初始角度为0
        self.current_angle = [90, 0, 160, 50, 10, 0, 0, 0]  # 初始关节角度

        # self.pybullet_correct = PybulletRobotController()

    def degree_to_radians(self, data):
        return list(np.radians(data))

    def end_action(self):
        end_action_angle = self.degree_to_radians([90, 0, 160, 50, 10])
        self.node.publish_arm(end_action_angle)

    def action(self):
        print("data receiving")
        while True:
            data = self.node.get_target_pos()

            # target_coord = data["amr_target_position"]
            target_coord = data
            radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)

            # degree = [90, 0, 160, 50, 10]
            # radians = self.degree_to_radians(degree)
            radians = list(radians)
            radians[0] += np.deg2rad(270)
            radians[1] += np.deg2rad(30)
            radians[2] += np.deg2rad(30)
            # radians[2] -= np.deg2rad(40)
            radians = radians[0:5]
            self.node.publish_arm(radians)
            time.sleep(1)
            self.current_angle = radians  # 更新当前角度

            # radians = self.degree_to_radians(
            #     [
            #         degree[0] + 90,
            #         degree[1] - 30,
            #         degree[2] - 40,
            #         degree[3] - 0,
            #         degree[4] - 0,
            #     ]
            # )
            # self.pybullet_correct.set_target_joint_angles(radians + [0, 0, 0])

            # self.ik_solver.run_simulation()
