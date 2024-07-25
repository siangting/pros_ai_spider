import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK


class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.ik_solver = pybulletIK([0] * 8)  # 假设有8个关节，初始角度为0
        self.current_angle = [90, 0, 160, 50, 10, 0, 0, 0]  # 初始关节角度

    def degree_to_radians(self, data):
        return list(np.radians(data))

    def end_action(self):
        end_action_angle = self.degree_to_radians([90, 0, 160, 50, 10])
        self.node.publish_arm(end_action_angle)

    def action(self):
        print("data receiving")
        while True:
            data = self.node.wait_for_data()

            target_coord = data["amr_target_position"]
            radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)
            self.node.publish_arm(radians)
            self.current_angle = radians  # 更新当前角度

            # self.ik_solver.run_simulation()
