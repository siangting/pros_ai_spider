import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK

# from robot_arm.pybullet_correction import PybulletRobotController


class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.ik_solver = pybulletIK([0] * 8)
        self.ik_solver.add_camera_to_link(
            link_index=6, position=[0, 0, 0.1], orientation=[0, -30, 0]
        )
        self.current_angle = [90, 0, 160, 50, 10, 0, 0, 0]

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

            radians = list(radians)
            radians[0] += np.deg2rad(270)
            radians[1] += np.deg2rad(30)
            radians[2] += np.deg2rad(30)
            # radians[2] -= np.deg2rad(40)
            radians = radians[0:5]
            self.node.publish_arm(radians)
            time.sleep(1)
            self.current_angle = radians  # 更新当前角度
            self.ik_solver.update_camera_position()

            rgba_image, depth_image = self.ik_solver.get_camera_image(
                width=640, height=480
            )
