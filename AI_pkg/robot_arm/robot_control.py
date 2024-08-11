import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK
import cv2


class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.current_angle = [
            np.deg2rad(90),
            np.deg2rad(0),
            np.deg2rad(160),
            0,
            0,
            0,
            0,
            0,
        ]
        self.ik_solver = pybulletIK(self.current_angle)
        # self.ik_solver.add_camera_to_link(
        #     link_index=3, position=[0, 0.1, -0.05], orientation=[0, 0, 0]
        # )
        # self.current_angle = [0, 0, 0, 0, 0, 0, 0, 0]  # 初始关节角度设置为0

    def degree_to_radians(self, data):
        return list(np.radians(data))

    def end_action(self):
        end_action_angle = self.degree_to_radians([90, 0, 160, 50, 10])
        self.node.publish_arm(end_action_angle)

    def action(self):
        print("data receiving")
        while True:
            data = None
            while data == None:
                data = self.node.get_target_pos()

            # 获取目标坐标
            target_coord = data

            # 使用逆运动学计算关节角度
            radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)

            # 调整关节角度
            # radians = list(radians)
            # radians[0] += np.deg2rad(270)
            # radians[1] += np.deg2rad(30)
            # radians[2] += np.deg2rad(30)
            # radians = radians[0:5]

            # 发布关节角度到机械臂
            # self.node.publish_arm(radians)
            # time.sleep(1)
            # self.current_angle = radians  # 更新当前角度


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("robot_arm_control_node")
    robot_arm_control = RobotArmControl(node)

    try:
        robot_arm_control.action()
    except KeyboardInterrupt:
        robot_arm_control.end_action()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
