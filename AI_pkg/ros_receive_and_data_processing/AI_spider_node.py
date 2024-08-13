from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
    Twist,
    TransformStamped,
    Point,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData, Odometry
import random
from std_msgs.msg import Header
import json
from std_msgs.msg import String
import orjson
import math
import time
from rclpy.node import Node
from ros_receive_and_data_processing.car_models import *
from ros_receive_and_data_processing.data_transform import preprocess_data
from ros_receive_and_data_processing.config import (
    ACTION_MAPPINGS,
    LIDAR_PER_SECTOR,
    NEXT_POINT_DISTANCE,
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from PIL import Image
import yaml
import os
from tf2_ros import StaticTransformBroadcaster
import roslibpy
from std_msgs.msg import Float32MultiArray


class AI_spider_node(Node):
    def __init__(self):
        super().__init__("AI_spider_node")
        self.get_logger().info("Ai spider start")
        self.spider_data = {}
        self.data_to_AI = ""
        self.lastest_data = None

        self.data_updated = {
            "center": False,
        }

        self.subscriber_spider_center = self.create_subscription(
            Float32MultiArray,
            "/spider_center",
            self.subscribe_callback_spider_center,
            10
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spider_joint_trajectory_point',
            10
        )
        


    def subscribe_callback_spider_center(self, msg):
        self.spider_data["center_position"] = msg.data
        self.update_and_check_data("center")
        # print(self.spider_center_position)

    def update_and_check_data(self, data_type):
        self.data_updated[data_type] = True
        self.check_and_get_lastest_data()
    
    def check_and_get_lastest_data(self):
        if all(self.data_updated.values()):
            # 確認所有的數據都更新並publish
            self.data_updated["center"] = False
            self.lastest_data = preprocess_data(self.spider_data)

    # call by RL_utils.get_observation
    def wait_for_data(self):
        spider_state = self.get_latest_data()
        while spider_state is None:
            spider_state = self.get_latest_data()
        return spider_state

    def get_latest_data(self):
        return self.lastest_data

    def publish_to_robot(self, actions):
        joint_pos = []
        # print(actions)
        for action in actions:
            if action == 0:
                joint_pos.append(math.radians(-30))
            elif action == 1:
                joint_pos.append(math.radians(0))
            elif action == 2:
                joint_pos.append(math.radians(30))
        self.pub_jointpos(joint_pos)

    def pub_jointpos(self, joint_pos):
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in joint_pos]
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)



    def get_target_pos(self):
        return self.real_car_data["arm_tartget_position"]

    def reset(self):
        self.lastest_data = None
        # self.publish_to_robot("STOP", pid_control=False)


