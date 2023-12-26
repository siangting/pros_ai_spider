from rclpy.node import Node
import rclpy
import math
from std_msgs.msg import Float32MultiArray
import threading
from collections import defaultdict
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import json
from std_msgs.msg import String
from car_models import *
import sys
import math
from tools import *

class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")  # ros2Ai #unity2Ros
        self.real_car_data = {}
        self.data_to_AI = ""
        self.prev_car_yaw = 0
        self.real_car_data['ROS2CarPosition'] = []
        self.real_car_data['ROS2CarQuaternion'] = []
        self.real_car_data['ROS2TargetPosition'] = []
        self.real_car_data['ROS2WheelAngularVelocityLeftBack'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityLeftFront'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityRightBack'] = 0.01
        self.real_car_data['ROS2WheelAngularVelocityRightFront'] = 0.01
        self.real_car_data['ROS2Range'] = []
        self.real_car_data['ROS2RangePosition'] = []

        self.subscriber_amcl = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                                        self.subscribe_callback_amcl, 10)
        self.subscriber_goal = self.create_subscription(PoseStamped, "/goal", self.subscribe_callback_goal, 10)
        self.subscriber_lidar = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.subscriber_rear = self.create_subscription(String, DeviceDataTypeEnum.car_D_state,
                                                        self.rear_wheel_callback, 10)
        self.subscriber_forward = self.create_subscription(String, DeviceDataTypeEnum.car_D_state_front,
                                                           self.forward_wheel_callback, 10)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.publisher_to_AI = self.create_publisher(
            String,
            "real_car_data",
            10
        )

    def quaternion_2_euler(self,x,y,z,w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            # use 90 degrees if out of range
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        roll_deg = math.degrees(roll) % 360
        pitch_deg = math.degrees(pitch) % 360
        yaw_deg = math.degrees(yaw) % 360
        return roll_deg, pitch_deg, yaw_deg

    def subscribe_callback_amcl(self, message):
        pose = self.transfer_car_pose(message)[0]
        quaternion = self.transfer_car_pose(message)[1]
        self.real_car_data['ROS2CarPosition'] = pose
        self.real_car_data['ROS2CarQuaternion'] = quaternion
        
        

    def transfer_car_pose(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return [position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w]

    def subscribe_callback_goal(self, message): #  有訊號才會近來
        target = self.transfer_target_pos(message)
        self.real_car_data['ROS2TargetPosition'] = target
            
        
        
        

    def transfer_target_pos(self, msg):
        position = msg.pose.position
        return [position.x, position.y, position.z]

    def laser_scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges_180 = []
        direction_180 = []
        all_ranges = msg.ranges
        for i in range(len(all_ranges)):
            if i % 5 == 0: # 處理lidar的數量
                angle_tmp = angle_min + i * angle_increment
                ranges_180.append(all_ranges[i])
                direction_180.append([math.cos(angle_tmp), math.sin(angle_tmp), 0])
        self.real_car_data['ROS2Range'] = ranges_180
        self.real_car_data['ROS2RangePosition'] = direction_180

    def rear_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if 'data' in data and 'vels' in data['data']:
                vels = data['data']['vels']
                # print(f"rear vels: {vels}")
                # TODO: using vels
                self.real_car_data['ROS2WheelAngularVelocityLeftBack'] = vels[0]
                self.real_car_data['ROS2WheelAngularVelocityRightBack'] = vels[1]
                # TODO:
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def forward_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if 'data' in data and 'vels' in data['data']:
                vels = data['data']['vels']
                # print(f"foward vels: {vels}")
                self.real_car_data['ROS2WheelAngularVelocityLeftFront'] = vels[0]
                self.real_car_data['ROS2WheelAngularVelocityRightFront'] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def timer_callback(self):
        flag = True
        for key, value in self.real_car_data.items():
            if isinstance(value, list) and not value:
                print(f"{key} is an empty list.")
                flag = False
            elif not value:
                print(f"{key} is 0.")
                # flag = False
        # print(self.real_car_data)
        self.car_data = json.dumps(self.real_car_data)
        if flag:
            data_to_AI = String()
            data_to_AI.data = self.car_data
            self.publisher_to_AI.publish(data_to_AI)
            # print(data_to_AI)


def spin_pros(node):
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(node)
    exe.spin()
    rclpy.shutdown()
    sys.exit(0)


def main():
    rclpy.init()
    node = AiNode()
    pros = threading.Thread(target=spin_pros, args=(node,))
    pros.start()


if __name__ == '__main__':
    main()
