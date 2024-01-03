import torch
from datetime import datetime
from RealCarAdaptor import transfer_obs
# from Entity import State
import threading
import sys
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import csv
from datetime import datetime
import orjson
from car_models import *
import torch
import torch.nn as nn
import torch.nn.functional as F
from tools import *
from avoidance import refined_obstacle_avoidance_with_target_orientation
import time

DEG2RAD = 0.01745329251
unityState = ""


class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")  # ros2Ai #unity2Ros
        self.subsvriber_ = self.create_subscription(String, "/real_car_data", self.receive_data_from_ros, 10)
        self.dataList = list()
        self.publisher_Ai2ros = self.create_publisher(String, DeviceDataTypeEnum.car_B_control, 10)  # Ai2ros #ros2Unity

        input_size = 182
        hidden_size1 = 128  # adjust if need
        hidden_size2 = 64
        output_size = 2

        # self.loaded_model_1 = MLP(input_size, hidden_size1, hidden_size2, output_size)

        # self.loaded_model_1.load_state_dict(
        #     torch.load("./dataFile/ver2.pth", map_location=torch.device('cpu')))  # model loading
        # self.loaded_model_1.apply(self.init_weights)

    def init_weights(self, m):
        if type(m) == nn.Linear:
            nn.init.xavier_uniform_(m.weight)
            m.bias.data.fill_(0.01)

    def wheel_speed_transform(self, action):
        speed = 14.0
        if action == -1:  # stop
            left = 0.0625
            right = 0.0625
        elif action == 0:  # forward
            left = speed
            right = speed
        elif action == 1:  # left turn
            left = -speed * 0.5
            right = speed
        elif action == 2:  # right turn
            left = speed
            right = -speed * 0.5
        elif action == 3:  # backward
            left = -speed
            right = -speed
        return left, right

    def receive_data_from_ros(self, msg):
        global unityState
        unityState = msg.data
        unityState = transfer_obs(unityState)
        action = refined_obstacle_avoidance_with_target_orientation(
            unityState['lidar_data'],
            unityState['car_quaternion'][2],
            unityState['car_quaternion'][3],
            unityState['car_pos'][:2],
            unityState['target_pos'][:2]
        )
        left, right = self.wheel_speed_transform(action)
        speed = [left, right]

        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_B_control),
            "data": dict(CarBControl(
                target_vel=speed
            ))
        }

        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()
        self.publisher_Ai2ros.publish(control_msg_forward)
        # Do 0.5 seconds after turn left or right, 0.75 seconds otherwise.
        if action == 1 or action == 2:
            time.sleep(0.5)
        else:
            time.sleep(0.75)

        # Pause the car
        speed = [0.0625, 0.0625]
        control_signal_forward = {
            "type": str(DeviceDataTypeEnum.car_B_control),
            "data": dict(CarBControl(
                target_vel=speed
            ))
        }
        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()
        self.publisher_Ai2ros.publish(control_msg_forward)
        time.sleep(5)
        # # Pause 5 seconds after turn left or right, 2 seconds otherwise.
        # if action == 1 or action == 2:
        #     time.sleep(5)
        # else:
        #     time.sleep(2)


def spin_pros(node):
    exe = rclpy.executors.SingleThreadedExecutor()
    exe.add_node(node)
    exe.spin()
    rclpy.shutdown()
    sys.exit(0)


def returnUnityState():
    while len(unityState) == 0:
        pass
    return unityState


class MLP(nn.Module):
    def __init__(self, input_size, hidden_size1, hidden_size2, output_size):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size1).double()
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size1, hidden_size2).double()
        self.relu2 = nn.ReLU()
        self.fc3 = nn.Linear(hidden_size2, output_size).double()

    def forward(self, x):
        x = x.float().double()
        x = self.fc1(x)
        x = self.relu1(x)
        x = self.fc2(x)
        x = self.relu2(x)
        x = self.fc3(x)
        return x


def main():
    rclpy.init()
    node = AiNode()
    pros = threading.Thread(target=spin_pros, args=(node,))
    pros.start()


if __name__ == '__main__':
    main()
