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

DEG2RAD = 0.01745329251
unityState = ""


class AiNode(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")  # ros2Ai #unity2Ros
        self.subsvriber_ = self.create_subscription(String, "/real_car_data", self.receive_data_from_ros, 10)
        self.dataList = list()
        self.publisher_Ai2ros = self.create_publisher(String, DeviceDataTypeEnum.car_D_control, 10)  # Ai2ros #ros2Unity

        input_size = 182
        hidden_size1 = 128  # 根据需要调整
        hidden_size2 = 64
        output_size = 2

        self.loaded_model_1 = MLP(input_size, hidden_size1, hidden_size2, output_size)

        self.loaded_model_1.load_state_dict(
            torch.load("./dataFile/ver2.pth", map_location=torch.device('cpu')))  # model loading
        self.loaded_model_1.apply(self.init_weights)

    def init_weights(self, m):
        if type(m) == nn.Linear:
            nn.init.xavier_uniform_(m.weight)
            m.bias.data.fill_(0.01)

    def receive_data_from_ros(self, msg):
        global unityState
        unityState = msg.data

        unityState = transfer_obs(unityState)
        # unityState.lidar = [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 0.8339383, 0.8186077, 0.8048507, 0.7925648, 0.7816623, 0.772067, 0.7637132, 0.7565457, 0.7505185, 0.745592952, 0.7417379, 0.7389294, 0.737150848, 0.7363909, 0.7366452, 0.737915039, 0.7402084, 0.7435392, 0.7479278, 0.7534018, 0.7599954, 0.767751455, 0.776720643, 0.786962867, 0.7985488, 0.8115604, 0.8260931, 0.8422568, 0.8601785, 0.880005538, 0.9019089, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
        self.data = torch.tensor([
                                     unityState.car_pos.x,
                                     unityState.car_pos.y,
                                     # 0.0,
                                     # 0.0,
                                     # unityState.car_vel.x,
                                     # unityState.car_vel.y,
                                     # unityState.car_angular_vel,
                                     # unityState.wheel_angular_vel.left_back,
                                     # unityState.wheel_angular_vel.right_back,

                                 ] + unityState.lidar, dtype=torch.float32)

        def abs_and_clamp(value):
            # 取绝对值
            abs_value = abs(value)
            # 将值限制在1到5之间
            clamped_value = max(min(abs_value, 5), 1)
            return clamped_value

        self.loaded_model_1.eval()
        with torch.no_grad():
            output = self.loaded_model_1(self.data)
            print(output)
            action = list()
            action.append(abs_and_clamp(output[0]))
            action.append(abs_and_clamp(output[1]))
            action = [float(value) for value in action]

            control_signal_forward = {
                "type": str(DeviceDataTypeEnum.car_D_control),
                "data": dict(CarDControl(
                    target_vel=action
                ))
            }

            control_msg_forward = String()
            control_msg_forward.data = orjson.dumps(control_signal_forward).decode()
            self.publisher_Ai2ros.publish(control_msg_forward)


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
