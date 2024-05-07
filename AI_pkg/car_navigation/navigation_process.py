from utils.navigation_utils import calculate_wheel_speeds, action_choice
from utils.rotate_angle import calculate_angle_to_target
import threading
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector


def robot_action_thread(self):
    self.robot_controler.action()


class NavigationProcess:
    def __init__(self, node):
        self.node = node
        self.robot_controler = RobotArmControl(
            node,
        )
        self.data_collector = DataCollector()  # 資料收集
        self.target_reached_once = False
        self.previous_target_pos = None

    def node_receive_data(self):
        self.node.reset()
        return self.node.wait_for_data()

    def process_car_data(self, car_data):
        received_global_plan = car_data["received_global_plan"]
        car_position = car_data["car_pos"][:2]
        pwm_left, pwm_right = calculate_wheel_speeds(car_data["navigation_data"])
        return received_global_plan, car_position, pwm_left, pwm_right

    def handle_reached_destination(self):
        print("end")
        action = "STOP"
        self.node.publish_to_robot(action, pid_control=False)

    def handle_action(self, car_data):
        # stop_signal == True時代表nav2的cmd_vel_nav的topic沒訊號，要改用其他控制方式
        stop_signal = self.node.check_signal()
        received_global_plan, car_position, pwm_left, pwm_right = self.process_car_data(
            car_data
        )
        if received_global_plan is None:
            action = "STOP"
            self.node.publish_to_robot(action, pid_control=False)
        elif stop_signal is not True:
            self.node.publish_to_robot(
                [pwm_left, pwm_right, pwm_left, pwm_right],
                pid_control=True,
            )
        else:
            angle_to_target = calculate_angle_to_target(
                car_position, received_global_plan, car_data["car_quaternion"]
            )
            action = action_choice(angle_to_target)
            self.node.publish_to_robot(action, pid_control=False)

    def run(self):
        car_data = self.node_receive_data()
        if car_data["car_target_distance"] < 0.3:
            self.handle_reached_destination()
        else:
            self.handle_action(car_data)
