from utils.navigation_utils import calculate_wheel_speeds, action_choice
from utils.rotate_angle import calculate_angle_to_target
import threading
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector
from ros_receive_and_data_processing.config import TARGET_DISTANCE


class NavigationProcess:
    def __init__(self, node):
        self.node = node
        self.robot_controler = RobotArmControl(
            node,
        )
        self.flag = 1
        # self.data_collector = DataCollector()  # 資料收集

    def robot_action_thread(self):
        self.robot_controler.action()

    """
    接收車子經處理過的state
    """

    def node_receive_data(self):
        self.node.reset()
        return self.node.wait_for_data()

    def process_car_data(self, car_data):
        received_global_plan = car_data["received_global_plan"]
        car_position = car_data["car_pos"][:2]
        pwm_left, pwm_right = calculate_wheel_speeds(car_data["cmd_vel_nav"])
        return received_global_plan, car_position, pwm_left, pwm_right

    # 到達終點時要做的事
    def handle_reached_destination(self):
        if self.flag == 1:
            print("end")
            action = "STOP"
            robot_thread = threading.Thread(target=self.robot_action_thread)
            robot_thread.start()
            self.node.publish_to_robot(action, pid_control=False)
            self.flag = 0

    # 未到達終點時做的事
    def handle_action(self, car_data):
        self.flag = 1
        # stop_signal == True時代表nav2的cmd_vel_nav的topic沒訊號，要改用其他控制方式
        stop_signal = self.node.check_signal()
        received_global_plan, car_position, pwm_left, pwm_right = self.process_car_data(
            car_data
        )
        # 用pid數值控制
        if stop_signal is not True:
            self.node.publish_to_robot(
                [pwm_left, pwm_right, pwm_left, pwm_right],
                pid_control=True,
            )
        # 用點對點到達方式
        else:
            angle_to_target = calculate_angle_to_target(
                car_position, received_global_plan, car_data["car_quaternion"]
            )
            action = action_choice(angle_to_target)
            self.node.publish_to_robot(action, pid_control=False)

    def run(self):
        car_data = self.node_receive_data()
        if car_data["car_target_distance"] < TARGET_DISTANCE:
            self.handle_reached_destination()
        else:
            self.handle_action(car_data)
