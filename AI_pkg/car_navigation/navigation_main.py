import rclpy
# from utils.obs_utils import
from math import pi
from utils.rotate_angle import calculate_angle_point, calculate_angle_to_target
import time
from ros_receive_and_data_processing.config import FRONT_LIDAR_INDICES, LEFT_LIDAR_INDICES, RIGHT_LIDAR_INDICES
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector

import csv

class NavigationController:
    def __init__(self, node):
        self.node = node
        self.body_length = 0.5
        self.body_width = 0.3
        self.wheel_diameter = 0.05
        self.angle_tolerance = 10
        self.robot_controler = RobotArmControl(
            node,
        )
        self.data_collector = DataCollector()
        self.target_reached_once = False
        self.previous_target_pos = None

    def write_to_csv(self,filename, data):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)

    '''
    換算出兩輪pwm速度
    '''
    def calculate_wheel_speeds(self, twist_data):

        linear_velocity = twist_data.linear.x
        angular_velocity = twist_data.angular.z
        L = self.body_width
        v_left = linear_velocity - (L / 2) * angular_velocity
        v_right = linear_velocity + (L / 2) * angular_velocity


        rpm_left, rpm_right = self.speed_to_rpm(v_left), self.speed_to_rpm(v_right)
        pwm_left, pwm_right = self.rpm_to_pwm(rpm_left), self.rpm_to_pwm(rpm_right)
        return pwm_left, pwm_right

    def speed_to_rpm(self, speed):
        wheel_circumference = pi * self.wheel_diameter
        return (speed / wheel_circumference) * 60

    def rpm_to_pwm(self, rpm):
        max_rpm = 176
        pwm = (rpm / max_rpm) * 100
        return max(0, min(pwm, 100))

    def is_direction_clear(self, car_data, direction_indices, threshold):
        return all(car_data["lidar_data"][i] > threshold for i in direction_indices)

    '''
    sensitivity_value 越小容易轉動
    '''
    def calculate_sensitivity(self, car_data):
        sensitivity_value = 3.0

        left_clear = self.is_direction_clear(car_data, LEFT_LIDAR_INDICES, 0.3)
        right_clear = self.is_direction_clear(car_data, RIGHT_LIDAR_INDICES, 0.3)
        front_clear = self.is_direction_clear(car_data, FRONT_LIDAR_INDICES, 0.3)

        if not left_clear or not right_clear or not front_clear:
            sensitivity_value = 5.0  # 前方不清晰或部分清晰时的敏感度值
        left_clear = self.is_direction_clear(car_data, LEFT_LIDAR_INDICES, 0.5)
        right_clear = self.is_direction_clear(car_data, RIGHT_LIDAR_INDICES, 0.5)
        front_clear = self.is_direction_clear(car_data, FRONT_LIDAR_INDICES, 0.5)
        # if left_clear and right_clear and front_clear:
        #     # 前方额外清晰的检查
        #     front_extra_clear = self.is_direction_clear(car_data, FRONT_LIDAR_INDICES, 0.7)
        #     sensitivity_value = 30 if front_extra_clear else 20

        # print(sensitivity_value)
        return sensitivity_value

    '''
    如果前方小於15cm然後又沒訊號,就選擇後退
    如果單純沒訊號就停止運作
    如果有訊號就回傳 None
    '''
    def decide_action_based_on_signal(self, stop_signal, lidar_data):
        front_clear = all(lidar_data[i] > 0.15 for i in FRONT_LIDAR_INDICES)
        if stop_signal:
            return 6  # Custom stop action
        elif not stop_signal and not front_clear:
            return 3
        else:
            # return None  # No action decided based on signal
            return self.adjust_action_based_on_pwm(self.pwm_left, self.pwm_right, self.sensitivity_value)

    def adjust_action_based_on_pwm(self, pwm_left, pwm_right, sensitivity_value):
        # print(pwm_left, pwm_right)
        self.write_to_csv("data.csv", [pwm_left, pwm_right])
        sensitivity_value = 5
        if pwm_right < 0 and pwm_left < 0:
            return 3
        if abs(pwm_right - pwm_left) <= sensitivity_value:
            return 0  # Forward
        if pwm_right > pwm_left:
            return 2  # Turn right
        if pwm_right < pwm_left:
            return 4  # Turn left
        if pwm_right == 0 and pwm_left == 0:
            return 6  # Stop
        return 0  # Default forward action if none above

    def check_new_target(self, current_target_pos):
        if self.previous_target_pos is None:
            self.previous_target_pos = current_target_pos
            return True
        if self.previous_target_pos != current_target_pos:
            self.previous_target_pos = current_target_pos
            return True
        return False

    '''
    根據曲線選擇動作
    '''
    def action_choice(self, angle_to_target):
        if abs(angle_to_target)<20:
            action = 0
        elif 20 < abs(angle_to_target) < 40:
            if angle_to_target > 0:
                action = 1
            elif angle_to_target < 0:
                action = 5
        else:
            if angle_to_target > 0:
                action = 2
            elif angle_to_target < 0:
                action = 4
        return action

    def run(self):
        while rclpy.ok():
            self.node.reset()
            # 抓adaptor資料
            car_data = self.node.wait_for_data()
            # received_global_plan
            self.received_global_plan = car_data['received_global_plan']
            # car_posel
            self.car_pos = car_data['car_pos'][:2]
            #  取得兩輪pwm速度
            self.pwm_left, self.pwm_right = self.calculate_wheel_speeds(car_data["navigation_data"])
            self.sensitivity_value = self.calculate_sensitivity(car_data)

            #
            if self.check_new_target(car_data['target_pos']):
                print("new game")
                self.data_collector._create_directory()
                self.target_reached_once = False

            #  偵測navigation有無輸出訊號 true就代表沒訊號
            stop_signal = self.node.check_signal()

            # 到達終點
            if car_data["car_target_distance"] < 0.3:
                print("end")
                action = 6
                if not self.target_reached_once:
                    print("test")
                    # self.robot_controler.action()
                    self.data_collector.save_data_to_csv()
                    self.target_reached_once = True

            # 未到達終點的動作
            elif self.received_global_plan == None:
                action = 6
            else:
                action = self.decide_action_based_on_signal(stop_signal, car_data["lidar_data"])
                self.data_collector.add_data(action, car_data)
                angle_to_target = calculate_angle_to_target(self.car_pos,
                                        self.received_global_plan,
                                        car_data['car_quaternion'])
                action = self.action_choice(angle_to_target)
            self.node.publish_to_unity(action)