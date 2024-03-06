from avoidance_rule.Simulated_Annealing import ObstacleAvoidanceController
import rclpy
from utils.obs_utils import *
from csv_store_and_file.csv_store import save_data_to_csv, set_csv_format
import time


class RuleBasedController:
    def __init__(self, node):
        self.node = node
        self.data = []
        self.controller = ObstacleAvoidanceController()

    def rule_action(self, obs_for_avoidance):
        action = self.controller.refined_obstacle_avoidance_with_target_orientation(
            obs_for_avoidance["lidar_data"],
            obs_for_avoidance["car_quaternion"][0],
            obs_for_avoidance["car_quaternion"][1],
            obs_for_avoidance["car_pos"],
            obs_for_avoidance["target_pos"],
        )
        return action

    def reset_controller(self):
        self.data = []
        self.node.publish_to_unity_RESET()

    def store_data(self, unity_data):
        if self.save_to_csv:
            self.data.append(unity_data)

    def run(self):
        while rclpy.ok():
            self.node.reset()
            _, unity_data = wait_for_data(self.node)

            if unity_data["car_target_distance"] < 1:
                self.reset_controller()

            #  檢查是否撞到牆壁
            elif min(unity_data["lidar_data"]) < 0.2:
                self.reset_controller()
            else:
                #  這邊可以自訂一演算法回傳action
                action = self.rule_action(unity_data)
                self.node.publish_to_unity(action)
