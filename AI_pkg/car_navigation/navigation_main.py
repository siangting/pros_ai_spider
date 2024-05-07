import rclpy
from math import pi
from utils.rotate_angle import calculate_angle_point, calculate_angle_to_target
import csv
from car_navigation.navigation_process import NavigationProcess


class NavigationController:
    def __init__(self, node):
        self.target_reached_once = False
        self.previous_target_pos = None
        self.NavigationProcess = NavigationProcess(node=node)

    def write_to_csv(self, filename, data):
        with open(filename, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)

    def run(self):
        while rclpy.ok():
            self.NavigationProcess.run()
