import rclpy
import math
import numpy as np
import time

class RobotArmControl:
    def __init__(self, node):
        self.node = node
    def action(self):
        # [0,0,0,0,0]
        actions = [
            [30, 50, 100, 50, 10],
            [130, 60, 160, 50, 10],
            # Add more actions as needed
        ]
        for angle_set in actions:
            radians = list(np.radians(angle_set))
            self.node.publish_arm(radians)
            time.sleep(1)

def main(args=None):
    print("hello")

if __name__ == '__main__':
    main()