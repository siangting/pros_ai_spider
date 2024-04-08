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
            [170,50,80,50,10],
            [170,50,0,50,10],
            # Add more actions as needed
        ]
        self.node.publish_arm(list(np.radians(actions[0])))
        time.sleep(1)
        for i in range(3):
            for angle_set in actions:
                radians = list(np.radians(angle_set))
                self.node.publish_arm(radians)
                time.sleep(0.5)

def main(args=None):
    print("hello")

if __name__ == '__main__':
    main()