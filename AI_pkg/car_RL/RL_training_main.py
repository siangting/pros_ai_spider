import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

# from avoidance import refined_obstacle_avoidance_with_target_orientation
from utils.obs_utils import process_data
from utils.RL_utils import get_observation
from ros_receive_and_data_processing.config import (
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
)


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"

    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        # state初始化
        self.AI_node = AI_node
        # self.start_time = time.time()
        shape_number = self.get_initial_shape()
        self.action_space = spaces.MultiDiscrete([21, 21])
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(shape_number,), dtype=np.float32
        )

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退

        # elapsed_time = time.time() - self.start_time  #  計時
        action_vel = [action[0], action[1], action[0], action[1]]
        self.AI_node.publish_to_robot(
            action_vel, pid_control=True
        )  #  送出後會等到unity做完動作後
        time.sleep(0.5)
        unity_data = get_observation(self.AI_node)
        reward = 1

        self.state = process_data(unity_data)
        combined_lidar_data = (
            [unity_data["lidar_data"][i] for i in FRONT_LIDAR_INDICES]
            + [unity_data["lidar_data"][i] for i in LEFT_LIDAR_INDICES]
            + [unity_data["lidar_data"][i] for i in RIGHT_LIDAR_INDICES]
        )
        terminated = (
            unity_data["car_target_distance"] < 1 or min(combined_lidar_data) < 0.2
        )
        return self.state, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        # self.AI_node.publish_to_unity_RESET()  #  送結束訊後給unity
        # self.AI_node.reset()
        self.AI_node.publisher_localiztion_map()
        unity_data_reset_state = get_observation(self.AI_node)
        self.state = process_data(unity_data_reset_state)

        # self.start_time = time.time()

        print("Reset Game")
        self.AI_node.reset()
        self.AI_node.reset_unity()
        time.sleep(3)
        self.AI_node.publisher_localiztion_map()
        return self.state, {}

    def get_initial_shape(self):
        # obs_state = self.AI_node.wait_for_data()
        obs_state = get_observation(self.AI_node)
        obs_state = process_data(obs_state)
        return len(obs_state)
