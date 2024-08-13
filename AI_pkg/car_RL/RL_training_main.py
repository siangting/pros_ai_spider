import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

# from avoidance import refined_obstacle_avoidance_with_target_orientation
from utils.obs_utils import process_data_to_npfloat32_array
from utils.RL_utils import get_observation
from ros_receive_and_data_processing.config import (
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
)
from car_RL.reward_cal import reward_cal


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"

    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        # state初始化
        self.AI_node = AI_node

        self.shape_number = self.get_initial_shape()

        """
        # ddpg
        self.action_space = spaces.Box(
            low=-10.0, high=10.0, shape=(shape_number,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(shape_number,), dtype=np.float32
        )
        """
        
        self.action_space = spaces.MultiDiscrete([3]*16)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.shape_number,), dtype=np.float32
        )

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退
        # elapsed_time = time.time() - self.start_time  #  計時
        self.AI_node.publish_to_robot(action)  #  送出後會等到unity做完動作後
        # print("action_vel : ", action_vel)
        # time.sleep(0.1)
        # unity_data = get_observation(self.AI_node)
        # reward = reward_cal(unity_data)  # reward
        # self.state = process_data_to_npfloat32_array(unity_data)
        # print("min lidar : ", min(unity_data["lidar_data"]))
        # terminated = min(unity_data["lidar_data"]) < 0.2
        return self.state, 0, False, False, {}

    def reset(self, seed=None, options=None):

        print("Reset Game")
        # self.AI_node.reset()

        # TODO
        # publish random /goal_pose
        # self.AI_node.reset_unity()
        # self.AI_node.publish_to_robot("STOP", pid_control=False)
        # self.AI_node.RL_mode_unity()
        """
        因為 lcoalization 的地圖用到一半會斷線, 而 unity 一直無法自動連線,
        因此該方法目前職暫時棄用,
        不然本來要自動做 localization 和自動 publish 一個隨機random位置
        """
        # self.AI_node.publisher_localization_map()
        # self.AI_node.reset_amcl()
        # self.AI_node.publisher_random_goal_pose()

        # 發送 unity 跟他說目前是 RL mode

        unity_data_reset_state = get_observation(self.AI_node)
        self.state = process_data_to_npfloat32_array(unity_data_reset_state)
        time.sleep(1)

        return self.state, {}

    def get_initial_shape(self):
        obs_state = get_observation(self.AI_node)
        obs_state = process_data_to_npfloat32_array(obs_state)
        return len(obs_state)
