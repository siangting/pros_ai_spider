import numpy as np
import time
import gymnasium as gym
import queue
from gymnasium import spaces
from utils import utils
from Spider_RL import reward_cal
from Spider_RL.PPOConfig import PPOConfig
import math


class CustomSpiderEnv(gym.Env):
    ENV_NAME = "CustomSpiderEnv-v0"

    def __init__(self, AI_node):
        super(CustomSpiderEnv, self).__init__()
        self.AI_node = AI_node
        self.pre_z = queue.Queue()
        self.queue_size = PPOConfig.PRE_Z_QUEUE_SIZE
   
        for _ in range(self.queue_size):
            self.pre_z.put(PPOConfig.Z_INIT_VALUE)
        self.step_counter : int = 0 # step_counter will reset to 0 again when reset game.

        # The flatten 1D array length of obervation dictionary
        self.shape_number = self.get_initial_shape()

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.shape_number,), dtype=np.float32
        )
        # 3 action options and 16 joints
        self.action_space = spaces.MultiDiscrete([3]) 



    def step(self, action):
        
        # call AI_spider_node.publish_to_robot
        self.AI_node.publish_jointtarget(action) 
        time.sleep(0.02)

        unity_data = utils.get_observation(self.AI_node)
        self.state = utils.process_data_to_npfloat32_array(unity_data)

        # toward_vector(z, x): spider_head - spider_center
        toward_vector: tuple = (unity_data["spider_toward_vecz"], unity_data["spider_toward_vecx"])
        # spider_target_vector(z, x): target - spider_center
        spider_target_vector: tuple = (PPOConfig.TARGET_Z - unity_data["spider_center_z"], PPOConfig.TARGET_X - unity_data["spider_center_x"])
        
        # offset_angle: The angular difference between the toward_vector and spider_target_vector.
        offset_angle: float = utils.two_vecs_to_angle(toward_vector, spider_target_vector)

        reward = reward_cal.reward_cal_main(unity_data, self.pre_z, self.step_counter, offset_angle)

        self.step_counter = self.step_counter + 1
        if (self.step_counter % 64 == 0):
            print("\nreward: " + str(round(reward)) + '\n')

        self.pre_z.get()
        self.pre_z.put(unity_data["spider_center_z"])

        terminated = False
        if (offset_angle >= PPOConfig.RESET_TOWARD_ANGLE):
            terminated = True
        

        return self.state, reward, terminated, False, {}
        

    def reset(self, seed = None, options = None):

        print("Reset Game")
        self.step_counter = 0
        self.AI_node.reset_unity()
        time.sleep(1)

        unity_data_reset_state = utils.get_observation(self.AI_node)
        self.state = utils.process_data_to_npfloat32_array(unity_data_reset_state)
        time.sleep(0.5)

        return self.state, {}

    def get_initial_shape(self) -> int:
        """
        Compute the 1D array length of self.state by AI_spider_node lastest_data.
        And return the array length to initialize the PPO obervation shape.

        Returns
        ----------
        int
            The length of the processed observation state array.
        """
        obs_state = utils.get_observation(self.AI_node)
        obs_state = utils.process_data_to_npfloat32_array(obs_state)
        return len(obs_state)