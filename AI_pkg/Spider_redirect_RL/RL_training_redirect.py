import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from utils import utils
from Spider_redirect_RL import redirect_reward_cal
# In RL_training_redirect, use PPOConfig, not use redirect_PPOConfig.
from Spider_RL.PPOConfig import PPOConfig


class CustomSpiderRedirectEnv(gym.Env):
    ENV_NAME = "CustomSpiderRedirectEnv-v0"

    def __init__(self, AI_node):
        super(CustomSpiderRedirectEnv, self).__init__()
        self.AI_node = AI_node
        self.step_counter : int = 0 # step_counter will reset to 0 again when reset game.

        # The flatten 1D array length of obervation dictionary
        self.shape_number = utils.get_initial_shape(self.AI_node)

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.shape_number,), dtype=np.float32
        )
        # 3 action options and 16 joints
        self.action_space = spaces.MultiDiscrete([4]) 



    def step(self, action):

        # call AI_spider_node.publish_to_robot
        self.AI_node.publish_jointtarget(action, is_redirect = True) 
        time.sleep(0.1)

        unity_data = utils.get_observation(self.AI_node)
        self.state = utils.process_data_to_npfloat32_array(unity_data)

        reward = redirect_reward_cal.reward_cal_main(unity_data, self.step_counter)

        self.step_counter = self.step_counter + 1
        if (self.step_counter % 64 == 0):
            print("\nreward: " + str(round(reward)) + '\n')

        terminated = False
        print("angle = " + str(unity_data["offset_angle"]))
        if (unity_data["offset_angle"] < PPOConfig.RESET_TOWARD_ANGLE_THRESHOLD):
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