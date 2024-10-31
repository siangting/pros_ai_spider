import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from utils import utils
from Spider_RL import reward_cal
from Spider_RL.PPOConfig import PPOConfig


class CustomSpiderEnv(gym.Env):
    ENV_NAME = "CustomSpiderEnv-v0"

    def __init__(self, AI_node):
        super(CustomSpiderEnv, self).__init__()
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
        self.AI_node.publish_jointtarget(action) 
        time.sleep(0.12)

        unity_data = utils.get_observation(self.AI_node)

        # TODO The current PPO model reads offset_angle as positive, but now it includes both positive and negative values.
        # For now, all values will be made positive, but after retraining the new model, this won't be necessary.

        self.state = utils.process_data_to_npfloat32_array(unity_data)

        reward = reward_cal.reward_cal_main(unity_data, self.step_counter)

        self.step_counter = self.step_counter + 1
        if (self.step_counter % 64 == 0):
            print("\nreward: " + str(round(reward)) + '\n')

        terminated = False
        if (abs(unity_data["offset_angle"]) >= PPOConfig.RESET_TOWARD_ANGLE_THRESHOLD):
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