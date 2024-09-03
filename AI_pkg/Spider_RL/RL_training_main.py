import numpy as np
import time
import gymnasium as gym
import queue
from gymnasium import spaces
from utils.utils import process_data_to_npfloat32_array
from utils.utils import get_observation
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

        unity_data = get_observation(self.AI_node)
        self.state = process_data_to_npfloat32_array(unity_data)

        reward = reward_cal.reward_cal_main(unity_data, self.pre_z, self.step_counter)

        self.step_counter = self.step_counter + 1
        if (self.step_counter % 64 == 0):
            print("\nreward: " + str(round(reward)) + '\n')

        self.pre_z.get()
        self.pre_z.put(unity_data["spider_center_z"])

        terminated = False
        if (self.angle_with_z_axis(unity_data) >= PPOConfig.RESET_TOWARD_ANGLE):
            terminated = True
        

        return self.state, reward, terminated, False, {}
        

    def reset(self, seed = None, options = None):

        print("Reset Game")
        self.step_counter = 0
        self.AI_node.reset_unity()
        time.sleep(1)

        unity_data_reset_state = get_observation(self.AI_node)
        self.state = process_data_to_npfloat32_array(unity_data_reset_state)
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
        obs_state = get_observation(self.AI_node)
        obs_state = process_data_to_npfloat32_array(obs_state)
        return len(obs_state)
    
    def angle_with_z_axis(self, data_dict: dict) -> float:
        """
        Calculate the angle of the torward vector in data_dict with (1.0, 0.0).
        Note that the vector is (z, x) as the spider in Unity moves forward along z axis.

        Parameter
        ----------
            data_dict: dict
                The observation dictionary from Unity.
        
        Return
        ----------
            angle_degree: float
                The angle between torward vector and (1.0, 0.0) in degree.
        """
        # Calculate the dot product of the vector with (1.0, 0.0)
        dot_product = data_dict["spider_toward_vecz"] * 1.0 + data_dict["spider_toward_vecx"] * 0.0
        
        # Calculate the magnitude (length) of the given vector
        vector_magnitude = math.sqrt(data_dict["spider_toward_vecz"]**2 + data_dict["spider_toward_vecx"]**2)
        
        # Calculate the cosine of the angle
        cos_angle = dot_product / vector_magnitude
        
        # Calculate the angle in radians using arccos (inverse cosine)
        angle_radians = math.acos(cos_angle)
        
        # Convert the angle from radians to degrees
        angle_degree = math.degrees(angle_radians)
        
        return angle_degree
