import numpy as np
import time
import gymnasium as gym
import queue
from gymnasium import spaces
from utils.obs_utils import process_data_to_npfloat32_array
from utils.RL_utils import get_observation
from Spider_RL import reward_cal
from Spider_RL.PPOConfig import PPOConfig


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
        self.action_space = spaces.MultiDiscrete([3]*16) 



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
        if (self.step_counter % PPOConfig.RESET_SCENE_STEP == 0):
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

    def choose_PPO_model(node) -> str:
        """
        This function will decide which PPO model to use later base on unity observation.
        if the angle of spider_vec and front_vec < PPOConfig.ANGLE_THRESHOLD, used "Forward PPO model". Otherwise used "Redirect PPO model". 

        Parameters
        ----------
            node: node
                The node initialized in init_ros_node() function.
        Returns
        ----------
            PPO_choice: str
                Which PPO model will be used later. 
        """
        print("123\n")
        unity_data: dict = get_observation(node)
        spider_vec: np.ndarray[float] = np.array([unity_data["spider_toward_vecz"], unity_data["spider_toward_vecx"]]) # (z, x)

        # The dot_product of spider_vec and front_vec will always be unity_data["spider_toward_vecz"] because front_vec is (1, 0).
        dot_product: float = unity_data["spider_toward_vecz"]  

        norm_spider_vec: float = np.linalg.norm(spider_vec)
        # norm_front_vec will always be 1 because front_vec is (1, 0).
        norm_front_vec: float = 1.0

        cos_theta: float = dot_product / (norm_spider_vec * norm_front_vec)
        theta_degrees: float = np.degrees(np.arccos(cos_theta))

        print("degree: "+ str(theta_degrees))
        return "None"
