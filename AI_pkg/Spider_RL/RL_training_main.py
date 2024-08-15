import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces
from utils.obs_utils import process_data_to_npfloat32_array
from utils.RL_utils import get_observation
from Spider_RL.reward_cal import reward_cal


class CustomSpiderEnv(gym.Env):
    ENV_NAME = "CustomSpiderEnv-v0"

    def __init__(self, AI_node):
        super(CustomSpiderEnv, self).__init__()
        self.AI_node = AI_node

        # observation的攤平長度
        self.shape_number = self.get_initial_shape()

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.shape_number,), dtype=np.float32
        )
        # 3: 3個動作選項(正負零) / 16: 16個關節
        self.action_space = spaces.MultiDiscrete([3]*16) 



    def step(self, action):
        
        # call AI_spider_node.publish_to_robot
        self.AI_node.publish_jointposition(action) 
        time.sleep(0.1)
        unity_data = get_observation(self.AI_node)

        self.state = process_data_to_npfloat32_array(unity_data)


        # TODO reward fuction
        reward = 0 
        #reward = reward_cal(unity_data)

        # TODO terminate condition
        terminated = False

        return self.state, reward, terminated, False, {}
        # TODO return self.state, reward, terminated, False, {}
        

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

    def get_initial_shape(self) -> int:
        """data dictionary 攤平成 data array 後，傳回 data array 長度"""
        obs_state = get_observation(self.AI_node)
        obs_state = process_data_to_npfloat32_array(obs_state)
        return len(obs_state)
