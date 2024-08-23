from queue import Queue
from Spider_RL.PPOConfig import PPOConfig
import os

def reward_cal(data : dict, pre_z: Queue, step_counter: int) -> float:
    """
    Calculates the reward based on the given data and configuration.
    Two modes to choose: "target mode", "no target mode". Choose in PPOConfig.py.

    Parameters
    ----------
        data: dict 
            Input data used for reward calculation.
        pre_z: Queue 
            A queue that stores previous z values.
        step_counter: int
            How many steps the training in this game round have done. Reset to zero when reset game.

    Returns
    ----------
        reward: float
            The calculated reward.
    
    Raises
    ----------
        PRE_Z_QUEUE_SIZE must be 1 in "target mode". Otherwise the terminal will raise error.
    """

    reward = 0
    x = data["spider_center_x"]
    z = data["spider_center_z"]

    if (PPOConfig.REWARD_MODE == "target mode" and PPOConfig.PRE_Z_QUEUE_SIZE != 1):
        print("Target mode reward now: PRE_Z_QUEUE_SIZE must be 1...\n")
        os._exit()

    if (PPOConfig.REWARD_MODE == "no target mode"):
        temp_list = []
        for _ in range(PPOConfig.PRE_Z_QUEUE_SIZE):
            temp_z = pre_z.get()  
            temp_list.append(temp_z)
            pre_z.put(temp_z)

        reward_offset_x = round(-5 * pow(10, 2) * abs(x - PPOConfig.X_INIT_VALUE))
        reward_forward_z = 0

        for i in range(PPOConfig.PRE_Z_QUEUE_SIZE):
            reward_forward_z += (z - temp_list[i]) * (PPOConfig.PRE_Z_QUEUE_SIZE - i)
        
        reward_forward_z = round(reward_forward_z * 1.5 * pow(10, 3) / ((1 + PPOConfig.PRE_Z_QUEUE_SIZE) * PPOConfig.PRE_Z_QUEUE_SIZE / 2))

        reward = reward_offset_x + reward_forward_z

    elif (PPOConfig.REWARD_MODE == "target mode"):
        reward = 0

    else :
        print("Error Reward Mode. Modify in Config.py\n")
        os._exit()

    return reward
