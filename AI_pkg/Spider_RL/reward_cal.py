from queue import Queue
from Spider_RL.PPOConfig import PPOConfig

def reward_cal(data : dict, pre_z: Queue) -> float:
    """
    Calculates the reward based on the given data and configuration.

    Parameters
    ----------
        data: dict 
            Input data used for reward calculation.
        pre_z: Queue 
            A queue that stores previous z values.

    Returns
    ----------
        reward: float
            The calculated reward.
    """

    reward = 0
    x = data["spider_center_x"]
    z = data["spider_center_z"]

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

    return reward
