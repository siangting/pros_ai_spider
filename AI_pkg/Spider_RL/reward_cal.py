from queue import Queue
from Spider_RL.PPOConfig import PPOConfig
import os

# SPIDER_TREE_INIT_DIST is used by TARGET_MODE.
SPIDER_TREE_INIT_DIST: float = (PPOConfig.TARGET_Z - PPOConfig.Z_INIT_VALUE) ** 2 + (PPOConfig.TARGET_X - PPOConfig.X_INIT_VALUE) ** 2

def reward_cal_main(data : dict, pre_z: Queue, step_counter: int, offset_angle: float) -> float:
    """
    Calculates the reward based on the given data and configuration.
    Two modes to choose: "TARGET_MODE", "NO_TARGET_MODE". Choose in PPOConfig.py.

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
        PRE_Z_QUEUE_SIZE must be 1 in "TARGET_MODE". Otherwise the terminal will raise error.
    """

    reward : float = 0.0
    x: float = data["spider_center_x"]
    z: float = data["spider_center_z"]
    
    if (PPOConfig.REWARD_MODE == "TARGET_MODE" and PPOConfig.PRE_Z_QUEUE_SIZE != 1):
        print("TARGET_MODE reward now: PRE_Z_QUEUE_SIZE must be 1...\n")
        os._exit()

    if (PPOConfig.REWARD_MODE == "NO_TARGET_MODE"):
        reward = no_target_reward(x, z, pre_z)

    elif (PPOConfig.REWARD_MODE == "TARGET_MODE"):
        reward = target_reward(x, z, step_counter, offset_angle)

    else :
        print("Error Reward Mode. Modify in Config.py\n")
        os._exit()

    return reward

def no_target_reward(x: float, z: float, pre_z: Queue) -> float:
    temp_list: list = []
    for _ in range(PPOConfig.PRE_Z_QUEUE_SIZE):
        temp_z = pre_z.get()  
        temp_list.append(temp_z)
        pre_z.put(temp_z)

    reward_offset_x: float = PPOConfig.X_MOTIPLY_PARAM * abs(x - PPOConfig.X_INIT_VALUE)
    reward_forward_z: float = 0.0

    for i in range(PPOConfig.PRE_Z_QUEUE_SIZE):
        reward_forward_z += (z - temp_list[i]) * (PPOConfig.PRE_Z_QUEUE_SIZE - i)  
    
    reward_forward_z = reward_forward_z * PPOConfig.Z_MOTIPLY_PARAM / ((1 + PPOConfig.PRE_Z_QUEUE_SIZE) * PPOConfig.PRE_Z_QUEUE_SIZE / 2)

    return reward_offset_x + reward_forward_z


def target_reward(x: float, z: float, step_counter: int, offset_angle: float) -> float:
    """
    Calculate reward for forward PPO training.
    target_reward consists of "distance_reward", "time_penalty", and "angle_penalty" 

    Parameters
    ----------
        x: float
            The latest received x-coordinate of spider center in Unity.
        z: float
            The latest received z-coordinate of spider center in Unity.
        step_counter: int
            The steps number counter. When the training resets, the step_counter will reset to 0.
        offset_angle: float
            The angular difference between the toward_vector and spider_target_vector.
    Returns
    ----------
        reward: float
            The reward of forward PPO training.
    """
    current_spider_tree_dist: float = (PPOConfig.TARGET_Z - z) ** 2 + (PPOConfig.TARGET_X - x) ** 2
    distance_reward: float = (SPIDER_TREE_INIT_DIST - current_spider_tree_dist) * PPOConfig.DISTANCE_REWARD_WEIGHT
    time_penalty: float = step_counter * PPOConfig.TIME_PENALTY_WEIGHT
    angle_penalty: float = offset_angle * PPOConfig.ANGLE_REWARD_WEIGHT

    print("Forward distance " +  str(SPIDER_TREE_INIT_DIST - current_spider_tree_dist))
    reward: float = distance_reward - time_penalty - angle_penalty
    return  reward