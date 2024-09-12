from Spider_RL.PPOConfig import PPOConfig
from ros_receive_and_data_processing.SpiderConfig import SpiderConfig

def reward_cal_main(data: dict, step_counter: int) -> float:
    """
    Calculate reward for forward PPO training.
    Reward consists of "distance_reward", "time_penalty", and "angle_penalty" 

    Parameters
    ----------
        data: dict
            Observation dictionary.
        
        step_counter: int
            The steps number counter. When the training resets, the step_counter will reset to 0.

    Returns
    ----------
        reward: float
            The reward of forward PPO training.
    """
    current_spider_tree_dist: float = (data["spider_target_vecz"]) ** 2 + (data["spider_target_vecx"]) ** 2
       
    distance_reward: float = (SpiderConfig.SPIDER_TARGET_INIT_DIST - current_spider_tree_dist) * PPOConfig.DISTANCE_REWARD_WEIGHT
    time_penalty: float = step_counter * PPOConfig.TIME_PENALTY_WEIGHT
    angle_penalty: float = abs(data["offset_angle"]) * PPOConfig.ANGLE_PENALTY_WEIGHT

    reward: float = distance_reward - time_penalty - angle_penalty

    print("Forward distance " +  str(round(SpiderConfig.SPIDER_TARGET_INIT_DIST - current_spider_tree_dist)))
    
    return  reward