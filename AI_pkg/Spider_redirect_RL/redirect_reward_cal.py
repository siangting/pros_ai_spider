from Spider_redirect_RL.redirect_PPOConfig import redirect_PPOConfig
from ros_receive_and_data_processing.SpiderConfig import SpiderConfig

def reward_cal_main(data: dict, step_counter: int) -> float:
    """
    Calculate reward for redirect PPO training.
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
            The reward of PPO training.
    """

    reward : float = 0.0
    current_spider_target_dist: float = (data["spider_target_vecz"]) ** 2 + (data["spider_target_vecx"]) ** 2
       
    distance_reward: float = (SpiderConfig.SPIDER_TARGET_INIT_DIST - current_spider_target_dist) * redirect_PPOConfig.DISTANCE_REWARD_WEIGHT
    angle_penalty: float = (abs(data["offset_angle"]) - redirect_PPOConfig.REWARD_CAL_ANGLE_BASELINE) * redirect_PPOConfig.ANGLE_PENALTY_WEIGHT
    time_penalty: float = step_counter * redirect_PPOConfig.TIME_PENALTY_WEIGHT

    reward = distance_reward - angle_penalty - time_penalty
    return reward
    
