from Spider_redirect_RL.redirect_PPOConfig import redirect_PPOConfig

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
    angle_reward = -(data["offset_angle"] - redirect_PPOConfig.REWARD_CAL_ANGLE_BASELINE) * redirect_PPOConfig.ANGLE_REWARD_WEIGHT
    time_penalty: float = step_counter * redirect_PPOConfig.TIME_PENALTY_WEIGHT

    reward = angle_reward - time_penalty
    return reward
    
