from Spider_redirect_RL.redirect_PPOConfig import redirect_PPOConfig

def reward_cal_main(offset_angle: float, step_counter: int) -> float:
    """
    Calculates the reward based on the offset_angle and steps.
    We aim to make the spider_torward_vec and spider_target_vec as similar as possible, rewarding higher the closer their orientations match.
    The reward consists of a direction reward and a time penalty.

    Parameters
    ----------
        offset_angle: float
            The angular difference between the toward_vector and spider_target_vector.
        step_counter: int
            How many steps the training in this game round have done. Reset to zero when reset game.

    Returns
    ----------
        reward: float
            The calculated reward.
    """

    reward : float = 0.0
    angle_reward = -(offset_angle - redirect_PPOConfig.REWARD_CAL_ANGLE_BASELINE) * redirect_PPOConfig.ANGLE_REWARD_WEIGHT
    time_penalty: float = step_counter * redirect_PPOConfig.TIME_PENALTY_WEIGHT

    reward = angle_reward - time_penalty
    return reward
    
