from Spider_RL.redirect_PPOConfig import redirect_PPOConfig

def reward_cal_main(data : dict, step_counter: int) -> float:
    """
    Calculates the reward based on the given data and configuration.
    We expect the spider to face the positive z-axis, with a higher reward the closer it is oriented towards the positive z-axis.    
    The reward consists of a direction reward and a time penalty.

    Parameters
    ----------
        data: dict 
            Input data used for reward calculation.
        step_counter: int
            How many steps the training in this game round have done. Reset to zero when reset game.

    Returns
    ----------
        reward: float
            The calculated reward.
    """

    reward : float = 0.0
    x: float = data["spider_center_x"]
    z: float = data["spider_center_z"]

    return 0
    
