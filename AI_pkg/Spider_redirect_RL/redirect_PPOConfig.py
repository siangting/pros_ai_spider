class redirect_PPOConfig:
    """
    The redirect_PPOConfig class encapsulates all configuration parameters for training a redirect PPO model
    for the spider robot. 
    This includes settings related to model creation, loading, training, and environment configuration.

    Attributes:
        LEARNING_RATE (float): The learning rate for the PPO model.
        N_STEPS (int): The number of steps to run in each environment per update.
        BATCH_SIZE (int): The mini-batch size for each gradient update.
        N_EPOCHS (int): The number of epochs for optimizing the PPO model.

        MODEL_PATH (str): The file path for saving and loading the PPO model.
        DEFAULT_MODLE_NAME (str): The default model name used when saving the PPO model.

        SAVE_MODEL_FREQUENCE (int): The frequency (in timesteps) at which the model is saved during training.
        TOTAL_TIME_STEPS (int): The total number of timesteps for training the PPO model. 
        RESET_SCENE_STEP (int): The step numbers PPO training terminate per time and unity scene reset.

        REWARD_CAL_ANGLE_BASELINE (float): The positive and negative threshold for calculating angle reward. When the offset angle is smaller than baseline, angle_reward will be positive and vice versa.
        ANGLE_REWARD_WEIGHT (float): The weight of angle reward.
        TIME_PENALTY_WEIGHT (float): The weight of time penalty.

    Note: 
        n_updates = total_timesteps // (n_steps * n_envs)
    
    """
    
    # Create PPO model
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10

    # Load PPO model
    MODEL_PATH: str = "./redirect_Model/redirect_PPO_spider_2024-09-05.pt"
    DEFAULT_MODLE_NAME: str = "./redirect_Model/redirect_PPO_spider"

    # training PPO model
    SAVE_MODEL_FREQUENCE: int = 1024 * 16
    TOTAL_TIME_STEPS: int = 1024 * 128 * 256
    RESET_SCENE_STEP: int = 1024 * 8

    # reward parameters
    REWARD_CAL_ANGLE_BASELINE: float = 100.0 # in degrees
    ANGLE_REWARD_WEIGHT: float = 5.0
    TIME_PENALTY_WEIGHT: float = 0.5






