class PPOConfig:
    """
    The PPOConfig class encapsulates all configuration parameters for training a PPO model
    for a spider robot. 
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

        Z_INIT_VALUE (float): The initial value for the pre-z parameter in the environment.
        Z_QUEUE_SIZE (int): The size of the queue for storing pre-z values. Note that the queue is later used to calculate model reward.
    
    Note: 
        n_updates = total_timesteps // (n_steps * n_envs)
    
    """
    
    # Create PPO model
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 256
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10

    # Load PPO model
    MODEL_PATH: str = "./Model/PPO_spider_2024-08-21.pt"
    DEFAULT_MODLE_NAME: str = "./Model/PPO_spider"

    # training PPO model
    SAVE_MODEL_FREQUENCE: int = 1024
    TOTAL_TIME_STEPS: int = 1024 * 2

    # Env setting
    X_INIT_VALUE: float = 0.0
    Z_INIT_VALUE: float = 0.0
    PRE_Z_QUEUE_SIZE: int = 20





