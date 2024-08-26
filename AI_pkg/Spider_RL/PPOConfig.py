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
        PRE_Z_QUEUE_SIZE (int): # The queue size to store init. Note that queue value must be 1 in reward target mode.
        
        REWARD_MODE (str):  "target mode" / "no target mode"

        X_MOTIPLY_PARAM (float) : The penalty multiply parameter of X offset.
        Z_MOTIPLY_PARAM (float) : The reward multiply parameter of forward z behavior.

        TARGET_X (float) : "target mode" parameters.
        TARGET_Z (float) : "target mode" parameters.

        DISTANCE_MULTIPLY_PARAM (float) : The multiply parameter of calculating distance reward.
        TIME_MULTIPLY_PARAM (float) : The multiply parameter of calculating time penalty.

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
    PRE_Z_QUEUE_SIZE: int = 1 # PRE_Z_QUEUE_SIZE must be 1 in reward target mode.

    # reward setting
    REWARD_MODE: str = "target mode"  # "target mode" / "no target mode"
    
    # no target reward mode setting
    X_MOTIPLY_PARAM: float = -5.0 * pow(10, 2) # The penalty multiply parameter of X offset.
    Z_MOTIPLY_PARAM: float = 1.5 * pow(10, 3) # The reward multiply parameter of forward z behavior.
    
    # target reward mode setting
    TARGET_X: float = 0.0 # "target mode" parameters.
    TARGET_Z: float = 70.0 # "target mode" parameters.

    DISTANCE_MULTIPLY_PARAM: float = 10.0 # The multiply parameter of calculating distance reward.
    TIME_MULTIPLY_PARAM: float = 1.0  # The multiply parameter of calculating time penalty.





