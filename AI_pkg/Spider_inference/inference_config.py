class InferenceConfig:


    FORWARD_MODEL_PATH: str = "./Model/PPO_spider_2024-09-11.pt"
    REDIRECT_MODEL_PATH: str = "./redirect_Model/PPO_spider_2024-09-11.pt"

    FORWORD_SWITCH_TO_REDIRECT_ANGLE: float = 30.0
    REDIRECT_SWITCH_TO_FORWARD_ANGLE: float = 3.0

    TARGET_OBJECT_RADIUS: float = 3.0