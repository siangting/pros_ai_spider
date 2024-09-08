from typing import Any
from utils.utils import trans_to_float


def preprocess_data(obs : dict[str, Any]) -> dict[str, Any]:
    """
    Preprocess the raw data subscribe from Unity.

    Parameters
    ----------
        obs: dict[str, Any]
            Raw data dictionary receive from subscriber.

    Returns
    ----------
        state_dict: dict[str, Any]
            The final data form in AI_spider_node. Ready to send to RL model.
    """

    # extract data part
    spider_center = obs["center_position"]
    spider_center: list[float] = trans_to_float(spider_center)

    spider_head = obs["head_position"]
    spider_head: list[float] = trans_to_float(spider_head)

    spider_cur_angle: list[float] = list(obs["joint_cur_angle"])
    
    state_dict = {

        "spider_center_x": spider_center[0], # float
        "spider_center_y": spider_center[1],
        "spider_center_z": spider_center[2],

        "spider_head_x": spider_head[0],
        "spider_head_y": spider_head[1],
        "spider_head_z": spider_head[2],
        
        "spider_joint_cur_angle": spider_cur_angle
    }

    return state_dict