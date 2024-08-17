from utils.obs_utils import trans_to_float


def preprocess_data(obs : dict) -> dict:
    """
    Preprocess the raw data.

    params: obs: Raw data dictionary receive from subscriber.
    return: state_dict: The final data form in AI_spider_node. Ready to send to RL model.

    """
    # extract data part
    spider_center = obs["center_position"]
    spider_center = trans_to_float(spider_center)

    spider_head = obs["head_position"]
    spider_head = trans_to_float(spider_head)

    spider_cur_angle = obs["joint_cur_angle"] # array[float]
    # declare data form what you want ot return
    state_dict = {

        "spider_center_x": spider_center[0],
        "spider_center_y": spider_center[1],
        "spider_center_z": spider_center[2], # float

        "spider_head_x": spider_head[0],
        "spider_head_y": spider_head[1],
        "spider_head_z": spider_head[2], # float
        
        "spider_joint_cur_angle": list(spider_cur_angle) 
    }

    return state_dict