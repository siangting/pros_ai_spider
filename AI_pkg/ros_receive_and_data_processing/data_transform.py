from utils.obs_utils import trans_to_float


def preprocess_data(obs : dict) -> dict:
    """
    Preprocess the raw data.

    params: obs: Raw data dictionary receive from subscriber.
    return: state_dict: The final data form in AI_spider_node. Ready to send to RL model.

    """
    # extract data part
    spider_center = obs["center_position"]

    # declare data form what you want ot return
    state_dict = {

        "spider_center_x": trans_to_float(spider_center)[0],
        "spider_center_y": trans_to_float(spider_center)[1],
        "spider_center_z": trans_to_float(spider_center)[2]
    }

    return state_dict