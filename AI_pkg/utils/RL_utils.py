"""
This python file aims to process AI_spider_node latest data.
Produce state dictionary ready to be flatten and become RL state.
"""


def get_observation(AI_spider_node) -> dict:
    """
    Get neweset Unity data and return processed data dict.

    Parameters
    ----------
        AI_spider_node: node
            Node that communicates with Unity.
    
    Returns
    ----------
        data_dict: dict
            Processed observation data, including spider vecz, vecx, centerz, centerx, 16 joints angles.
    """
    AI_spider_node.reset_latest_data()
    data_dict = add_spider_toward_key(AI_spider_node.wait_for_data())
    data_dict = remove_dict_key(data_dict)
    return data_dict


def remove_dict_key(data_dict: dict) -> dict:
    keys_to_remove = ["spider_center_y",
                      "spider_head_y",
                      "spider_head_x",
                      "spider_head_z"]
    for key in keys_to_remove:
        data_dict.pop(key, None)
    return data_dict

def add_spider_toward_key(data_dict: dict) -> dict:
    """
    spider_toward_vec = spider_head - spider_center 
    Add spider_toward in observation dictionary.
    Delete spider_head in observation dictionary.

    Parameters
    ----------
    data_dict: dict
        Spider state data from AI_spider_node.py lastest_data. 

    Returns
    ----------
    data_dict: dict
        State Observation next send to remove dict key. 
    """
    data_dict["spider_toward_vecx"] = data_dict["spider_head_x"] - data_dict["spider_center_x"]
    data_dict["spider_toward_vecz"] = data_dict["spider_head_z"] - data_dict["spider_center_z"]

    return data_dict

