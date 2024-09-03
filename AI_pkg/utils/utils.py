"""
This python file aims to process AI_spider_node latest data.
Produce state dictionary ready to be flatten and become RL state.
"""
import numpy as np
import math
import sys


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

def process_data_to_npfloat32_array(unity_data: dict) -> dict:
    """
    Process dictionary to 1D array.

    Parameters
    ----------
    unity_data: dict
        Processed observation return by get_observation().
    
    Returns
    ----------
    flat_array: dict
        Final state of RL model input.

    Raises
    ----------
    type error
        Parameter dictionary unity_data values should be single number (float, int...) or list.  
    """
    flat_list = []
    for value in unity_data.values():
        if isinstance(value, list):
            flat_list.extend(value)
        else:
            flat_list.append(value)
    flat_array = np.array(flat_list, dtype=np.float32)
    return flat_array

def trans_to_float(data: any) -> list:
    return [float(i) for i in data]

def radians_degree_transfer(data: list[float] , mode: str) -> list[float]:
    """
    Transfer a list from degrees to radians or radians to degree.

    Parameters
    ----------
        data: list[float]
            The data want to be transfered.
        mode: str
            degree2radian or radian2degree.
    
    Returns
    ----------
        data: list[float]
            The data transfered.

    Raises
    ----------
        Value Error
            If the mode(str) is not "degree2radian" or "radian2degree" will cause an error.
    """
    
    if (mode == "degree2radian"):
        data = [math.radians(deg) for deg in data]
    elif (mode == "radian2degree"):
        data = [math.degrees(deg) for deg in data]
    else:
        print("Error: The mode should be degree2radian or radian2degree...")
        sys.exit()

    return data