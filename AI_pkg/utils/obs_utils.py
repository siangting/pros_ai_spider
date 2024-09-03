import numpy as np
import math
import sys

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
