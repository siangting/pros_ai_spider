import numpy as np

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

