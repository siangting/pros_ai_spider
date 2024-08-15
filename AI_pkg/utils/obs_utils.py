import numpy as np

def process_data_to_npfloat32_array(unity_data):
    """把 unity_data 字典攤平為 array"""
    flat_list = []
    for value in unity_data.values():
        flat_list.append(value)
    return np.array(flat_list, dtype=np.float32)

def trans_to_float(data: any) -> list:
    return [float(i) for i in data]

