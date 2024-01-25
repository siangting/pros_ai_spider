from Utility import clamp
import json
from tools import * # test
import config

def get_smallest_lidar_values_with_direction(lidar_data):
    """
    Divide the lidar data into 15 chunks and return the smallest value from each chunk
    along with its direction.
    """
    chunk_size = len(lidar_data) // 8
    result = []
    min_value_list = []
    min_direction_list = []

    for i in range(0, len(lidar_data), chunk_size):
        chunk = lidar_data[i:i + chunk_size]
        min_value = min(chunk)        
        min_value_list.append(min_value) 

    return min_value_list,

def trans_to_float(data_list):
    return [float(i) for i in data_list]

def transfer_obs(obs):
    obs = jsonTransToDict(obs)
    lidar_list = [round(100 if x == float('inf') else x, 2) for x in obs['ROS2Range']]
    lidar_list = list(get_smallest_lidar_values_with_direction(lidar_list))

    state_dict = config.state_dict
    return state_dict


def jsonTransToDict(obs):
    obs = json.loads(obs)

    for key, value in obs.items():
        if isinstance(value, str) and value.startswith('(') and value.endswith(')'):
            coordinate_str = value.strip('()')
            coordinates = list(map(float, coordinate_str.split(',')))
            obs[key] = coordinates
    return obs
