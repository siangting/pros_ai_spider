from Entity import State
from Entity import ROS2Point
from Entity import WheelAngularVel
from Utility import clamp
import json
from tools import * # test

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
    # min_value = 0.1
    # max_value = 1.0
    # filtered_list = [x for x in lidar_list if x != 100]
    # max_val = max(filtered_list)
    # min_val = min(filtered_list)

    # tar_pos = obs['ROS2TargetPosition'][0]
    # car_qua = obs['ROS2CarQuaternion']
    # car_pos = obs['ROS2CarPosition'][:2]
    # car_yaw = get_yaw_from_quaternion(car_qua[2], car_qua[3])
    # direction_vector = get_direction_vector(car_pos, tar_pos)
    # angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    # print(angle_to_target)
    
    # # iterate the whole list and then normalized it.
    # normalized_list = [(x - min_val) / (max_val - min_val) * (max_value - min_value) + min_value if x != 100 else x for x in lidar_list]
    
    # state = State(
    #     car_pos=ROS2Point(x=obs['ROS2CarPosition'][0],
    #                       y=obs['ROS2CarPosition'][1],
    #                       z=0.0),
    #     target_pos=ROS2Point(
    #         x=obs['ROS2TargetPosition'][0],
    #         y=obs['ROS2TargetPosition'][1],
    #         z=0.0
    #     ),
    #     car_direction=obs['ROS2CarQuaternion'],
    #     wheel_angular_vel_left_back=obs['ROS2WheelAngularVelocityLeftBack'],
    #     wheel_angular_vel_right_back=obs['ROS2WheelAngularVelocityRightBack'],
    #     wheel_angular_vel_left_front=obs['ROS2WheelAngularVelocityLeftFront'],
    #     wheel_angular_vel_right_front=obs['ROS2WheelAngularVelocityRightFront'],
    #     lidar=lidar_list,
    #     lidar_direction=obs['ROS2RangePosition'],
    #     target_vel=[obs['ROS2WheelAngularVelocityLeftBack'], obs['ROS2WheelAngularVelocityRightBack']]
    # )

    state_dict = {
        "car_pos": trans_to_float(obs['ROS2CarPosition']),
        "target_pos": trans_to_float(obs['ROS2TargetPosition']),
        # "car_target_distance": float(car_target_distance),
        "car_quaternion": trans_to_float(obs['ROS2CarQuaternion']),
        "lidar_data": trans_to_float(lidar_list[0]),
        # "relative_coordinates": trans_to_float([target_pos[0]-car_pos[0],target_pos[1]-car_pos[0]])
        # "flattened_directions": trans_to_float(flattened_directions), #  lidar direction
    }

    return state_dict


def jsonTransToDict(obs):
    obs = json.loads(obs)

    for key, value in obs.items():
        if isinstance(value, str) and value.startswith('(') and value.endswith(')'):
            coordinate_str = value.strip('()')
            coordinates = list(map(float, coordinate_str.split(',')))
            obs[key] = coordinates
    return obs
