from Entity import State
from Entity import ROS2Point
from Entity import WheelAngularVel
from Utility import clamp
import json


def transfer_obs(obs):
    obs = jsonTransToDict(obs)
    lidar_list = [round(100 if x == float('inf') else x, 2) for x in obs['ROS2Range']]
    # min_value = 0.1
    # max_value = 1.0
    # filtered_list = [x for x in lidar_list if x != 100]
    # max_val = max(filtered_list)
    # min_val = min(filtered_list)

    # # iterate the whole list and then normalized it.
    # normalized_list = [(x - min_val) / (max_val - min_val) * (max_value - min_value) + min_value if x != 100 else x for x in lidar_list]

    state = State(
        car_pos=ROS2Point(x=obs['ROS2CarPosition'][0],
                          y=obs['ROS2CarPosition'][1],
                          z=0.0),
        car_direction=obs['ROS2CarQuaternion'],
        wheel_angular_vel_left_back=obs['ROS2WheelAngularVelocityLeftBack'],
        wheel_angular_vel_right_back=obs['ROS2WheelAngularVelocityRightBack'],
        wheel_angular_vel_left_front=obs['ROS2WheelAngularVelocityLeftFront'],
        wheel_angular_vel_right_front=obs['ROS2WheelAngularVelocityRightFront'],
        lidar=lidar_list,
        lidar_direction=obs['ROS2RangePosition'],
        target_vel=[obs['ROS2WheelAngularVelocityLeftBack'], obs['ROS2WheelAngularVelocityRightBack']]
    )

    return state


def jsonTransToDict(obs):
    obs = json.loads(obs)

    for key, value in obs.items():
        if isinstance(value, str) and value.startswith('(') and value.endswith(')'):
            coordinate_str = value.strip('()')
            coordinates = list(map(float, coordinate_str.split(',')))
            obs[key] = coordinates
    return obs
