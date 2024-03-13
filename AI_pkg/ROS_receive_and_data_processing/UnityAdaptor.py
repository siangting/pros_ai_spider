from utils.adaptor_utils import *
from utils.rotate_angle import *

"""
將AI_node.py收到的資料在這邊做轉換
這邊會決定main裡面的rule和navigation所收到的數值
"""


def transfer_obs(obs):
    car_pos = obs["ROS2CarPosition"][:2]
    target_pos = obs["ROS2TargetPosition"][:2]
    car_quaternion = obs["ROS2CarQuaternion"][2:4]
    lidar_data = obs["ROS2Range"]
    left_front_speed = obs["ROS2WheelAngularVelocityLeftFront"]
    right_front_speed = obs["ROS2WheelAngularVelocityRightFront"]
    left_back_speed = obs["ROS2WheelAngularVelocityLeftBack"]
    right_back_speed = obs["ROS2WheelAngularVelocityRightBack"]

    angle_diff = calculate_angle_point(  #  輸出面向目標角度
        car_quaternion[0], car_quaternion[1], car_pos, target_pos
    )
    car_target_distance = cal_distance(car_pos, target_pos)
    navigation_data = obs["twist_msg"]
    navigation_plan_data = obs["path_msg"]
    navigation_plan_position_data = obs["path_position_msg"]
    navigation_plan_orientation_data = obs["path_orientation_msg"]

    # 以下宣告自己要回傳的資料

    state_dict = {
        "car_pos": trans_to_float(car_pos),
        "target_pos": trans_to_float(target_pos),
        "car_target_distance": float(car_target_distance),
        "car_quaternion": trans_to_float(car_quaternion),
        "lidar_data": trans_to_float(lidar_data),
        "relative_coordinates": trans_to_float(
            round_to_decimal_places(
                [target_pos[0] - car_pos[0], target_pos[1] - car_pos[1]]
            )
        ),
        "angle_diff": angle_diff,
        "navigation_data": navigation_data,
        "navigation_plan_data": navigation_plan_data,
        "navigation_plan_position_data": navigation_plan_position_data,
        "navigation_plan_orientation_data": navigation_plan_orientation_data,
    }

    return state_dict
