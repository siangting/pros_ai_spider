import numpy as np
import math

def get_yaw_from_quaternion(z, w):
    '''四位數的z、w取得偏行角'''
    return np.degrees(2 * np.arctan2(z, w))
    
def get_direction_vector(current_position, target_position):
    '''計算目前車體位置指向目標的vector'''
    return np.array(target_position) - np.array(current_position)

def get_angle_to_target(car_yaw, direction_vector):
    '''計算car與target之間的角度差'''
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return (np.degrees(angle_diff)) % 360

def calculate_angle_point(car_quaternion_z, car_quaternion_w, car_pos, target_pos):
    '''回傳車頭面對目標的角度, 左轉是0~-180, 右轉是0~180, 越接近0代表車頭越正面於目標'''
    car_yaw = get_yaw_from_quaternion(car_quaternion_z, car_quaternion_w)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    angle_diff = angle_to_target
    if angle_diff > 180:
        angle_diff -= 360
    
    return angle_diff


def round_to_decimal_places(data_list, decimal_places=3):
    """
    Round the elements of a list to a specified number of decimal places.
    """
    return [round(num, decimal_places) for num in data_list]

def trans_to_float(data_list):
    return [float(i) for i in data_list]

def cal_distance(car_pos,target_pos):
    car_target_distance = (car_pos[0] - target_pos[0])**2 + (car_pos[1] - target_pos[1])**2
    car_target_distance = round_to_decimal_places([math.sqrt(car_target_distance)])[0]
    return car_target_distance