"""
This python file aims to process AI_spider_node latest data.
Produce state dictionary ready to be flatten and become RL state.
"""
import numpy as np
import math
import sys
from Spider_RL.PPOConfig import PPOConfig

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

            - spider_center_z: float
            - spider_toward_vecz: float
            - spider_toward_vecx: float
            - spider_target_vecz: float
            - spider_target_vecx: float
            - offset_angle: float
            - spider_joint_cur_angle: list[float]
    """
    AI_spider_node.reset_latest_data()
    data_dict = add_spider_toward_key(AI_spider_node.wait_for_data())
    data_dict = remove_dict_key(data_dict)
    return data_dict

def add_spider_toward_key(data_dict: dict) -> dict:
    """
    Add key in observation dictionary.

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

    data_dict["spider_target_vecx"] = PPOConfig.TARGET_X - data_dict["spider_center_x"]
    data_dict["spider_target_vecz"] = PPOConfig.TARGET_Z - data_dict["spider_center_z"]

    # offset_angle: The angular difference between the toward_vector and spider_target_vector.
    data_dict["offset_angle"] = two_vecs_to_angle(
        (data_dict["spider_toward_vecz"], data_dict["spider_toward_vecx"]),
        (data_dict["spider_target_vecz"], data_dict["spider_target_vecx"])
        )

    return data_dict



def remove_dict_key(data_dict: dict) -> dict:
    keys_to_remove = ["spider_center_x",
                      "spider_center_y",
                      
                      "spider_head_x",
                      "spider_head_y",
                      "spider_head_z"]
    for key in keys_to_remove:
        data_dict.pop(key, None)
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

def trans_to_float(data: any) -> list[float]:
    return [float(i) for i in data]


def get_initial_shape(AI_node) -> int:
    """
    Compute the 1D array length of self.state by AI_spider_node lastest_data.
    And return the array length to initialize the PPO obervation shape.

    Returns
    ----------
    int
        The length of the processed observation state array.
    """
    obs_state = get_observation(AI_node)
    obs_state = process_data_to_npfloat32_array(obs_state)
    return len(obs_state)

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

def two_vecs_to_angle(vec1: tuple[float, float], vec2: tuple[float, float]) -> float:
    """
    Calculate the angle of two vectors.

    Parameter
    ----------
        vec1: tuple[float, float]
            vector1
        vec2: tuple[float, float]
            vector2
    Return
    ----------
        angle_degree: float
            The angle between two vectors.
    """
    # Calculate the dot product of the two vectors.
    dot_product = vec1[0] * vec2[0] + vec1[1] * vec2[1]
    
    # Calculate the magnitude (length) of the given vectors.
    vec1_magnitude = math.sqrt(vec1[0]**2 + vec1[1]**2)
    vec2_magnitude = math.sqrt(vec2[0]**2 + vec2[1]**2)

    # Calculate the cosine of the angle.
    cos_angle = dot_product / (vec1_magnitude * vec2_magnitude)

    # Ensure the cosine value is between -1 and 1 to avoid math domain errors due to floating-point precision issues
    cos_angle = max(min(cos_angle, 1), -1)
    
    # Calculate the angle in radians using arccos (inverse cosine).
    angle_radians = math.acos(cos_angle)
    
    # Convert the angle from radians to degrees.
    angle_degree = math.degrees(angle_radians)
    
    return angle_degree


def convert_human_to_unity_spider_joint_angles(human_spider_angles: list[float]) -> list[float]:
    """
    Converts spider joint angles from human-friendly format to Unity's format.

    Parameters
    ----------
    human_spider_angles: list[float]
        A list of 16 angles that are intuitive for humans.
        
        Shoulder joints:
            - Abduction: positive
            - Adduction: negative
        
        Calf joints:
            - Forward extension: positive
            - Backward extension: negative
    
    Returns
    ----------
    unity_spider_angles: list[float]
        A list of 16 angles formatted for Unity.
        
        Facing the spider in Unity:
        - For the 4 front shoulder joints, positive angle values indicate counterclockwise rotation.
        - For the 4 back shoulder joints, positive angle values indicate clockwise rotation.
        - For the 4 left calf joints, positive angle values indicate backward extension.
        - For the 4 right calf joints, positive angle values indicate forward extension.
    
    Raises
    ----------
    ValueError
        If human_spider_angles does not consist of exactly 16 float values.
    """

    unity_spider_angles: list[float] = [
        human_spider_angles[0],
        -human_spider_angles[1],

        human_spider_angles[2],
        -human_spider_angles[3],

        -human_spider_angles[4],
        -human_spider_angles[5],

        -human_spider_angles[6],
        -human_spider_angles[7],

        -human_spider_angles[8],
        human_spider_angles[9],

        -human_spider_angles[10],
        human_spider_angles[11],

        human_spider_angles[12],
        human_spider_angles[13],

        human_spider_angles[14],
        human_spider_angles[15]
    ]

    return unity_spider_angles