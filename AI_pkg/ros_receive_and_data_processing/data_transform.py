from utils.obs_utils import cal_distance, trans_to_float, round_to_decimal_places
from utils.rotate_angle import calculate_angle_point

"""
將AI_node.py收到的資料在這邊做轉換
這邊會決定main裡面的rule和navigation所收到的數值
"""


# obs 是 AI_node.py  real_car_data 的字典資料 這個 func 只會出現在 AI_node.py 內使用
def preprocess_data(obs):

    # 先擷取資料片段
    spider_center = obs["center_position"]


    # 以下宣告自己要回傳的資料
    state_dict = {
        "spider_center_x": trans_to_float(spider_center)[0],
        "spider_center_y": trans_to_float(spider_center)[1],
        "spider_center_z": trans_to_float(spider_center)[2]
    }

    return state_dict

"""
def normalize(value, min_value, max_value):
    return (value - min_value) / (max_value - min_value)
"""
