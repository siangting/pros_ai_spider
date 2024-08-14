from utils.obs_utils import trans_to_float

"""
處理 AI_spider_node 收到的 raw data 並傳回，傳收資料都是字典格式
"""

def preprocess_data(obs):

    # 擷取資料片段
    spider_center = obs["center_position"]

    # 宣告回傳資料
    state_dict = {

        # 將 list[x, y, z] 拆分成獨立 float 數值較直觀
        "spider_center_x": trans_to_float(spider_center)[0],
        "spider_center_y": trans_to_float(spider_center)[1],
        "spider_center_z": trans_to_float(spider_center)[2]
    }

    return state_dict