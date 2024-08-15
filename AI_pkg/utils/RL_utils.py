"""
會先刪除以前的 data 後將不需要的 key 移除, 只留下自己想給 RL 看的
"""


def get_observation(AI_spider_node) -> dict:
    AI_spider_node.reset()
    data_dict = remove_dict_key(AI_spider_node.wait_for_data())
    return data_dict


def remove_dict_key(data_dict: dict) -> dict:
    keys_to_remove = []
    for key in keys_to_remove:
        data_dict.pop(key, None)  # 使用 pop 方法，避免 KeyError
    return data_dict

