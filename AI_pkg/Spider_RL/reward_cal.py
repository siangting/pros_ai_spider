from queue import Queue

def reward_cal(data : dict, pre_z: Queue, queue_size: float) -> float:

    reward = 0
    x = data["spider_center_x"]
    z = data["spider_center_z"]

    temp_list = []
    for _ in range(queue_size):
        temp_z = pre_z.get()
        temp_list.append(temp_z)
        pre_z.put(temp_z)

    reward_offset_x = round(-5 * pow(10, 2) * abs(x - -1.5))
    reward_forward_z = 0

    for i in range(queue_size):
        reward_forward_z += (z - temp_list[i]) * (20 - i)
    
    reward_forward_z = round(reward_forward_z * 1.5 * pow(10, 3) / 210)

    reward = reward_offset_x + reward_forward_z

    return reward
