from queue import Queue

def reward_cal(data : dict, pre_z: Queue, queue_size: float) -> float:
    """
    Calculates the reward based on the given data and configuration.

    Parameters
    ----------
        data: dict 
            Input data used for reward calculation.
        pre_z: Queue 
            A queue that stores previous z values.
        queue_size: float
            The size of the queue for pre_z.

    Returns
    ----------
        reward: float
            The calculated reward.
    """

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
        reward_forward_z += (z - temp_list[i]) * (queue_size - i)
    
    reward_forward_z = round(reward_forward_z * 1.5 * pow(10, 3) / ((1 + queue_size) * queue_size / 2))

    reward = reward_offset_x + reward_forward_z

    return reward
