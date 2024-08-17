def reward_cal(data : dict, pre_z: float) -> float:
    reward = 0
    x = data["spider_center_x"]
    z = data["spider_center_z"]

    if z > pre_z:
        reward = -pow(10, 3) * abs(x - -1.5) + 9 * pow(10, 3) * abs(z - pre_z)
    else:
        reward = -pow(10, 3) * abs(x - -1.5) + -2 * pow(10, 3) * abs(z - pre_z)


    
    reward = float(reward)

    return reward
