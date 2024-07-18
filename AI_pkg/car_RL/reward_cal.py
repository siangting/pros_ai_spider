def reward_cal(data):
    reward = 0
    # if data["car_target_distance"] < 0.5:
    #     reward += 10
    # elif data["car_target_distance"] < 0.8:
    #     reward += 5
    # elif data["car_target_distance"] < 0.2:
    #     reward += 100
    if min(data["lidar_data"]) < 0.3:
        reward -= 50
    if min(data["lidar_data"]) > 0.3:
        reward += 10
    return reward
