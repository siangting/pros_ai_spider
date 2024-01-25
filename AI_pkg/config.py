state_dict = {
    "car_pos": trans_to_float(obs['ROS2CarPosition']),
    "target_pos": trans_to_float(obs['ROS2TargetPosition']),
    # "car_target_distance": float(car_target_distance),
    "car_quaternion": trans_to_float(obs['ROS2CarQuaternion']),
    "lidar_data": trans_to_float(lidar_list[0]),
    # "relative_coordinates": trans_to_float([target_pos[0]-car_pos[0],target_pos[1]-car_pos[0]])
    # "flattened_directions": trans_to_float(flattened_directions), #  lidar direction
}
