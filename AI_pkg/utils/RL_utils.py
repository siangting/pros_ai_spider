def get_observation(AI_node):
    AI_node.reset()
    data_dict = AI_node.wait_for_data()
    keys_to_remove = ["cmd_vel_nav", "received_global_plan"]
    for key in keys_to_remove:
        data_dict.pop(key, None)
    return data_dict
