def get_observation(AI_node):
    while AI_node.get_latest_data() == None:
        print(AI_node.get_latest_data())
        pass
    return AI_node.get_latest_data()
