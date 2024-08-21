class SpiderConfig:
    """
    SpiderConfig class holds configuration parameters for the spider robot, 
    including action degrees and joint limits.
    
    Attributes
    ----------
    ACTION_DEGREE : dict
        A dictionary mapping action identifiers to the corresponding degrees of movement 
        for the spider robot's joints.
        
    JOINT_LIMIT : dict
        A dictionary specifying the maximum and minimum angle limits for the spider robot's joints.
    """

    ACTION_DEGREE: dict = {
        "0": -20.0,  # Action 0 represents degree movement
        "1": 0.0,    # Action 1 represents no movement (0 degree)
        "2": 20.0    # Action 2 represents degree movement
    }

    JOINT_LIMIT: dict = {
        "shoulder_abduction": 88,  # Maximum allowable abduction angle for the shoulder joints
        "shoulder_adduction": 18,  # Maximum allowable adduction angle for the shoulder joints
        "calf": 58           # Maximum allowable extension angle for the calf joints
    }