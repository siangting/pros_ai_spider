class SpiderConfig:
    """
    SpiderConfig class holds configuration parameters for the spider robot, 
    including action degrees and joint limits.
    
    Attributes
    ----------
    ACION : dict[str, list[float]]
        A dictionary mapping action identifiers to the corresponding degrees set for the spider robot's joints.
        
    JOINT_LIMIT : dict[str, float]
        A dictionary specifying the maximum and minimum angle limits for the spider robot's joints.
    """

    ACTION: dict[str, list[float]] = {
        "0": [
            90.0,  0.0,  90.0,  0.0, 
            -90.0,  0.0, -90.0,  0.0, 
            -90.0,  0.0, -90.0,  0.0,
            90.0,  0.0,  90.0,  0.0
        ],

        "1": [
            0.0,  0.0,  0.0,  0.0, 
            0.0,  0.0,  0.0,  0.0, 
            0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0
        ],

        "2": [
            0.0, 40.0,  0.0, 40.0, 
            0.0, 40.0,  0.0, 50.0, 
            0.0, 10.0,  0.0, 10.0,
            0.0, 10.0,  0.0, 10.0
        ]   
    }

    JOINT_LIMIT: dict[str, float] = {
        "shoulder_abduction": 88.0,  # Maximum allowable abduction angle for the shoulder joints
        "shoulder_adduction": 18.0,  # Maximum allowable adduction angle for the shoulder joints
        "calf": 58.0           # Maximum allowable extension angle for the calf joints
    }