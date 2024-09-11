class SpiderConfig:
    """
    SpiderConfig class holds configuration parameters for the spider robot, 
    including action degrees, joint limits, initial pose and target position.
     
    Attributes
    ----------
    _forward_routine_angles: dict[str, float]
        A dictionary mapping spider locomotion routine angles.

        Note that the angles are human oriented in degrees.
     
    FORWARD__ACTION: dict[str, list[float]]
        A dictionary mapping forward action identifiers to the corresponding degrees set for the spider robot's joints.
        
    JOINT_LIMIT: dict[str, float]
        A dictionary specifying the maximum and minimum angle limits for the spider robot's joints.

    _Z_INIT_VALUE (float): The initial z-coordination of the spider center.
    _X_INIT_VALUE (float): The initial x-coordination of the spider center.

    TARGET_X (float) : The x-coordination of target in Unity.
    TARGET_Z (float) : The z-coordination of target in Unity.
    SPIDER_TARGET_INIT_DIST (float) : The init distance between spider and target.
        
    """

    _FORWARD_ROUTINE_ANGLES: dict[str, float] = {
        "shoulder_down": 0,
        "shoulder_flat": 15,
        "shoulder_up": 40.0,
        "calf_front": 35.0,
        "calf_back": -35.0
    }

    FORWARD_ACTION: dict[str, list[float]] = {
        "0": [
            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
        ],

        "1": [
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

        ],

        "2": [
            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

            _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
            _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

        ],

        "3": [
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
            _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
    ]  
    }

    JOINT_LIMIT: dict[str, float] = {
        "shoulder_abduction": 88.0,  # Maximum allowable abduction angle for the shoulder joints
        "shoulder_adduction": 18.0,  # Maximum allowable adduction angle for the shoulder joints
        "calf": 58.0           # Maximum allowable extension angle for the calf joints
    }


    _Z_INIT_VALUE: float = 0.0
    _X_INIT_VALUE: float = 0.0
    TARGET_X: float = 0.0
    TARGET_Z: float = 70.0
    SPIDER_TARGET_INIT_DIST: float = (TARGET_Z - _Z_INIT_VALUE) ** 2 + (TARGET_X - _X_INIT_VALUE) ** 2