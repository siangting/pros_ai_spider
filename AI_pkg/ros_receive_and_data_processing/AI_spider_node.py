import math
from rclpy.node import Node
from ros_receive_and_data_processing.data_transform import preprocess_data
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from ros_receive_and_data_processing.SpiderConfig import SpiderConfig
from utils import utils
import numpy as np


class AI_spider_node(Node):
    def __init__(self):
        super().__init__("AI_spider_node")
        self.get_logger().info("Ai spider start")

        # spider_data 儲存從 rosbridge 收到的 raw data
        self.spider_data = {}

        # 儲存從 ros_receive_and_data_processing.data_transform.preprocess_data 處理好的 data (字典)
        self.latest_data = None 

        # data_updated 為確保資料項目有收到的 flag，
        self.data_updated = {
            "center_position": False,
            "joint_cur_angle": False,
            "head_position": False
        }


        # Recieve spider center array(x, y, z) from unity SpiderCenterPublisher.cs
        self.spider_center_subscriber = self.create_subscription(
            Float32MultiArray,
            "/spider_center",
            self.spider_center_subscribe_callback,
            1
        )

        # Recieve spider head array(x, y, z) from unity SpiderHeadPublisher.cs 
        self.spider_head_subscriber = self.create_subscription(
            Float32MultiArray,
            "/spider_head",
            self.spider_head_subscribe_callback,
            1
        )

        # Recieve 16 current joints angles published by unity SpiderJointCurAnglePublisher.cs
        self.spider_joint_cur_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/spider_joint_cur_angle",
            self.spider_joint_cur_angle_subscribe_callback,
            1
        )

        # send 16 joints' angles to unity spiderROSBridgeSubscriber.cs
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spider_joint_trajectory_point',
            10
        )

        # send reset signal to SpiderSceneResetSubscriber.cs
        self.spider_scene_reset_publisher_ = self.create_publisher(
            Bool,
            'reset_unity',
            10
        )
        
    def spider_center_subscribe_callback(self, msg: Float32MultiArray) -> None:
        """
        Receive the raw spider center position msg from spider_center_subscriber.
        Store the data in to self.data_updated dictionary
        Change the data flag data_updated["center_position"].

        Parameters
        ----------
        msg: Float32MultiArray
            The raw spider center position message (x, y, z) receive from  spider_center_subscriber.
        """
        self.spider_data["center_position"] = msg.data     
        self.data_updated["center_position"] = True
        self.check_and_get_latest_data()

    def spider_head_subscribe_callback(self, msg: Float32MultiArray) -> None:
        """
        Receive the raw spider head position msg from spider_head_subscriber.
        Store the data in to self.data_updated dictionary
        Change the data flag data_updated["head_position"].

        Parameters
        ----------
        msg: Float32MultiArray
            The raw spider head position message (x, y, z) receive from  spider_head_subscriber.

        """
        self.spider_data["head_position"] = msg.data
        self.data_updated["head_position"] = True
        self.check_and_get_latest_data()        

    
    def spider_joint_cur_angle_subscribe_callback(self, msg: Float32MultiArray) -> None:
        """
        Receive the raw msg from spider_joint_cur_angle_subscriber.
        Store the data in to self.data_updated dictionary
        Change the data flag data_updated[""].

        :param msg: The raw message receive from  spider_center_subscriber.

        """
        self.spider_data["joint_cur_angle"] = msg.data
        self.data_updated["joint_cur_angle"] = True
        self.check_and_get_latest_data()


    def check_and_get_latest_data(self) -> None: 
        """
        Check if all data is recieved.
        refresh all data flag self.data_updated to false.
        """
        if all(self.data_updated.values()):
            for key in self.data_updated:
                self.data_updated[key] = False
            #latest_data is the final preprocessed data
            self.latest_data = preprocess_data(self.spider_data)  

    
    def wait_for_data(self) -> dict[str, float]:
        """
        Will be call by utils.get_observation.
        Before system call wait_for_data, self.latest_data will be clear.
        wait_for_data() will wait util self.latest_data exit.

        Return
        ----------
        spider_state : dict[str, float]
            The newest processed data from ROSBridge.
        
        Raise
        ----------
        Aware of the endless loop in the function.
        First 90000000 times waiting while loop will not log imformation.
        """
        spider_state = self.latest_data

        i = 0 # calculators        
        while spider_state is None:
            if (i % 90000000 == 0) and i != 0:
                print("\nwaiting for data ...")
            spider_state = self.latest_data
            i = i + 1
        
        return spider_state



    ## ---------- publish 16 joints target ----------

    def publish_jointtarget(self, action : np.ndarray[int]) -> None:
        """
        The publish event main function.
        Call functions to compute model action to actual joint angle position and publish.

        Parameters
        ----------
        actions: np.ndarray[int]
            RL model produce discrete actions.

        Raises
        ----------
        Be aware of degree and radians transformation.
        """
        msg = JointTrajectoryPoint()

        joint_target: list[float] = self.actions_to_joint_targets(action) # joint_target --> degrees
        joint_target = utils.convert_human_to_unity_spider_joint_angles(joint_target)
        joint_target = utils.radians_degree_transfer(joint_target, "degree2radian")
        msg.positions = self.limit_joint_targets(joint_target)
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  
        self.joint_trajectory_publisher_.publish(msg)

    def actions_to_joint_targets(self, actions: np.ndarray[int]) -> list[float]:
        """
        Transfer discrete action options to joint target (degrees).

        Parameters
        ----------
        actions: np.ndarray[int]
            RL model produce discrete actions.
        
        Returns
        -------
        joint_target: list[float]
            Trasfer from action option, the deserve target degrees of joints in human orientation.
        """
        for action in actions:
            if action == 0:
                joint_target: list[float] = SpiderConfig.FORWARD_ACTION["0"]
            elif action == 1:
                joint_target: list[float] = SpiderConfig.FORWARD_ACTION["1"]
            elif action == 2:
                joint_target: list[float] = SpiderConfig.FORWARD_ACTION["2"]
            elif action == 3:
                joint_target: list[float] = SpiderConfig.FORWARD_ACTION["3"]
            
        return joint_target
    
    def limit_joint_targets(self, joint_target: list[float]) -> list[float]:
        """
        Limit the joint targets' degree.

        Parameters
        ----------
        joint_target: list[float]
            Trasfer from RL model action in action_to_joint_target(). In Unity orientation.
        
        Returns
        ----------
        joint_target: list[float]
            The desire joint radians ready to send to unity.
        
        Raises
        ----------
        Be aware of degree and radians transformation.
        """

        for i in range(len(joint_target)): # limits are all less than Unity target limit for 2 degree for safety
            if (i % 2 == 0): # shoulder
                if i in [0, 2, 12, 14]:
                    joint_target[i] = max(min(joint_target[i], math.radians(SpiderConfig.JOINT_LIMIT["shoulder_abduction"])), math.radians(-SpiderConfig.JOINT_LIMIT["shoulder_adduction"]))
                else:
                    joint_target[i] = max(min(joint_target[i], math.radians(SpiderConfig.JOINT_LIMIT["shoulder_adduction"])), math.radians(-SpiderConfig.JOINT_LIMIT["shoulder_abduction"]))
            else: # calf
                joint_target[i] = max(min(joint_target[i], math.radians(SpiderConfig.JOINT_LIMIT["calf"])), math.radians(-SpiderConfig.JOINT_LIMIT["calf"]))

        return joint_target
    # ---------- publish 16 joints target -----------


    def reset_latest_data(self) -> None:
        self.latest_data = None

    def reset_unity(self) -> None:
        msg = Bool()
        msg.data = True # True: reset Unity/ False: Do not reset Unity
        self.spider_scene_reset_publisher_.publish(msg)


