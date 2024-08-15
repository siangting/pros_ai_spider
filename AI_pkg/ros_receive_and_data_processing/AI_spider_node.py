import math
from rclpy.node import Node
from ros_receive_and_data_processing.data_transform import preprocess_data
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray


class AI_spider_node(Node):
    def __init__(self):
        super().__init__("AI_spider_node")
        self.get_logger().info("Ai spider start")
        
        
        # spider_data 儲存從 rosbridge 收到的 raw data
        self.spider_data = {}

        # 儲存從 ros_receive_and_data_processing.data_transform.preprocess_data 處理好的 data (字典)
        self.lastest_data = None 

        # data_updated 為確保資料項目有收到的 flag，
        self.data_updated = {
            "center_position": False,
            "joint_cur_angle": False
        }


        # 收 unity SpiderCenterPublisher.cs 傳的 spider 中點座標 (x, y, z)
        self.spider_center_subscriber = self.create_subscription(
            Float32MultiArray,
            "/spider_center",
            self.spider_center_subscribe_callback,
            10
        )

        # Recieve 16 current joints angles published by unity SpiderJointCurAnglePublisher.cs
        self.spider_joint_cur_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/spider_joint_cur_angle",
            self.spider_joint_cur_angle_subscribe_callback,
            10
        )

        # 發 spider 的16個關節角度給 unity spiderROSBridgeSubscriber.cs
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spider_joint_trajectory_point',
            10
        )
        
    def spider_center_subscribe_callback(self, msg: Float32MultiArray) -> None:
        """
        Receive the raw msg from spider_center_subscriber.
        Store the data in to self.data_updated dictionary
        Change the data flag data_updated["center_position"].

        :param msg: The raw message receive from  spider_center_subscriber.

        """
        self.spider_data["center_position"] = msg.data
        self.data_updated["center_position"] = True
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
            #lastest_data is the final preprocessed data
            self.lastest_data = preprocess_data(self.spider_data)

    
    def wait_for_data(self) -> dict[str, float]:
        """call by RL_utils.get_observation"""
        spider_state = self.lastest_data

        i = 0 # calculators        
        while spider_state is None:
            if not (i % 90000000):
                print("\nwaiting for data ...")
                print("Info: AI_spider_node")
            spider_state = self.lastest_data
            i = i + 1

        return spider_state



    ## ---------- publish 16 joints position ----------

    def publish_jointposition(self, action : any) -> None:
        msg = JointTrajectoryPoint()

        joint_pose = self.preprocess_jointposition_publishData(action)
        msg.positions = [float(pose) for pose in joint_pose]

        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  
        self.joint_trajectory_publisher_.publish(msg)

    def preprocess_jointposition_publishData(self, actions : any) -> list:
        joint_pose = []
        for action in actions:
            if action == 0:
                joint_pose.append(math.radians(-30))
            elif action == 1:
                joint_pose.append(math.radians(0))
            elif action == 2:
                joint_pose.append(math.radians(30))
        return joint_pose
        
    # ---------- publish 16 joints position -----------


    def reset(self) -> None:
        self.lastest_data = None


