from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
import random
from std_msgs.msg import Header
import json
from std_msgs.msg import String
import orjson
import math
import time
from rclpy.node import Node
from ros_receive_and_data_processing.car_models import *
from ros_receive_and_data_processing.data_transform import preprocess_data
from ros_receive_and_data_processing.config import (
    ACTION_MAPPINGS,
    LIDAR_PER_SECTOR,
    NEXT_POINT_DISTANCE,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data


class AI_node(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")
        self.real_car_data = {}
        self.data_to_AI = ""

        self.lastest_data = None
        """
        以下字典是用來定義這個node接收到最新的數值後傳送到UnityAdaptor.py內轉換
        最終轉換格式是UnityAdaptor.py決定 這邊只負責接收最新數值
        """
        self.real_car_data["ROS2CarPosition"] = []
        self.real_car_data["ROS2CarQuaternion"] = []
        self.real_car_data["ROS2TargetPosition"] = []

        #  讀取四輪電壓
        self.real_car_data["ROS2WheelAngularVelocityLeftBack"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityLeftFront"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityRightBack"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityRightFront"] = 0.01

        #  lidar的射線
        self.real_car_data["ROS2Range"] = []
        # lidar每條射線的vector
        self.real_car_data["ROS2RangePosition"] = []

        #  navigation資料
        self.real_car_data["cmd_vel_nav"] = Twist()
        self.real_car_data["received_global_plan"] = None

        """
        確定以下資料都有收到 才會在 check_and_get_lastest_data() 更新最新資料
        amcl 追蹤車體目前位置
        goal 目標位置
        lidar lidar資料
        """
        self.data_updated = {
            "amcl": False,
            "goal": False,
            "lidar": False,
        }

        """
        接收目標, 車體目前位置, 輪子轉速, lidar的距離和vector
        """
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscribe_callback_amcl, 1
        )
        """
        目標座標
        """
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscribe_callback_goal, 1
        )
        """
        lidar掃描
        """
        self.subscriber_lidar = self.create_subscription(
            LaserScan, "/scan", self.laser_scan_callback, 1
        )
        """
        車體後輪電壓
        """
        self.subscriber_rear = self.create_subscription(
            String, DeviceDataTypeEnum.car_C_state, self.rear_wheel_callback, 1
        )
        """
        車體前輪電壓
        """
        self.subscriber_forward = self.create_subscription(
            String, DeviceDataTypeEnum.car_C_state_front, self.forward_wheel_callback, 1
        )

        """
        publish給前後的esp32驅動車輪
        """
        self.publisher = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_rear_wheel, 10
        )  # 後輪esp32

        self.publisher_forward = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_front_wheel, 10
        )  # 前輪esp32

        self.publisher_unity_reset_signal = self.create_publisher(
            String, "/reset_signal", 10
        )
        """
        publish to localization initialpose
        """
        self.publisher_localization_map_signal = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        """
        機械手臂
        """
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

        """
        回傳經過路徑規劃時物體該用多少速度去移動
        return linear x y z & angular x y z
        """
        self.subscriber_navigation = self.create_subscription(
            Twist, "/cmd_vel_nav", self.navigation_callback, 1
        )
        self.last_message_time = time.time()
        self.no_signal = False

        """
        received_global_plan
        """
        self.subscriber_received_global_plan = self.create_subscription(
            Path, "/received_global_plan", self.global_plan_callback, 1
        )

        """
        Receive map data from foxglove map,
        publish map position in random,
        ready to receive data
        """
        self.subscription_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.publisher_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.map_data = None

    """
    檢查所有數據是否更新,
    更新最新車體狀態資料
    """

    def check_and_get_lastest_data(self):
        if all(self.data_updated.values()):
            # 確認所有的數據都更新並publish
            self.data_updated["amcl"] = False
            self.data_updated["lidar"] = False
            self.lastest_data = preprocess_data(self.real_car_data)

    def get_latest_data(self):
        return self.lastest_data

    """
    取得經過data_transform處理後的最新資料

    Returns:
        dict: raw data經過整理的資料

    """

    def wait_for_data(self):
        car_state_data = self.get_latest_data()
        while car_state_data is None:
            car_state_data = self.get_latest_data()
        return car_state_data

    def publish_control_signal(self, velocities: List[float], publisher):
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
            "data": dict(CarCControl(target_vel=velocities)),
        }
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()
        publisher.publish(control_msg)

    """
    Publish vehicle control signals to robot.

    Args :
    - action(str or [float]) : action code or velocity
    - pid_control : If True, action is treated as PID value
    """

    def publish_to_robot(self, action, pid_control: bool = True):
        if pid_control:
            _vel1, _vel2, _vel3, _vel4 = action
        else:
            mapped_action = ACTION_MAPPINGS.get(
                action, [0, 0, 0, 0]
            )  # Default to stop if action is invalid
            _vel1, _vel2, _vel3, _vel4 = mapped_action
        velocities_front = [_vel1, _vel2]  # 前面的esp32
        velocities_back = [_vel3, _vel4]  # 後面的esp32
        self.publish_control_signal(velocities_front, self.publisher)
        self.publish_control_signal(velocities_back, self.publisher_forward)

    def publish_arm(self, joint_pos):
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in joint_pos]
        msg.velocities = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)

    """
    給車子四個輪子停止的電壓
    """

    def publish_to_robot_reset(self):
        self.publish_to_robot("STOP", pid_control=False)

    """
    重製lastest_data
    """

    def reset(self):
        self.lastest_data = None
        # self.publish_to_robot("STOP", pid_control=False)

    """
    傳送給 unity 車子回去到原點的訊號
    """

    def reset_unity(self):
        msg = String()
        msg.data = "1"
        self.publisher_unity_reset_signal.publish(msg)

    """
    確定amcl goal lidar其中一個有收到訊號
    並更新目前是否有接收到的狀態
    """

    def update_and_check_data(self, data_type):
        self.data_updated[data_type] = True
        self.check_and_get_lastest_data()

    """
    amcl的subscribe function
    """

    def subscribe_callback_amcl(self, message):
        pose = self.transfer_car_pose(message)[0]
        quaternion = self.transfer_car_pose(message)[1]
        self.real_car_data["ROS2CarPosition"] = pose
        self.real_car_data["ROS2CarQuaternion"] = quaternion
        self.update_and_check_data("amcl")

    """
    給 subscribe_callback_amcl 做轉換使用
    """

    def transfer_car_pose(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return [position.x, position.y, position.z], [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]

    """
    goal 的 subscribe function
    """

    def subscribe_callback_goal(self, message):
        target = self.transfer_target_pos(message)
        if target:
            self.real_car_data["ROS2TargetPosition"] = target
            self.update_and_check_data("goal")

    """
    給 subscribe_callback_goal 轉換用
    """

    def transfer_target_pos(self, msg):
        position = msg.pose.position
        return [position.x, position.y, position.z]

    """
    lidar 的 subscriber function
    """

    def laser_scan_callback(self, msg):
        """
        普通lidar 預設輸出1800個數值
        高級lidar 預設輸出3240個數值
        可以透過print(len(msg.ranges))去觀察
        """
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges_180 = []
        direction_180 = []
        all_ranges = msg.ranges
        for i in range(len(all_ranges)):
            if i % LIDAR_PER_SECTOR == 0:  # handle the amount of lidar.
                angle_tmp = angle_min + i * angle_increment
                ranges_180.append(all_ranges[i])
                direction_180.append([math.cos(angle_tmp), math.sin(angle_tmp), 0])
        self.real_car_data["ROS2Range"] = ranges_180
        self.real_car_data["ROS2RangePosition"] = direction_180
        self.update_and_check_data("lidar")

    """
    navigation輸出路徑所需速度及轉角的subscribe function
    """

    def navigation_callback(self, msg):
        self.last_message_time = time.time()
        self.real_car_data["cmd_vel_nav"] = msg

    """
    navigation的global route planning
    會輸出座標點
    """

    def global_plan_callback(self, msg):
        try:
            if len(msg.poses) > 1:
                current_x, current_y = (
                    self.real_car_data["ROS2CarPosition"][0],
                    self.real_car_data["ROS2CarPosition"][1],
                )
                #  找出離目前車體 NEXT_POINT_DISTANCE 距離的座標點
                for i in range(20):
                    point_x, point_y = (
                        msg.poses[i].pose.position.x,
                        msg.poses[i].pose.position.y,
                    )
                    distance = math.sqrt(
                        (point_x - current_x) ** 2 + (point_y - current_y) ** 2
                    )
                    if abs(distance - NEXT_POINT_DISTANCE) < 0.01:
                        break
                self.real_car_data["received_global_plan"] = [point_x, point_y]
            else:
                self.get_logger().info("Global plan does not contain enough points.")
        except:
            self.real_car_data["received_global_plan"] = None

    """
    在navigation沒送資料時讓車子停止
    """

    def check_signal(self):
        timeout = 2.0
        current_time = time.time()
        if current_time - self.last_message_time > timeout:
            self.publish_to_robot_reset()
            return True

    """
    接收後輪的電壓 subscribe function
    """

    def rear_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if "data" in data and "vels" in data["data"]:
                vels = data["data"]["vels"]
                self.real_car_data["ROS2WheelAngularVelocityLeftBack"] = vels[0]
                self.real_car_data["ROS2WheelAngularVelocityRightBack"] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    """
    接收前輪的電壓 subscribe function
    """

    def forward_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if "data" in data and "vels" in data["data"]:
                vels = data["data"]["vels"]
                self.real_car_data["ROS2WheelAngularVelocityLeftFront"] = vels[0]
                self.real_car_data["ROS2WheelAngularVelocityRightFront"] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    """
    專門給 RL 做第一次localization的動作
    """

    def publisher_localization_map(self):
        self.publisher_clear_localization_map()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.29377966745215334
        msg.pose.pose.position.y = -0.327070701568549
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.045576476174623244
        msg.pose.pose.orientation.w = 0.9989608524959847
        msg.pose.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853892060437211,
        ]
        self.publisher_localization_map_signal.publish(msg)
        self.get_logger().info("Publish initial map position")

    """
    Reset map localization
    """

    def publisher_clear_localization_map(self):
        empty_msg = PoseWithCovarianceStamped()
        empty_msg.header.frame_id = "map"
        empty_msg.pose.pose.position.x = float("nan")
        self.publisher_localization_map_signal.publish(empty_msg)
        self.get_logger().info("Cleared initial map position")

    """
    Wait for map data
    """

    def map_callback(self, msg):
        self.get_logger().info("Received map data")
        self.map_data = msg

    def publisher_random_goal_pose(self):
        free_spaces = []
        if self.map_data is None:
            self.get_logger().warn("No map data available.")
            return
        for y in range(self.map_data.info.height):
            for x in range(self.map_data.info.width):
                index = x + y * self.map_data.info.width
                if self.map_data.data[index] == 0:  # 0 indicates no obstacle
                    free_spaces.append((x, y))

        if free_spaces:
            chosen_x, chosen_y = random.choice(free_spaces)
            self.get_logger().info(f"Chosen free space at: ({chosen_x}, {chosen_y})")

            # Convert map coordinates to world coordinates
            world_x = (
                chosen_x * self.map_data.info.resolution
                + self.map_data.info.origin.position.x
            )
            world_y = (
                chosen_y * self.map_data.info.resolution
                + self.map_data.info.origin.position.y
            )

            # Publish the random goal
            goal = PoseStamped()
            goal.header.frame_id = "map"
            # goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = world_x
            goal.pose.position.y = world_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0  # Default orientation

            self.publisher_goal_pose.publish(goal)
            self.get_logger().info(f"Published goal at: ({world_x}, {world_y})")
        else:
            self.get_logger().warn("No free spaces found in the map.")
