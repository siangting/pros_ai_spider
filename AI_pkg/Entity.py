from pydantic import BaseModel


class ROS2Point(BaseModel):
    x: float
    y: float
    z: float


# class lidar_direciton(BaseModel):
#     list(float, float, float)

# around ROS2 z axis, left +, right -, up 0, down 180
class WheelOrientation(BaseModel):
    left_front: float = 0
    right_front: float = 0


# around car wheel axis, front: +, back: -, r/s
class WheelAngularVel(BaseModel):
    left_back: float
    right_back: float


class State(BaseModel):
    car_pos: ROS2Point
    target_pos: ROS2Point
    car_direction: list
    wheel_angular_vel_left_back: float
    wheel_angular_vel_right_back: float
    wheel_angular_vel_left_front: float
    wheel_angular_vel_right_front: float
    lidar: list
    lidar_direction: list
    target_vel: list


class ControlSignal(BaseModel):
    wheel_vel: float  # rad/s
    steering_angle: float  # degree, left: -, right: +
