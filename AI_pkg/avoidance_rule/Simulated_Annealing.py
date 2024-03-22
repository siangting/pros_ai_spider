import random
from utils.rotate_angle import calculate_angle_point
from utils.obs_utils import calculate_dynamic_indices
import pickle
from ros_receive_and_data_processing.config import (
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
)
import os


class ObstacleAvoidanceController:
    def __init__(self, initial_temperature=1.0, cooling_rate=0.95):
        self.temperature = initial_temperature
        self.cooling_rate = cooling_rate
        self.last_turn_direction = None
        self.turn_persistence = 3
        self.bias_counter = 0



    def refined_obstacle_avoidance_with_target_orientation(
        self, lidars, car_quaternion_1, car_quaternion_2, car_pos, target_pos
    ):
        safe_distance = 1.0
        angle_tolerance = 10  # degrees, tolerance for angle alignment

        angle_diff = calculate_angle_point(
            car_quaternion_1, car_quaternion_2, car_pos, target_pos
        )

        obstacle_near = any(lidar < safe_distance for lidar in lidars)
        if obstacle_near:

            front_clear = all(lidars[i] > safe_distance for i in FRONT_LIDAR_INDICES)
            left_clear = all(lidars[i] > safe_distance for i in LEFT_LIDAR_INDICES)
            right_clear = all(lidars[i] > safe_distance for i in RIGHT_LIDAR_INDICES)
            clear_directions = []
            if front_clear:
                return 0
                clear_directions.append(0)  # 前進
            if left_clear:
                return 2
                clear_directions.append(2)  # 左
            if right_clear:
                return 4
                clear_directions.append(4)  # 右

            # # 用溫度影響決策
            # if len(clear_directions) > 1:
            #     if random.random() < self.temperature:
            #         self.temperature *= self.cooling_rate
            #         return random.choice(clear_directions)
            #     else:
            #         self.temperature *= self.cooling_rate
            #         return (
            #             self.last_turn_direction
            #             if self.last_turn_direction in clear_directions
            #             else 0
            #         )
            # elif len(clear_directions) == 1:
            #     self.temperature *= self.cooling_rate
            #     return clear_directions[0]
            # else:
            #     self.temperature *= self.cooling_rate
            #     return random.choice([2, 4])
        else:
            if abs(angle_diff) > angle_tolerance:
                if self.last_turn_direction is None or self.turn_persistence == 0:
                    self.last_turn_direction = 2 if angle_diff > 0 else 4
                    self.turn_persistence = 3
                else:
                    self.turn_persistence -= 2
                self.temperature *= self.cooling_rate
                return self.last_turn_direction
            else:
                self.temperature *= self.cooling_rate
                return 0  # forward
