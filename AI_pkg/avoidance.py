import numpy as np


def refined_obstacle_avoidance_with_target_orientation(lidars,
                                                       car_quaternion_1,
                                                       car_quaternion_2,
                                                       car_pos,
                                                       target_pos):
    """
    Adjust the vehicle's heading towards the target until the angle difference is less than 5 degrees,
    then move forward, continuously correcting the orientation to maintain the course towards the target.
    Switch to obstacle avoidance if any lidar detects an object within the safe distance.

    :param lidars: List of 8 distance readings from lidars placed around the vehicle.
    :param car_quaternion_1: 4 digits representing the rotation angle along z-axis of the car.
    :param car_quaternion_2: 4 digits representing the rotation angle along w-axis of the car.
    :param car_pos: The position of the car.
    :param target_pos: The position of the target.
    :returns: Movement direction (0: Forward, 1: Turn Left, 2: Turn Right, 3: Backward)
    """
    safe_distance = 0.5  # meters or as per lidar unit
    angle_tolerance = 5  # degrees, the tolerance for angle alignment

    # Calculate the smallest angle difference to the target, considering the circular nature of angles
    angle_diff = calculate_angle_point(car_quaternion_1,
                                       car_quaternion_2,
                                       car_pos,
                                       target_pos)

    """
    The lidar position in the car.
        front
         0 7
       1     6
       2     5
         3 4
    """
    # Check if any lidar reading indicates a close obstacle
    obstacle_near = any(lidar < safe_distance for lidar in lidars)

    # If an obstacle is detected, switch to obstacle avoidance mode
    if obstacle_near:
        front_clear = lidars[0] > safe_distance and lidars[7] > safe_distance
        left_clear = all(lidar > safe_distance for lidar in lidars[1:4])
        right_clear = all(lidar > safe_distance for lidar in lidars[4:7])

        # Decide on movement based on clear path
        if front_clear:
            return 0  # Forward
        elif left_clear:
            return 1  # Turn left
        elif right_clear:
            return 2  # Turn right
        else:
            return 3  # No clear path, reverse
    else:
        # No obstacle near, align and move towards the target
        if np.abs(angle_diff) > 30:
            # Rotate towards target angle
            if angle_diff > 0:
                return 1  # if (target_angle - current_angle + 360) % 360 < 180 else 2
            else:
                return 2
        else:
            # Move forward as the angle is within tolerance
            return 0  # Forward


def get_yaw_from_quaternion(z, w):
    return np.degrees(2 * np.arctan2(z, w))


def get_direction_vector(current_position, target_position):
    return np.array(target_position) - np.array(current_position)


def get_angle_to_target(car_yaw, direction_vector):
    target_yaw = np.arctan2(direction_vector[1], direction_vector[0])
    angle_diff = target_yaw - np.radians(car_yaw)

    return (np.degrees(angle_diff)) % 360


def calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos):
    car_yaw = get_yaw_from_quaternion(car_quaternion_1, car_quaternion_2)
    direction_vector = get_direction_vector(car_pos, target_pos)
    angle_to_target = get_angle_to_target(car_yaw, direction_vector)
    # angle_diff = np.abs(angle_to_target - 180)
    angle_diff = angle_to_target - 360 if angle_to_target > 180 else angle_to_target
    print(angle_diff)
    return angle_diff
