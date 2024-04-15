# config.py

"""
vel、rotate_vel
"""
vel = 20.0
rotate_vel = 5.0

ACTION_MAPPINGS = {
    0: [vel, vel, vel, vel],  # 前進
    1: [rotate_vel, rotate_vel * 2, rotate_vel, rotate_vel * 2],  # 左前
    2: [-rotate_vel, rotate_vel, -rotate_vel, rotate_vel],  # 左自轉
    3: [-vel, -vel, -vel, -vel],  # 後退
    4: [rotate_vel, -rotate_vel, rotate_vel, -rotate_vel],  # 右自轉
    5: [rotate_vel * 2, rotate_vel, rotate_vel * 2, rotate_vel],  # 右前
    6: [0.0, 0.0, 0.0, 0.0],
}

"""
LIDAR_RANGE : 設定lidar要有幾個偵測
FRONT_LIDAR_INDICES : 車子偵測前面障礙物的lidar range
LEFT_LIDAR_INDICES : 車子偵測左邊障礙物的lidar range
RIGHT_LIDAR_INDICES : 車子偵測右邊障礙物的lidar range
"""
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(
    range(-15, 0)
)  # 0~16個和-15到0的LIDAR RANGE
LEFT_LIDAR_INDICES = list(range(16, 46))
RIGHT_LIDAR_INDICES = list(range(-45, -15))

"""
OBSTACLE_DISTANCE : 開始閃避障礙物的距離(公尺)
WALL_DISTANCE : 判定為撞到牆壁的距離
TARGET_DISTANCE : 判定為成功到達目標的距離
"""
OBSTACLE_DISTANCE = 0.7
WALL_DISTANCE = 0.2
TARGET_DISTANCE = 1
