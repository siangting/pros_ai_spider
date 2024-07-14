import rclpy
import threading
from ros_receive_and_data_processing.AI_node import AI_node
from avoidance_rule.rule_base import RuleBasedController

# from car_supervised.lstm_inference import supervised_inference
from car_navigation.navigation_main import NavigationController
from robot_arm.robot_control import RobotArmControl

# RL
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from car_RL.RL_training_main import CustomCarEnv
from car_RL.custom_callback import CustomCallback


def load_or_create_model(env, model_path):
    try:
        model = PPO.load(model_path)  #  load model
        model.set_env(env)
    except FileNotFoundError:  #  找不到就重新train一個
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.001, device="cuda")
    return model


def train_model(env):
    model = load_or_create_model(
        env, "./Model/ppo_custom_car_model_278000_1703457082.884543"
    )
    custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
    total_timesteps = 1000000  # 訓練回合數
    model.learn(
        total_timesteps=total_timesteps, callback=custom_callback
    )  #  進入env開始訓練


def gym_env_register(AI_node):
    gym.register(
        id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
        entry_point="car_RL.RL_training_main:CustomCarEnv",  # 模組名稱:類名稱
    )
    return gym.make("CustomCarEnv-v0", AI_node=AI_node)


def init_ros_node():
    """node初始化並開一個thread跑ros node"""
    rclpy.init()
    node = AI_node()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread


def main(mode):
    node, ros_thread = init_ros_node()

    if mode == "1":
        rule_controller = RuleBasedController(
            node,
        )
        rule_controller.run()
    elif mode == "2":
        navigation_controller = NavigationController(
            node,
        )
        navigation_controller.run()
    elif mode == "3":
        robot_controler = RobotArmControl(
            node,
        )
        robot_controler.action()
    elif mode == "4":
        env = gym_env_register(node)
        train_model(env)

    else:
        print("Please type the correct numbers.")

    rclpy.shutdown()
    ros_thread.join()


def print_usage():
    print("modes:")
    print(" 1 -- rule-based.")
    # print(" 2 -- supervised learning inference.")
    print(" 2 -- ros2 navigation.")
    print(" 3 -- ros2 arm.")
    print(" 4 -- RL.")


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)
