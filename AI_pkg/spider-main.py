import rclpy
import threading
from ros_receive_and_data_processing.AI_spider_node import AI_spider_node


import gymnasium as gym
from stable_baselines3 import PPO

from Spider_RL.RL_training_main import CustomSpiderEnv
from Spider_RL.custom_callback import CustomCallback
from stable_baselines3.common.monitor import Monitor



def init_ros_node():
    rclpy.init()
    node = AI_spider_node()
    thread = threading.Thread(target = rclpy.spin, args = (node,))
    thread.start()
    return node, thread


def load_or_create_model_PPO(env, model_path):
    try:
        model = PPO.load(model_path)  #  load model
        env = Monitor(env)
        model.set_env(env)
        print(f"Model loaded successfully from {model_path}")
        print(f"Model learning rate: {model.lr_schedule(1.0)}")
        print(f"Model policy network: {model.policy}")
    except FileNotFoundError:  #  找不到就重新train一個
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.001, device="cuda")
        print("Model not found. Training a new model.")
    return model


def train_model_PPO(env):
    model = load_or_create_model_PPO(
        env, "./Model/ppo_custom_car_model_1000_1721272714.460118"
    )
    custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
    total_timesteps = 100000  # 訓練回合數
    model.learn(
        total_timesteps=total_timesteps, callback=custom_callback, log_interval=1
    )  #  進入env開始訓練
    
    # TODO save model

def gym_env_register(node):
    gym.register(
        id = CustomSpiderEnv.ENV_NAME,  
        entry_point="Spider_RL.RL_training_main:CustomSpiderEnv",  # TODO modify
    )
    return gym.make("CustomSpiderEnv-v0", AI_node = node)

    
def print_usage():
    print("modes:")
    print(" 1 -- PPO")


def main(mode):
    node, ros_thread = init_ros_node()

    if mode == "1":
        env = gym_env_register(node)
        train_model_PPO(env)
    else:
        print("Please type the correct numbers.")

    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)


