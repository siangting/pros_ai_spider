import rclpy
import threading
from ros_receive_and_data_processing.AI_spider_node import AI_spider_node
import gymnasium as gym
from stable_baselines3 import PPO
from Spider_RL.RL_training_main import CustomSpiderEnv
from Spider_redirect_RL.RL_training_redirect import CustomSpiderRedirectEnv
from Spider_RL.custom_callback import CustomCallback
from stable_baselines3.common.monitor import Monitor
from Spider_RL.PPOConfig import PPOConfig
from Spider_redirect_RL.redirect_PPOConfig import redirect_PPOConfig
import sys

def init_ros_node():
    rclpy.init()
    node = AI_spider_node()
    thread = threading.Thread(target = rclpy.spin, args = (node,))
    thread.start()
    return node, thread


def load_or_create_model_PPO(env, PPO_Mode: str):
    
    if (PPO_Mode == "forward"):
        try:
            model = PPO.load(PPOConfig.LOAD_MODEL_PATH)
            env = Monitor(env)
            model.set_env(env)
            print(f"Model loaded successfully from {PPOConfig.LOAD_MODEL_PATH}")
            print(f"Model learning rate: {model.lr_schedule(1.0)}")
            print(f"Model policy network: {model.policy}")
        
        except FileNotFoundError: 
            model = PPO("MlpPolicy", 
                        env, verbose = 1, learning_rate = PPOConfig.LEARNING_RATE,
                        n_steps = PPOConfig.N_STEPS, batch_size = PPOConfig.BATCH_SIZE, 
                        n_epochs = PPOConfig.N_EPOCHS, device = "cuda")

            print("Model is not found. Train a new model.")

    elif (PPO_Mode == "redirect"):
        try:
            model = PPO.load(redirect_PPOConfig.LOAD_MODEL_PATH)
            env = Monitor(env)
            model.set_env(env)
            print(f"Model loaded successfully from {redirect_PPOConfig.LOAD_MODEL_PATH}")
            print(f"Model learning rate: {model.lr_schedule(1.0)}")
            print(f"Model policy network: {model.policy}")
        
        except FileNotFoundError: 
            model = PPO("MlpPolicy", 
                        env, verbose = 1, learning_rate = redirect_PPOConfig.LEARNING_RATE,
                        n_steps = redirect_PPOConfig.N_STEPS, batch_size = redirect_PPOConfig.BATCH_SIZE, 
                        n_epochs = redirect_PPOConfig.N_EPOCHS, device = "cuda")

            print("Model is not found. Train a new model.")   
    
    else:
        print("PPO_Mode not exist...")
        sys.exit()   
    

    return model


def train_model_PPO(env):
    model = load_or_create_model_PPO(
        env, PPO_Mode = "forward"
    )
    custom_callback = CustomCallback(PPOConfig.SAVE_MODEL_PATH, PPOConfig.SAVE_MODEL_FREQUENCE)
    model.learn(
        total_timesteps = PPOConfig.TOTAL_TIME_STEPS, callback = custom_callback, log_interval = 1
    )  #  Enter forward env and start forward training

def train_redirect_PPO(env):
    model = load_or_create_model_PPO(
        env, PPO_Mode = "redirect"
    )
    custom_callback = CustomCallback(redirect_PPOConfig.SAVE_MODEL_PATH, redirect_PPOConfig.SAVE_MODEL_FREQUENCE)
    model.learn(
        total_timesteps = redirect_PPOConfig.TOTAL_TIME_STEPS, callback = custom_callback, log_interval = 1
    )  #  Enter redirect env and start redirect training


def gym_env_register(node):
    gym.register(
        id = CustomSpiderEnv.ENV_NAME,  
        entry_point = "Spider_RL.RL_training_main:CustomSpiderEnv",
    )
    return gym.make("CustomSpiderEnv-v0", AI_node = node)

def gym_redirect_env_register(node):
    gym.register(
        id = CustomSpiderRedirectEnv.ENV_NAME,  
        entry_point = "Spider_redirect_RL.RL_training_redirect:CustomSpiderRedirectEnv",
    )
    return gym.make("CustomSpiderRedirectEnv-v0", AI_node = node)

    
def print_usage():
    print("modes:")
    print(" 1 -- Train forward PPO")
    print(" 2 -- Train redirect PPO")


def main(mode):
    node, ros_thread = init_ros_node()

    if mode == "1":
        env = gym_env_register(node)
        train_model_PPO(env)
    elif mode == "2":
        env = gym_redirect_env_register(node)
        train_redirect_PPO(env)
    else:
        print("Please type the correct numbers.")

    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)


