from pyexpat import model
import rclpy
import threading
import gymnasium as gym
from stable_baselines3 import PPO
from Spider_RL.RL_training_main import CustomSpiderEnv
from Spider_redirect_RL.RL_training_redirect import CustomSpiderRedirectEnv
from Spider_inference.inference_config import InferenceConfig
from ros_receive_and_data_processing.AI_spider_node import AI_spider_node
from utils import utils

class Inference:

    _forward_model = PPO.load(InferenceConfig.FORWARD_MODEL_PATH)
    _redirect_model = PPO.load(InferenceConfig.REDIRECT_MODEL_PATH)


    def init_AI_spider_node():
        rclpy.init()
        node = AI_spider_node()
        thread = threading.Thread(target = rclpy.spin, args = (node,))
        thread.start()
        return node, thread

    def gym_env_register(self, node):
        gym.register(
            id = CustomSpiderEnv.ENV_NAME,  
            entry_point = "Spider_RL.RL_training_main:CustomSpiderEnv",
        )
        return gym.make("CustomSpiderEnv-v0", AI_node = node)

    def gym_redirect_env_register(self, node):
        gym.register(
            id = CustomSpiderRedirectEnv.ENV_NAME,  
            entry_point = "Spider_redirect_RL.RL_training_redirect:CustomSpiderRedirectEnv",
        )
        return gym.make("CustomSpiderRedirectEnv-v0", AI_node = node)
    

    def select_model(self, angle: float, current_model) -> str:
        """
        Select the model to use based on the spider toward angle.
        If angle < 30 degrees, use the forward model.
        If angle >= 30 degrees, use the turning model.
        """
        if current_model == "forward" and abs(angle) < 30:
            next_model: str = "forward"
        elif current_model == "forward" and abs(angle) >= 30:
            next_model: str = "redirect"
        elif current_model == "redirect" and abs(angle) > 3:
            next_model: str = "redirect"
        elif current_model == "redirect" and abs(angle) <= 3:
            next_model: str = "forward"
        else:
            print("Choose model fail...")

        return next_model
    
    def get_spider_toward_angle(self, node) -> float:
        unity_data: dict = utils.get_observation(node)
        angle: float = abs(unity_data["offset_angle"])
        
        return angle


    def inference(self, node):

        forward_env = self.gym_env_register(node)
        redirect_env = self.gym_redirect_env_register(node)

        redirect_env.reset()
        obs, _ = forward_env.reset()
        

        current_model = "forward"
        while (True):
            # Get the robot's current angle
            spider_toward_angle = self.get_spider_toward_angle(node)

            # Select the model and environment based on the angle
            current_model = self.select_model(spider_toward_angle, current_model)

            if current_model == "forward":
                model = self._forward_model
                env = forward_env
            else:
                model = self._redirect_model
                env = redirect_env

            action, _ = model.predict(obs)

            obs, reward, done, info, _ = env.step(action)