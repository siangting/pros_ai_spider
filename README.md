# PROS AI Spider

![Static Badge](https://img.shields.io/badge/ROS2-green)
![Static Badge](https://img.shields.io/badge/Unity-black)
![Static Badge](https://img.shields.io/badge/python3-blue)
![Static Badge](https://img.shields.io/badge/docker-blue)
![Static Badge](https://img.shields.io/badge/websocket-purple)  

This is a project about robot digital twins.  
[Motion Demo in Unity](https://youtu.be/rl5G2wjM3S0)


## User Guide

### Build Docker image
![Static Badge](https://img.shields.io/badge/docker-blue)  
The project image inherits from  [pros_base_image](https://github.com/screamlab/pros_base_image/blob/main/README.md).  

Rename the image to  `pros_ai_stable_baselines3:latest`.

### ROSBridge
![Static Badge](https://img.shields.io/badge/ROS2-green)  
Build the rosbridge `scripts_my_bridge_network` manually.

### Unity project
![Static Badge](https://img.shields.io/badge/Unity-black)  
Download the [pros_integration_spider](https://paia-tech.synology.me:8943/pros/pros_integration_spider#) project and open it in Unity.

### Create Model folders
```bash
cd AI_pkg
```
```bash
mkdir Model
```
```bash
mkdir redirect_Model
```
If you have RL Models have been trained, then move the `.pt` model files in the folders created above. 

### Run the project
1.  Open and run pros_integration_spider project in Unity.
2.  Launch Docker.
3.  Enter the Linux or wsl terminal.  
    Run a container from `pros_ai_stable_baselines3:latest`
    ```bash
    ./ai_docker_pc.sh
    ``` 
    ```bash
    pip install stable-baselines3==1.1.0
    ```
    ROS2 colcon build the project by the hotkey
    ```bash
    r
    ```  
    ```bash
    cd AI_pkg
    ```  
    ```bash
    python3 spider-main.py
    ```  


## Code Architechture
### pros AI spider Architecture
![pros AI spider Architecture](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/pros_AI%20spider%20Architecture.drawio.png)


### spider-main.py Architecture
![spider-main.py Architecture](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/spider-main.drawio.png)

### AI spider node subscribe unity data flowchart
![AI spider node subscribe unity data flowchart](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/AI%20spider%20node%20subscribe%20unity%20data.drawio.png)

### RL training main flowchart
![RL training main flowchart](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/RL%20training%20main.drawio.png)

### PPO model Architechture
![PPO model Architechture](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/PPO.drawio.png)

### Multi-PPO Predict flowchart
![Multi-PPO Predict flowchart](https://github.com/roger20415/pros_AI_spider/blob/develop/diagram/Multi-PPO_predict.drawio.png)


## Other Resource
[Circuit and Machenism Design](https://drive.google.com/drive/folders/139djy3Gg4i-CpHx8RhV_BGYDbM0R4fZM?usp=sharing)  
[3D Model in Fusion360](https://a360.co/4enicxd)