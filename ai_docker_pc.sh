docker run -it --rm --gpus all -v "$(pwd)/AI_pkg:/workspaces/AI_pkg" --network scripts_my_bridge_network --env-file ./.env pros_ai_stable_baselines3:latest /bin/bash
