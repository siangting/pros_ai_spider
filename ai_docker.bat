@echo off
docker run -it --rm -v "%cd%/AI_pkg:/workspaces/AI_pkg" --network pros_app_my_bridge_network --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
