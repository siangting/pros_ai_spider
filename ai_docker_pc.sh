docker run -it --rm -v "$(pwd)/AI_pkg:/workspaces/AI_pkg"  --network scripts_my_bridge_network  --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
