docker run -it --rm -v "$(pwd)/AI_pkg:/AI_pkg" --runtime nvidia --network host  --env-file ./.env ghcr.io/otischung/pros_ai_image:0.0.0 /bin/bash
