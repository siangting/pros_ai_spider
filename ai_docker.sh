docker run -it --rm -v "$(pwd)/AI_pkg:/AI_pkg" --runtime nvidia --network host  --env-file ./.env ghcr.io/otischung/pros_ai:latest /bin/bash
