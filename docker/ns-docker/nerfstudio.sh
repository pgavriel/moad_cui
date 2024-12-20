# https://docs.nerf.studio/quickstart/installation.html#using-an-interactive-container 
# Run with SUDO
docker run --gpus all \
	    --name nerfstudio \
            -u $(id -u) \
            -v /home/csrobot/ns-data:/workspace/ \
            -v /home/csrobot/.cache/:/home/user/.cache/ \
            -v /home/csrobot/ns-docker/scripts:/workspace/scripts/ \
            -p 7007:7007 \
            --rm \
            -it \
            --shm-size=32gb \
            ghcr.io/nerfstudio-project/nerfstudio 
