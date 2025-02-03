# https://docs.nerf.studio/quickstart/installation.html#using-an-interactive-container 
# Run with SUDO if you have GPU issues

default_mount_dir="/home/csrobot/data-mount"
# Get arguments or use default values
mount_dir="${1:-$default_mount_dir}"
echo "Mounting $mount_dir..."

docker run --gpus all \
	    --name nerfstudio-container \
        --hostname nerfstudio \
            --user root \
            -v $mount_dir:/workspace/ \
            -v /home/csrobot/.cache/:/home/user/.cache/ \
            -v /home/csrobot/moad_cui/docker/ns-docker/scripts:/workspace/scripts/ \
            -p 7007:7007 \
            --rm \
            -it \
            --shm-size=32gb \
            ghcr.io/nerfstudio-project/nerfstudio "/workspace/scripts/init.sh"
            
