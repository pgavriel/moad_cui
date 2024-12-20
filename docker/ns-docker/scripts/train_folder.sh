# https://docs.nerf.studio/quickstart/installation.html#using-an-interactive-container 
#!/bin/bash

# Accessing arguments
base_dir="/workspace"
object="$1"
subfolder="DSLR"
stop_after_training="True"
predict_normals="False"
camera_optimizer="SO3xR3" #default: SO3xR3, can set to 'off'

echo "Base Dir: $base_dir"
echo "Object: $object"
echo "Data Subfolder: $subfolder"
echo "Stop After Training: $stop_after_training"
echo "Predict Normals: $predict_normals"

# Directory to check
target_directory="$base_dir/$object"
# Verify Directory Exists
if [ -d "$target_directory" ]; then
    echo "Target object directory $target_directory found."
else
    echo "Target object directory $target_directory not found." >&2
    exit 1
fi

# Required folder names
required_folders=("images" "images_2" "images_4" "images_8")

# Flag to track missing folders
all_folders_exist=true

echo "Checking Directory $target_directory ..."
# Check for each folder
for folder in "${required_folders[@]}"; do
    if [ ! -d "$target_directory/$folder" ]; then
        echo "Missing folder: $folder"
        all_folders_exist=false
    fi
done

# Proceed if all folders exist
if $all_folders_exist; then
    echo "All required folders exist. Continuing..."
    # Your code here
else
    echo "One or more required folders are missing. Processing data first..."
    ns-process-data images --data $target_directory/$subfolder \
        --output-dir $target_directory \
        --skip-colmap   
fi

# Start Training
ns-train nerfacto --data $target_directory \
    --output-dir $base_dir/ \
    --viewer.quit-on-train-completion $stop_after_training \
    --pipeline.model.predict-normals $predict_normals \
    --pipeline.model.camera-optimizer.mode $camera_optimizer \
    --pipeline.model.far-plane 1000.0 \
    --log-gradients True


# docker run --gpus all \
# 	    --name nerfstudio \
#             -u $(id -u) \
#             -v /home/csrobot/ns-data:/workspace/ \
#             -v /home/csrobot/.cache/:/home/user/.cache/ \
#             -p 7007:7007 \
#             --rm \
#             -it \
#             --shm-size=24gb \
#             ghcr.io/nerfstudio-project/nerfstudio ns-train nerfacto --data /workspace/$1 --output-dir /workspace/$1 --viewer.quit-on-train-completion True
            
# echo "Done!"