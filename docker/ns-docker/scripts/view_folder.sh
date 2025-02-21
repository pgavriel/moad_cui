#!/bin/bash

# TO BE RUN FROM INSIDE DOCKER CONTAINER

# Default values
base_dir="/workspace"
object_default="None"
model_type="nerfacto"

# Get arguments or use default values
object="${1:-$object_default}"

# Directory containing the date-named folders
parent_dir="$base_dir/$object/$model_type"
echo "Checking in folder: $parent_dir"
# Find the most recent folder
most_recent_folder=$(ls -d $parent_dir/*/ | sort -r | head -n 1)

# Open viewer with most recent config
if [ -n "$most_recent_folder" ]; then
    echo "Most recent folder: $most_recent_folder"
    model_path="$most_recent_folder/config.yml"
    # Open model for viewing
    ns-viewer --load-config $model_path
    
else
    echo "No matching folders found, did you train a $model_type model yet?"
fi
