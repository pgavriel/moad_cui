#!/bin/bash

# Get the directory of the current script
base_dir="$(dirname "$(realpath "$0")")"
object_name="$1"

# First use NerfStudio to process the image data and train a nerfacto model
if "$base_dir/train_folder.sh" "$object_name"; then
    echo "NeRF Model trained successfully."
else
    echo "NeRF Model training failed." >&2
    exit 1
fi

# Second, export pointcloud from trained model
if "$base_dir/export_nerf_cloud.sh" "$object_name"; then
    echo "Cloud exported successfully."
else
    echo "Cloud export failed." >&2
    exit 1
fi

