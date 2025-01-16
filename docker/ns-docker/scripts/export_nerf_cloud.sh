#!/bin/bash

# TO BE RUN FROM INSIDE DOCKER CONTAINER

# Default values
base_dir="/workspace"
object_default="None"
crop_sz_default="0.80" # was 0.42
crop_z_default="-0.6" # was -0.6
points_default="1000000" #1000000
normal_method="model_output" # {open3d,model_output} 

# Get arguments or use default values
object="${1:-$object_default}"
crop_sz="${2:-$crop_sz_default}"
crop_z="${3:-$crop_z_default}"
points="${4:-$points_default}"

# Display the values
echo "Exporting pointcloud for NeRF object: $object"
echo "Crop Size: $crop_sz"
echo "Crop Z Center: $crop_z"
echo "Number of points: $points"

# Set output directory
output_dir="$base_dir/$object/exports"
echo "Output Dir: $output_dir"

# file_name="test.ply"
# echo "Rename File To: $file_name"

# Directory containing the date-named folders
parent_dir="$base_dir/$object/nerfacto"
echo "Checking in folder: $parent_dir"
# Find the most recent folder
most_recent_folder=$(ls -d $parent_dir/*/ | sort -r | head -n 1)

# Export pointcloud from most recent config
if [ -n "$most_recent_folder" ]; then
    echo "Most recent folder: $most_recent_folder"

    ns-export pointcloud --load-config $most_recent_folder/config.yml \
        --output-dir $output_dir \
        --num-points $points \
        --remove-outliers True --normal-method $normal_method --save-world-frame False \
        --obb_center 0.00 0.00 $crop_z \
        --obb_rotation 0.0 0.0 0.0 \
        --obb_scale $crop_sz $crop_sz $crop_sz

    file_count=$(find "$output_dir" -maxdepth 1 -type f | wc -l)
    file_name="$object-cloud-$file_count-$points.ply"
    echo "Rename File To: $file_name"
    old_cloud_file="$output_dir/point_cloud.ply"
    new_cloud_file="$output_dir/$file_name"
    echo "Old: $old_cloud_file"
    echo "New: $new_cloud_file"
    mv $old_cloud_file $new_cloud_file
else
    echo "No matching folders found."
fi


