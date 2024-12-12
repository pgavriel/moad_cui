#!/bin/bash

root_dir="/home/csrobot/colmap/55mm"
cd $root_dir
img_dir="$root_dir/DSLR"

colmap feature_extractor \
    --database_path database.db \
    --image_path $img_dir \
    --ImageReader.camera_model OPENCV \
    --ImageReader.camera_params "13811,13811,3000,2000,0.09,0,0,0"

colmap exhaustive_matcher --database_path database.db

mkdir "$root_dir/sparse"
colmap mapper \
    --database_path database.db \
    --image_path $img_dir \
    --output_path sparse/

colmap image_undistorter \
    --image_path $img_dir \
    --input_path sparse/ \
    --output_path dense/ \
    --output_type COLMAP

colmap patch_match_stereo \
    --workspace_path dense/ \
    --workspace_format COLMAP \
    --DenseStereo.geom_consistency true

colmap model_converter \
    --input_path sparse/ \
    --output_path sparse_text/ \
    --output_type TXT


