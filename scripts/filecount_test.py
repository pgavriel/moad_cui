"""
This script serves two primary functions and is mostly run directly after scan data is collected:   
    1. Verify that the expected number of frames exist on the output folder
    2. Downscale (mipmap) the source frames to prepare them for downstream uses

Downscaled frames are used for NeRF training, model inference, benchmarking, etc. 
"""

import os
import sys
from pathlib import Path
import shutil
import cv2

import argparse
import fnmatch
import time

VERBOSE = False

def check_num_images_in_folder(folder_path, expected_count, patterns:tuple=None, images_folder=False):
    """
    Check if the folder contains the expected number of images.
    :param folder_path: Path to the folder containing images.
    :param expected_count: Expected number of images.
    :return: True if the count matches, False otherwise.
    """
    # Default patterns
    if patterns is None: 
        patterns = ('*.jpg', '*.jpeg', '*.png')
    elif isinstance(patterns, str):
        patterns = (patterns,)

    if VERBOSE: print(f"Checking Patterns: {patterns}")
    images = [f for f in os.listdir(folder_path)
          if any(fnmatch.fnmatch(f, p) for p in patterns)]
    count = len(images)
    if count != expected_count:
        if images_folder:
            print(f"\033[91mFound {count} images in {folder_path} [  \033[91mEXPECTED {expected_count}\033[91m  ]\033[0m")
        else:
            print(f"\033[91mFound {count} images in {folder_path} [  \033[91mEXPECTED {expected_count}\033[91m  ]\033[0m")
        return False
    else:
        if images_folder:
            print(f"\033[96mFound {count} images in {folder_path} [  \033[96mOK\033[96m  ]\033[0m")
        else:
            print(f"\033[92mFound {count} images in {folder_path} [  \033[92mOK\033[92m  ]\033[0m")

    return True


def generate_downscale_folders(output_path, mipmap_level:int, base_text="images"):
    assert mipmap_level >= 0
    subfolders = []
    factor = 1
    for i in range (mipmap_level+1):
        if i == 0:
            folder_name = base_text
            subfolders.append(folder_name)
        else:
            folder_name = base_text+"_"+str(factor)
            subfolders.append(folder_name)
        factor *= 2

        if not os.path.isdir(os.path.join(output_path,folder_name)):
            if VERBOSE: print(f"Mipmap folder \'{folder_name}\' not found, creating it now.")
            os.mkdir(os.path.join(output_path,folder_name))
        
    print(f"Generated subfolders for Mipmap level {mipmap_level}: {subfolders}")
    return subfolders


def process_images(output_path, dslr_folder, mipmap_level:int, expected_frames:int):
    """ 
    Create mipmaps for specified dslr_folder  
    ARGS:
      - output_path : Full path to scan/pose folder  
      - dslr_folder : Full path to DSLR/input frames  
      - mipmap_level: (int) Number of times to half image resolution  
      
    TO FIX: Instead of applying to all files in 'dslr_folder' use pattern matching for more generic 'input_folder'  
    """
    downscale_folders = generate_downscale_folders(output_path, mipmap_level,base_text="images")

    # Check if downscaling has already been completed
    ALL_IMAGES_SUBFOLDERS_COMPLETE = True
    for subfolder in downscale_folders:
        subfolder_path = os.path.join(output_path, subfolder)

        # Check if the subfolder contains the expected number of images
        if check_num_images_in_folder(subfolder_path, expected_frames, images_folder=True):
            continue
        else:
            ALL_IMAGES_SUBFOLDERS_COMPLETE = False
    if ALL_IMAGES_SUBFOLDERS_COMPLETE:
        print("This scan has already been downscaled, no action needed.")
        return
    
    # for each image in DSLR
    for index, image_file in enumerate(sorted(os.listdir(dslr_folder))):

        # rename image file to "frame_0000X.jpg"
        frame_id = f"frame_{str(index + 1).zfill(5)}.jpg"

        # load image
        image_path = os.path.join(dslr_folder, image_file)
        image = cv2.imread(image_path)

        if image is None:
            print(f"Error reading image {image_path}. Skipping...")
            continue

        # Save all downscaled copies
        for i, scale_folder in enumerate(downscale_folders):
            save_path = os.path.join(output_path, scale_folder, frame_id)
            if i != 0: image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5) # The first folder is not downscaled
            cv2.imwrite(save_path, image)  

        # Print a simple loading bar
        bar_length = 50
        progress = (index + 1) / len(os.listdir(dslr_folder))
        block = int(bar_length * progress)
        bar = "█" * block + "-" * (bar_length - block)
        print(f"\rProcessing images: {bar} {index + 1}/{len(os.listdir(dslr_folder))}", end="", flush=True)

    print(f"\nProcessed all images in {dslr_folder}.")


def main(args):
    print_args(args)

    if len(args.target_scans) == 0:
        print("Error: No target scans provided")
        sys.exit(1)
    
    for scan in args.target_scans:
        print(f"\nProcessing \'{scan}\'...")
        output_path = os.path.join(args.data_root,scan)
        frame_path = os.path.join(args.data_root,scan,args.frame_subdir)
        if not os.path.isdir(frame_path):
            print(f"Error: {frame_path} is not a directory, skipping...")
            continue

        # TASK 1: Verify frame count
        if args.check_count: 
            print("Checking source frame count...")
            if not check_num_images_in_folder(frame_path, args.total_frames, patterns=args.frame_pattern, images_folder=False):
                print("Error: Scan did not have the expected number of frames, skipping...")
                continue

        # TASK 2: Downscale / Mipmap Frames
        if args.downscale:
            print(f"\nDownscaling images for input folder: {frame_path}")
            process_images(output_path,frame_path,args.mipmap_level,args.total_frames)


def print_args(args, title: str = "File Checker Script Arguments") -> None:
    """Print all argparse values, aligned, sorted by name."""
    items = vars(args)
    width = max((len(k) for k in items), default=0)
    print(f"  {title}:")
    for key in items:
        print(f"    {key:<{width}} : {items[key]}")
        
if __name__ == "__main__":
    DEFAULT_DATA_ROOT       = "/home/csrobot/MOAD_DATA"
    DEFAULT_TOTAL_FRAMES    = 360
    CHECK_COUNT             = True
    DOWNSCALE_FRAMES        = True

    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=str, 
                        default=DEFAULT_DATA_ROOT,
                        help="Path to scan data root, target scans are specified relative to this path.")
    parser.add_argument('--target-scans', nargs='+', type=str, 
                        help="Target scan/pose folder to check/downscale (i.e. 'obj1/pose-a'). Accepts multiple")
    parser.add_argument("--frame-subdir", type=str, 
                        default="DSLR", 
                        help="Scan subfolder to check/downscale")
    parser.add_argument("-p", "--frame-pattern", type=str, 
                        default='cam[1-5]_[0-9][0-9][0-9]_img.jpg', 
                        help="Pattern to match for counting frames")

    parser.add_argument("--check-count", type=bool, 
                        default=CHECK_COUNT, 
                        help="Verify the image count in each scan is correct")
    parser.add_argument("-t", "--total-frames", type=int, 
                        default=DEFAULT_TOTAL_FRAMES, 
                        help="Expected number of frames in image subfolder")
    
    parser.add_argument("--downscale", type=bool, 
                        default=DOWNSCALE_FRAMES, 
                        help="Apply mipmap style downscaling to sorted image frames.")
    parser.add_argument("--mipmap-level", type=int, 
                        default=3, 
                        help="How many times to half the image size")
    
    args = parser.parse_args()

    if not args.check_count and not args.downscale:
        print("ERROR: No task specified.\n > Please set either --check-count or --downscale to 'true'.")
        sys.exit(1)

    main(args)