"""
GS 7/10/2025
functions to:
- count images in ANY folder (via path args) and compare to expected 360 (for full degree rotation)
- create subfolders for images, images_2, images_4, images_8
    - NOTE: renames/saves/downscales each image from DSLR to these subfolders
- optionally add progress delay

OTHER THINGS:
- during counting, the CLI will print different colors based on the results:
    - red text indicates incorrect number of images regardless of folder
    - green text indicates correct number of images for DSLR folder
    - cyan text indicates correct number of images for images subfolders
- ARGS:
    - "--create" 
        - will deal with ALL images subfolders (rename, save, downscale)
        - NOTE: if any images subfolder does not have have 360 images, ALL FOLDERS WILL BE OVERWRITTEN
    - "--count"
        - will only count images in the DSLR folder and images subfolders
    - "--prog_delay"
        - will introduce a delay between processing each object folder for better progress monitoring
    - "--delay=<seconds>"
        - sets the delay time in seconds (default is 0.0, no delay)

TODO:
- only show error messages for incomplete folders?
- definitely possibility for more modular code
"""


import os
import sys
from pathlib import Path
import shutil
import cv2

import argparse
import time


NUM_IMAGES = 360  # Expected number of images in each pose folder


root_dir = "g:\\MOAD_V2"


def check_num_images_in_folder(folder_path, expected_count=NUM_IMAGES, images_folder=False):
    """
    Check if the folder contains the expected number of images.
    :param folder_path: Path to the folder containing images.
    :param expected_count: Expected number of images.
    :return: True if the count matches, False otherwise.
    """
    images = [f for f in os.listdir(folder_path) if f.endswith(('.jpg', '.jpeg', '.png'))]
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


def check_and_create_images_subfolders(object_folder, object_path, pose_folder, dslr_folder, NUM_IMAGES=NUM_IMAGES, prog_delay=False, delay=0.0):
    """
    Check if the required subfolders exist, create them if they don't.
    :param object_folder: Name of the object folder.
    :param object_path: Path to the object folder.
    :param pose_folder: Name of the pose folder.
    :param dslr_folder: Path to the DSLR folder.
    """

    ALL_IMAGES_SUBFOLDERS_COMPLETE = True

    for subfolder in ["images", "images_2", "images_4", "images_8"]:

        if prog_delay:
            time.sleep(delay)  # slow down to show progress

        subfolder_path = os.path.join(object_path, pose_folder, subfolder)

        if not os.path.isdir(subfolder_path):
            print(f"No {subfolder} folder found in {pose_folder} for {object_folder}. Creating...")
            os.makedirs(subfolder_path)
            continue

        # check if the subfolder contains the expected number of images
        if check_num_images_in_folder(subfolder_path, NUM_IMAGES, images_folder=True):
            # print(f"No action needed.")
            # continue to the next subfolder
            continue
        else:
            # print(f"Subfolder {subfolder_path} does not contain the expected number of images. It will be overwritten.")
            ALL_IMAGES_SUBFOLDERS_COMPLETE = False

    if ALL_IMAGES_SUBFOLDERS_COMPLETE:
        print(f"All images subfolders for {object_folder} - {pose_folder} are complete. No further action needed.")
        return
    else:
        print(f"Some images subfolders for {object_folder} - {pose_folder} were incomplete. Processing images...")
        # process images in the DSLR folder
        print("temp skipping image processing for now...")
        # process_images(object_folder, object_path, pose_folder, dslr_folder)
        return


def process_images(object_folder, object_path, pose_folder, dslr_folder):
    """ 
    Process images in the DSLR folder, rename them, and save them into the appropriate subfolders.
    :param object_folder: Name of the object folder.
    :param object_path: Path to the object folder.
    :param pose_folder: Name of the pose folder.
    :param dslr_folder: Path to the DSLR folder.
    """

    # Check if the DSLR folder exists
    if not os.path.isdir(dslr_folder):
        print(f"DSLR folder {dslr_folder} does not exist. Skipping...")
        return

    # for each image in DSLR
    for index, image_file in enumerate(sorted(os.listdir(dslr_folder))):

        # rename image file to "frame_0000X.jpg"
        new_name = f"frame_{str(index + 1).zfill(5)}.jpg"

        # load image
        image_path = os.path.join(dslr_folder, image_file)
        image = cv2.imread(image_path)

        if image is None:
            print(f"Error reading image {image_path}. Skipping...")
            continue

        # save renamed image into first images subfolder
        first_images_subfolder = os.path.join(object_path, pose_folder, "images")

        # NOTE: first image not downalscaled, just renamed
        # downscaled_image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)

        # rename and save the image:
        new_image_path = os.path.join(first_images_subfolder, new_name)
        cv2.imwrite(new_image_path, image)  
        # print(f"Saved renamed image as {new_name} in {first_images_subfolder}")

        # downscale and save renamed image into images_2 subfolder
        second_images_subfolder = os.path.join(object_path, pose_folder, "images_2")
        downscaled_image_2 = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        # NOTE: downscaled to 50% of the original size
        new_image_path_2 = os.path.join(second_images_subfolder, new_name)
        cv2.imwrite(new_image_path_2, downscaled_image_2)
        # print(f"Saved downscaled image as {new_name} in {second_images_subfolder}")

        # downscale and save renamed image into images_4 subfolder
        third_images_subfolder = os.path.join(object_path, pose_folder, "images_4")
        downscaled_image_4 = cv2.resize(image, (0, 0), fx=0.25, fy=0.25)
        # NOTE: downscaled to 25% of the original size
        new_image_path_4 = os.path.join(third_images_subfolder, new_name)
        cv2.imwrite(new_image_path_4, downscaled_image_4)
        # print(f"Saved downscaled image as {new_name} in {third_images_subfolder}")

        # downscale and save renamed image into images_8 subfolder
        fourth_images_subfolder = os.path.join(object_path, pose_folder, "images_8")
        downscaled_image_8 = cv2.resize(image, (0, 0), fx=0.125, fy=0.125)
        # NOTE: downscaled to 12.5% of the original size
        new_image_path_8 = os.path.join(fourth_images_subfolder, new_name)
        cv2.imwrite(new_image_path_8, downscaled_image_8)
        # print(f"Saved downscaled image as {new_name} in {fourth_images_subfolder}")

        print(f"Successfully processed image {new_name} to images, images_2, images_4, and images_8 subfolders.")

    print(f"Processed all images in {dslr_folder} for {object_folder} - {pose_folder}.")







def main(count_images=False, create_folders=False, prog_delay=False, delay=0.0):
    # all_objects = []
    # loop over all folders (aka all objects that have been scanned)
    for object_folder in sorted(os.listdir(root_dir)):

        if count_images:
            # slow down to show progress
            if prog_delay:
                time.sleep(delay)
        print("\n")
        print(object_folder)

        # setup subdirectory path to the object folder
        object_path = os.path.join(root_dir, object_folder)
        if not os.path.isdir(object_path):
            print(f"Skipping {object_folder}, not a directory.")
            continue

        # check if "pose-" folders exist, place into a list:
        pose_folders = [f for f in os.listdir(object_path) if f.startswith("pose-")]
        if not pose_folders:
            print(f"No pose folders found for {object_folder}. Skipping...")
            continue
        print(f"Found {len(pose_folders)} pose folders for {object_folder}.")

        # loop over each pose folder
        for pose_folder in sorted(pose_folders):

            # check for DSLR subfolder
            dslr_folder = os.path.join(object_path, pose_folder, "DSLR")
            if not os.path.isdir(dslr_folder):
                print(f"No DSLR folder found in {pose_folder} for {object_folder}. Skipping...")
                continue


            # check if the DSLR folder contains the expected number of images
            if count_images:
                if not check_num_images_in_folder(dslr_folder, NUM_IMAGES, images_folder=False):
                    continue

            # check and create images subfolders
            if create_folders:
                check_and_create_images_subfolders(object_folder, object_path, pose_folder, dslr_folder, NUM_IMAGES, prog_delay, delay)

            # exit after processing the first pose folder   
            # exit(1)          

    # print(f"Found {len(all_objects)} objects with DSLR images:")
    # for obj, count in all_objects:
    #     print(f"{obj}: {count} images")



        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", action="store_true", help="Count images")
    parser.add_argument("--create", action="store_true", help="Create folders")
    parser.add_argument("--prog-delay", action="store_true", help="Enable progress delay")
    parser.add_argument("--delay", type=float, default=0.0, help="Delay between processing images (in seconds, used if --prog-delay is set)")
    args = parser.parse_args()

    if not args.count and not args.create:
        print("Please specify --count or --create and optionally --prog-delay with a delay value.")
        sys.exit(1)

    print(f"Running with count={args.count}, create={args.create}, prog_delay={args.prog_delay}, delay={args.delay}")

    main(args.count, args.create, args.prog_delay, args.delay)