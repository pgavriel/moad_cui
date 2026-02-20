import os
import cv2
import random
import numpy as np
import math
from pathlib import Path
import re

def find_files(root_dir: str, pattern: str, max_depth: int, ignore_case = False):
    """
    Recursively search for files under `root_dir` up to `max_depth` levels deep,
    returning full paths for files whose names match the given regex `pattern`.

    Args:
    root_dir (str): The root directory to start searching from.
    pattern (str): A regular expression to match filenames (not paths).
    max_depth (int): Maximum depth to search (0 means only the root).

    Returns:
    list[str]: Full paths to matching files.
    """
    root_path = Path(root_dir).resolve()
    if ignore_case:
        regex = re.compile(pattern, re.IGNORECASE)
    else:
        regex = re.compile(pattern)
    matches = []

    # Walk manually with depth control
    def _walk(dir_path: Path, depth: int):
        if depth > max_depth:
            return
        for entry in os.scandir(dir_path):
            if entry.is_file() and regex.search(entry.name):
                matches.append(str(Path(entry.path).resolve()))
            elif entry.is_dir():
                _walk(Path(entry.path), depth + 1)

    _walk(root_path, 0)
    return matches

def images_to_collage_video(
    image_paths,
    output_path,
    grid_rows=2,
    grid_cols=2,
    scale=1.0,
    fps=30,
    max_frames=None,
    show_frames=True
):
    """
    Create a video from images by stacking them into MxN collages.

    Parameters:
    - image_paths: list of image file paths
    - output_path: output video file path (e.g. 'output.mp4')
    - grid_rows: number of rows in collage
    - grid_cols: number of columns in collage
    - scale: downscale factor applied to final collage
    - fps: output video frames per second
    - max_frames: optional maximum number of frames to generate
    """

    images_per_frame = grid_rows * grid_cols
    total_frames_possible = len(image_paths) // images_per_frame
    
    if max_frames is not None:
        total_frames = min(total_frames_possible, max_frames)
    else:
        total_frames = total_frames_possible

    if total_frames == 0:
        raise ValueError("Not enough images to create even one frame.")

    input(f"Generate {total_frames} frames? (Yes: ENTER, No: Ctrl+C)")

    # --- Load first image to determine base size ---
    first_img = cv2.imread(image_paths[0])
    if first_img is None:
        raise ValueError(f"Could not read image: {image_paths[0]}")

    img_h, img_w = first_img.shape[:2]

    collage_h = grid_rows * img_h
    collage_w = grid_cols * img_w

    output_h = int(collage_h * scale)
    output_w = int(collage_w * scale)

    # --- Video writer ---
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (output_w, output_h))

    frame_idx = 0
    path_idx = 0

    while frame_idx < total_frames:
        tiles = []

        # Collect images for this frame
        for _ in range(images_per_frame):
            img = cv2.imread(image_paths[path_idx])
            if img is None:
                raise ValueError(f"Could not read image: {image_paths[path_idx]}")

            # Resize if needed to match first image
            if img.shape[:2] != (img_h, img_w):
                img = cv2.resize(img, (img_w, img_h))

            tiles.append(img)
            path_idx += 1

        # Stack into grid
        rows = []
        for r in range(grid_rows):
            row_imgs = tiles[r * grid_cols:(r + 1) * grid_cols]
            row_stack = np.hstack(row_imgs)
            rows.append(row_stack)

        collage = np.vstack(rows)

        # Downscale final collage
        if scale != 1.0:
            collage = cv2.resize(collage, (output_w, output_h))

        cv2.imshow("Output Frame",collage)
        cv2.waitKey(1)
        writer.write(collage)
        frame_idx += 1

    writer.release()

if __name__ == "__main__":
    # Settings
    output_root = "/home/csrobot/Videos/moadv2_grid"
    output_name = "wp_pose-b.mp4"
    OUTPUT_PATH = os.path.join(output_root,output_name)
    FPS = 24
    MAX_FRAMES = None
    GRID_DIMENSIONS = [1,1]
    OUTPUT_SCALE = 1.0
    
    # Acquire file list
    search_dir = "/home/csrobot/Desktop/atb1_conn-wp/pose-b/images_4"
    imgs1 = find_files(search_dir,".jpg",3)

    image_paths = imgs1
    image_paths = sorted(image_paths) # SORT?
    # random.shuffle(image_paths) # RANDOMIZE?

    # Generate Video
    images_to_collage_video(
        image_paths,
        OUTPUT_PATH,
        grid_rows=GRID_DIMENSIONS[1],
        grid_cols=GRID_DIMENSIONS[0],
        scale=OUTPUT_SCALE,
        fps=FPS,
        max_frames=MAX_FRAMES,
        show_frames=True
    )