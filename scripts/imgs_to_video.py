import os
from os.path import join
import cv2
from pathlib import Path

def ensure_directory_exists(directory):
    path = Path(directory)
    if path.exists():
        print(f"Directory already exists: {directory}")
    else:
        path.mkdir(parents=True, exist_ok=True)
        print(f"Created directory: {directory}")

def create_video_from_images(search_dir, recursive, output_dir, output_file, downscale_factor=1,fps=30,exclude_strings=None,max_frames=None):
    # Validate inputs
    search_dir = Path(search_dir)
    output_dir = Path(output_dir)
    if not search_dir.is_dir():
        raise ValueError(f"Search directory '{search_dir}' does not exist or is not a directory.")
    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    # Find image files
    img_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"}
    if recursive:
        images = search_dir.rglob("*") #sorted(search_dir.rglob("*"))
    else:
        images = search_dir.glob("*") #sorted(search_dir.glob("*"))

    
    images = [img for img in images if img.suffix.lower() in img_extensions]
    
    if not images:
        raise ValueError("No images found in the specified directory.")
    else:
        print(f"Images Found: {len(images)}")
        if exclude_strings is not None:
            print(f"Excluding Strings: {exclude_strings}...")
            images = [img for img in images if not any(excl in img.name for excl in exclude_strings)]
            print(f"Images After Exlcusion: {len(images)}")
        if max_frames is not None and len(images) > max_frames:
            print(f"[WARNING] Max Frames exceeded, reducing to {max_frames} frames.")
            images = images[:max_frames]

    # Read images and process them
    frames = []
    images = sorted(images)
    for img_path in images:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Warning: Could not read image {img_path}. Skipping.")
            continue
        
        # Downscale if necessary
        if downscale_factor > 1:
            new_width = img.shape[1] // downscale_factor
            new_height = img.shape[0] // downscale_factor
            img = cv2.resize(img, (int(new_width), int(new_height)), interpolation=cv2.INTER_AREA)
        
        frames.append(img)
        if len(frames) % 10 == 0:
            print(f"Loaded {len(frames)} frames.")

    if not frames:
        raise ValueError("No valid images were loaded. Ensure the files are accessible and valid image formats.")

    print(f"Frames Loaded: {len(frames)}")
    # Determine video properties
    frame_height, frame_width = frames[0].shape[:2]
    output_path = join(output_dir,output_file)

    # Create the video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Save as .mp4
    video_writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_width, frame_height))

    # Write frames to the video
    i = 1
    for frame in frames:
        video_writer.write(frame)
        if i % 25 == 0:
            print(f"Frame [{i}/{len(frames)}] written.")
        i += 1

    video_writer.release()
    print(f"Video saved successfully to {output_path}")

# Example usage
if __name__ == "__main__":
    # Define parameters
    USING_MOAD_DATA = False

    if USING_MOAD_DATA:
        root_dir = "/home/csrobot/data-mount"
        object_name = "demo_waterproof_connector"
        subfolder = "pose-a/DSLR"
        search_dir = join(root_dir,object_name,subfolder)

        recursive_search = True
        search_exclude_strings = []#["cam1","cam2","cam4","cam5"]

        output_file = f"{object_name}_DSLR_cam3.mp4"
        output_dir = join(root_dir,object_name, "media")
        ensure_directory_exists(output_dir)
        output_fps = 24
        downscale_factor = 4.0
    else:
        # NOT USING MOAD DATA
        # dataset = "solo_3"
        # search_dir = join("/home/csrobot/synth_perception/data/windex",dataset)
        search_dir = "/home/csrobot/FoundationPose/debug/track_vis"
        # search_dir = "/home/csrobot/Unity/SynthData/PoseTesting/wp_demo/x"
        # search_dir = "/home/csrobot/synth_perception/replicator_docker/output/test_018"
        # search_dir = "/home/csrobot/Pictures/eng_pose_demo_002"

        recursive_search = True
        search_exclude_strings = []#["cam1","cam2","cam4","cam5"]

        # output_dir = "/home/csrobot/synth_perception"
        # output_dir = search_dir
        output_dir = "/home/csrobot/FoundationPose/demo_data/results"
        # output_file = f"dataset_{dataset}.mp4"
        output_file = f"gear1_result.mp4"
        output_fps = 15

        downscale_factor = 1.0

    # Create Video
    max_frames = None # Cut down to specified number of frames 
    create_video_from_images(
        search_dir = search_dir,
        recursive = recursive_search,
        output_dir = output_dir,
        output_file = output_file,
        downscale_factor = downscale_factor,
        fps = output_fps,
        exclude_strings=search_exclude_strings,
        max_frames=max_frames
    )
