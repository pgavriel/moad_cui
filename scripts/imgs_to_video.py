import os
from os.path import join
import cv2
from pathlib import Path

def create_video_from_images(search_dir, recursive, output_dir, output_file, downscale_factor=1,fps=30,exclude_strings=None):
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
            print("fExcluding Strings: {exclude_strings}...")
            images = [img for img in images if not any(excl in img.name for excl in exclude_strings)]
            print(f"Images After Exlcusion: {len(images)}")

    # Read images and process them
    frames = []
    for img_path in images:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Warning: Could not read image {img_path}. Skipping.")
            continue
        
        # Downscale if necessary
        if downscale_factor > 1:
            new_width = img.shape[1] // downscale_factor
            new_height = img.shape[0] // downscale_factor
            img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
        
        frames.append(img)

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
    create_video_from_images(
        search_dir = "/home/csrobot/Unity/Data/EngineTest/solo",
        recursive = True,
        output_dir = "/home/csrobot/Unity/Data/EngineTest/solo",
        output_file = "output_video.mp4",
        downscale_factor = 1.0,
        fps = 2,
        exclude_strings=["semantic"]
    )
