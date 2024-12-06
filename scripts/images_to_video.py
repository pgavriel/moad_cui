import cv2
import os

scale_percent = 25
# Directory containing the input images
base_dir = 'D:/MOAD'
object_name = "nist_atb1_conn-bnc"
input_dir = os.path.join(base_dir,object_name,"DSLR")
print(f"Input Dir: {input_dir}")
# Output directory for the video file
output_dir = os.path.dirname(input_dir)
print(f"Output Dir: {output_dir}")

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

# List the image files in the input directory and sort them by name
image_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.jpg')])
num_images = len(image_files)
print(f"{num_images} images found.")

# Get the first image to determine the video frame size
first_image = cv2.imread(os.path.join(input_dir, image_files[0]))
frame_height, frame_width, _ = first_image.shape
print(f"Original Dimensions: {frame_width} x {frame_height}")
width = int(first_image.shape[1] * scale_percent / 100)
height = int(first_image.shape[0] * scale_percent / 100)
print(f"Scaled Dimensions: {width} x {height}")

# input("Continue?")

# Define the output video file name and settings
output_file = os.path.join(output_dir, f'{object_name}-video.mp4')
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use appropriate codec for MP4

# Create the VideoWriter object
out = cv2.VideoWriter(output_file, fourcc, 30, (width, height))

# Loop through the image files and add them to the video
i = 1
for image_file in image_files:
    if i%10 == 0: 
        print(f"{i}/{num_images} done")
    image_path = os.path.join(input_dir, image_file)
    frame = cv2.imread(image_path)
    scaled_image = cv2.resize(frame, (width, height))
    out.write(scaled_image)
    i += 1

# Release the VideoWriter
out.release()

print(f'Video saved to: {output_file}')
