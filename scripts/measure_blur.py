import os
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import re
from os.path import join

def print_elapsed_time(start_time):
    end_time = time.time()
    elapsed_time = end_time - start_time
    # Convert elapsed time to hours, minutes, and seconds
    hours, remainder = divmod(elapsed_time, 3600)
    minutes, seconds = divmod(remainder, 60)

    print(f"Script execution time: {int(hours):02}:{int(minutes):02}:{int(seconds):02}")

def detect_outliers(camera_blurriness, num_std_dev=2):
    """
    Detect positions of outliers for each camera based on blurriness data.
    Outliers are defined as points that fall outside `num_std_dev` standard deviations from the mean.

    :param camera_blurriness: Dictionary with camera names as keys and lists of blurriness scores as values.
    :param num_std_dev: Number of standard deviations outside which points are considered outliers.
    :return: Dictionary with camera names as keys and lists of outlier positions as values.
    """
    outliers = {}

    for camera_id, data in camera_blurriness.items():
        scores = np.array(data["scores"])
        indices = np.array(data["indices"])

        mean_score = np.mean(scores)
        std_dev = np.std(scores)

        # Identify outliers
        lower_bound = mean_score - num_std_dev * std_dev
        upper_bound = mean_score + num_std_dev * std_dev
        outlier_mask = (scores < lower_bound) | (scores > upper_bound)
        outliers[camera_id] = indices[outlier_mask].tolist()

    return outliers

def write_outliers_to_file(outliers, metric_used, object_name, output_file):
    """
    Write outlier information to a text file, including total count, metric used, 
    and outliers per camera.

    :param outliers: Dictionary with camera names as keys and lists of outlier positions as values.
    :param metric_used: String describing the metric used to calculate blurriness.
    :param output_file: Path to the output text file.
    """
    total_outliers = sum(len(positions) for positions in outliers.values())

    with open(output_file, "w") as file:
        # Write header with metric used and total count
        file.write("AUTOMATIC BLURRED IMAGE DETECTION\n")
        file.write(f"Object: {object_name}\n")
        file.write(f"Metric used: {metric_used}\n")
        file.write(f"Total outliers: {total_outliers}\n\n")

        # Write outlier count per camera
        file.write("Outliers per camera:\n")
        for camera, positions in outliers.items():
            file.write(f"{camera}: {len(positions)}\n")
        file.write("\n")

        # Write detailed outliers for each camera
        file.write("Outlier positions:\n")
        for camera, positions in outliers.items():
            file.write(f"{camera}: {positions}\n")

def measure_blurriness(image):
    """
    Measure the blurriness of an image using the variance of the Laplacian.
    :param image_path: Path to the image file.
    :param scale_factor: Factor to downscale the image before computing blurriness.
    :return: Blurriness score (higher values indicate sharper images).
    """
    laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
    return laplacian_var

def directional_variance(image):
    grad_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)  # Horizontal gradient
    grad_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)  # Vertical gradient
    grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
    directionality = np.std(grad_x) - np.std(grad_y)  # Compare variances
    return np.mean(grad_magnitude), abs(directionality)

def frequency_sharpness(image):
    f_transform = np.fft.fft2(image)
    f_shift = np.fft.fftshift(f_transform)
    magnitude_spectrum = 20 * np.log(np.abs(f_shift))
    high_freq_energy = np.mean(magnitude_spectrum[-10:])  # Mean of high-frequency energy
    return high_freq_energy

def brisque_score(image):
    score = cv2.quality.QualityBRISQUE_compute(image, "brisque_model.yml", "brisque_range.yml")[0]
    return score

def extract_index_from_filename(file_name):
    """
    Extract the middle number from filenames like 'cam1_015_img.jpg'.
    :param file_name: The image file name.
    :return: Extracted index as an integer.
    """
    match = re.search(r'cam\d+_(\d+)_', file_name)
    if match:
        return int(match.group(1))
    return None

def analyze_blurriness_by_camera(folder_path, measure, scale_factor=1.0,verbose=True):
    """
    Analyze blurriness for images grouped by camera and indexed by the middle number in filenames.
    :param folder_path: Path to the folder containing images.
    :param scale_factor: Factor to downscale images before blurriness measurement.
    :return: Dictionary with camera names as keys and tuples of (indices, blurriness scores) as values.
    """
    camera_blurriness = {}

    for file_name in sorted(os.listdir(folder_path)):
        if file_name.startswith("cam") and "_" in file_name:
            camera_id = file_name.split("_")[0]  # Extract camera ID, e.g., "cam1"
            index = extract_index_from_filename(file_name)
            if index is None:
                continue

            image_path = os.path.join(folder_path, file_name)
            image = cv2.imread(image_path)
            if image is None:
                raise ValueError(f"Cannot read image at {image_path}")
            if scale_factor < 1.0:
                image = cv2.resize(image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
            # Measure blurriness
            if measure == "directional":
                blurriness_score, _ = directional_variance(image)
            elif measure == "frequency":
                blurriness_score = frequency_sharpness(image)
            elif measure == "brisque":
                blurriness_score = brisque_score(image)
            else:
                image = cv2.cvtColor(image,cv2.BGR2GRAY)
                blurriness_score = measure_blurriness(image)
            if verbose: print(f"{camera_id} - {index} - {blurriness_score}")
            # Append to the corresponding camera's list
            if camera_id not in camera_blurriness:
                camera_blurriness[camera_id] = {"indices": [], "scores": []}
            camera_blurriness[camera_id]["indices"].append(index)
            camera_blurriness[camera_id]["scores"].append(blurriness_score)

    # Sort indices and scores for each camera
    for camera_id in camera_blurriness:
        indices, scores = zip(*sorted(zip(camera_blurriness[camera_id]["indices"], camera_blurriness[camera_id]["scores"])))
        camera_blurriness[camera_id]["indices"] = indices
        camera_blurriness[camera_id]["scores"] = scores

    return camera_blurriness

def plot_blurriness(camera_blurriness,object=""):
    """
    Plot blurriness scores for each camera, including average blurriness lines.
    :param camera_blurriness: Dictionary of blurriness scores grouped by camera.
    """
    plt.figure(figsize=(12, 7))
    for camera_id, data in camera_blurriness.items():
        indices = data["indices"]
        scores = data["scores"]
        average_blurriness = np.mean(scores)
        
        # Plot the blurriness data
        plt.plot(indices, scores, label=camera_id)
        
        # Plot the average blurriness as a horizontal dotted line
        plt.axhline(y=average_blurriness, color=plt.gca().lines[-1].get_color(), linestyle='dotted')

    plt.title(f"{object} Blurriness Scores by Camera")
    plt.xlabel("Image Index")
    plt.ylabel("Blurriness Score (Frequency Domain Analysis)")
    plt.legend(loc='upper right')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(np.arange(0, 411, 30))  # Tick marks 10 apart
    plt.minorticks_on()
    plt.xlim(0, 410)  # X-axis range
    plt.tight_layout()
    plt.show()

# Example usage:
start_time = time.time()
root_dir = "E:/MOAD"  # Replace with the path to your folder
object_name = "a2-engine-bot"
folder_path = join(root_dir,object_name,"DSLR")
measure = ["directional","frequency","brisque","laplacian"][0] # Change index to select mode
scale_factor = 0.5  # Downscale images to 50% of their original size for faster computation
camera_blurriness = analyze_blurriness_by_camera(folder_path,measure,scale_factor=scale_factor)
outliers = detect_outliers(camera_blurriness)
print(f"Outliers Found:\n{outliers}")
outlier_file = join(root_dir,object_name,"outlier_images.txt")
write_outliers_to_file(outliers,measure,object_name,outlier_file)
print_elapsed_time(start_time)
plot_blurriness(camera_blurriness,object_name)
