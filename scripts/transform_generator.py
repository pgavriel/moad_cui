import os
import argparse
import datetime
from os.path import join
import json
import numpy as np
import transforms3d
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

def list_folders_by_creation_date(directory, target_date):
    """
    List folder names within the specified directory created on or after a specific date.

    :param directory: The directory to scan for folders.
    :param target_date: The date (datetime.date) to filter by.
    :return: A list of folder names.
    """
    folder_names = []
    for entry in os.scandir(directory):
        if entry.is_dir():
            # Get creation time in seconds since epoch
            creation_time = os.path.getctime(entry.path)
            creation_date = datetime.date.fromtimestamp(creation_time)
            if creation_date >= target_date:
                folder_names.append(entry.name)
    return sorted(folder_names)

def get_folders_with_prefix(root_dir, prefix):
    """
    Returns a list of folders in the root directory that start with the given prefix.

    Args:
        root_dir (str): The root directory to search (non-recursive).
        prefix (str): The prefix to filter folder names.

    Returns:
        list: A list of folder names that start with the specified prefix.
    """
    try:
        # List all entries in the root directory
        entries = os.listdir(root_dir)
        
        # Filter entries to include only directories that start with the prefix
        folders = [
            entry for entry in entries 
            if os.path.isdir(os.path.join(root_dir, entry)) and entry.startswith(prefix)
        ]
        
        return sorted(folders)
    
    except FileNotFoundError:
        print(f"Error: The directory '{root_dir}' does not exist.")
        return []
    except PermissionError:
        print(f"Error: Permission denied to access '{root_dir}'.")
        return []
    
# Function to load a JSON file and return its contents as a dictionary
def load_json(file_path):
    print(f"Loading JSON: {file_path}")
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def load_align_txt(file_path):
    print(f"Loading Alignment Textfile: {file_path}")
    data = np.loadtxt(file_path, dtype=float, comments='#')
    return data

def print_array(array, name=None):
    if name is not None:
        print(name)
    for row in array:
        print(row)

def get_zrot_matrix(degrees):
    # Convert the angle to radians
    angle_radians = np.radians(degrees)

    # Create a rotation matrix around the z-axis
    # NOTE: Direction is inverted by default to match the turntables movement
    rotation_matrix = transforms3d.axangles.axangle2mat((0, 0, 1), -angle_radians)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    print(f"Rotation Matrix for {degrees} degrees:")
    print(transform_matrix)
    return transform_matrix


class MoadTransformGenerator:
    def __init__(self,object_name="Test"):
        self.output_dir = "E:/MOAD"
        self.object_name = object_name

        self.calibration_dir = "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/calibration"
        self.modes = ['18mm','55mm'] #TODO: Add 24mm and 35mm?
        self.mode = self.modes[0]

        self.scan_angle_inc = 45 # 5
        self.scan_range = 360
        # self.scan_init_angle = 0
        self.num_cameras = 5

        self.exclude_cams = []
        self.exclude_frames = None

        self.apply_alignment = True
        self.auto_save = True
        self.visualize = True

        self.camera_tfs = None
        self.align_tf = None
        self.transform_json = None

    def load_transforms(self,camera_file="transforms.json",align_file="alignment_tf.txt"):
        print(f"Loading Camera Transforms...")
        print(f"Calibration Mode: {self.mode}")
        # Load Camera Transforms
        tf_path = join(self.calibration_dir,self.mode,camera_file)
        self.transform_json = load_json(tf_path)

        frames = self.transform_json["frames"]
        print(f"Frames defined: {len(frames)}")

        # Get individual transform frame for each camera
        self.camera_tfs = dict()
        for f in frames:
            id = f["file_path"].split("/")
            id = id[-1].split("_")[0]
            print(f"ID: {id}")
            self.camera_tfs[id] = f["transform_matrix"]
            print_array(self.camera_tfs[id])

        # Clear the frames to be rewritten, preserving camera intrinsics
        self.transform_json["frames"] = []
        print("Base Transform JSON")
        for x in self.transform_json:
            print(f"{x} = {self.transform_json[x]}")


        # Load Alignment Transform
        print("Loading Alignment Transforms...")
        align_path = join(self.calibration_dir,self.mode,align_file)
        self.align_tf = load_align_txt(align_path)
        print_array(self.align_tf,"Alignment Array")

    
    def calculate_transforms(self):
        print("Calculating Transforms...")
        
        # Assemble list of camera transforms
        tf1 = self.camera_tfs["cam1"]
        tf2 = self.camera_tfs["cam2"]
        tf3 = self.camera_tfs["cam3"]
        tf4 = self.camera_tfs["cam4"]
        tf5 = self.camera_tfs["cam5"]
        transforms = [tf1, tf2, tf3, tf4, tf5]

        # Define the alignment transform
        if self.apply_alignment:
            print("Applying Alignment TF")
            alignment_tf = self.align_tf
        else:
            alignment_tf = np.eye(4)

        # Apply the alignment transform to each camera transform
        transformed_matrices = [np.dot(alignment_tf, matrix) for matrix in transforms]

        # Calculate all transforms and json frames list
        frame_count = 1 
        frames = []
        frames_per_camera = self.scan_range // self.scan_angle_inc
        print(f"Cameras: {self.num_cameras}")
        print(f"Angle Inc: {self.scan_angle_inc}")
        print(f"Frames Per Camera: {frames_per_camera}")
        
        for i in range(self.num_cameras):
            current_camera = i+1
            current_position = 0
            for j in range(frames_per_camera):
                base_camera_tf = transformed_matrices[i]
                rotation_matrix = get_zrot_matrix(current_position)
                camera_tf = np.dot(rotation_matrix, base_camera_tf)
                frame = {
                    "file_path": f"images/frame_{frame_count:05d}.jpg",
                    "camera_id": current_camera,
                    "position_deg": current_position,
                    "transform_matrix": camera_tf.tolist()
                }
                # TESTING EXCLUDING CAMERA AND FRAMES
                if current_camera not in self.exclude_cams:
                    if self.exclude_frames is None or current_position not in self.exclude_frames[current_camera]:
                        frames.append(frame)
                frame_count += 1
                current_position += self.scan_angle_inc

        self.transform_json["frames"] = frames
        print(f"Frames Generated: {len(frames)}")
        if self.auto_save:
            self.save_json()

        if self.visualize:
            self.visualize_frames(transformed_matrices,frames)

    def visualize_frames(self,tfs,frames,show_origin=False):
        print("Visualizing Camera Transforms...")
        # Determine bounds 
        transformed_points = []
        for tf in tfs:
            point = np.dot(tf, np.array([0, 0, 0, 1]))
            transformed_points.append(point)
        min_val = np.min(np.asarray(transformed_points))
        max_val = np.max(np.asarray(transformed_points))
        # print(f"Min: {min_val}")
        # print(f"Max: {max_val}")

        # Visualize
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Set the range for all axes
        ax.set_xlim(min_val, max_val)
        ax.set_ylim(min_val, max_val)
        ax.set_zlim(-0.05, max_val)
        # ax.set_zlim(min_val, max_val)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Camera Transforms\nCalibration: {self.mode}')
        c = 1
        for f in frames:
            point = np.dot(f["transform_matrix"], np.array([0, 0, 0, 1]))
            ax.scatter(point[0], point[1], point[2])
            if len(frames) <= 5:
                ax.text(point[0], point[1], point[2], f"cam{c}", color='black')
            # print(f'Point {c}: {point}')
            c += 1

        if show_origin:
            ax.scatter(0, 0, 0,c="#000000")
            ax.scatter(1, 0, 0,c="#ff0000")
            ax.scatter(0, 1, 0,c="#00ff00")
            ax.scatter(0, 0, 1,c="#0000ff")

        plt.show()

    def batch_generate(self,object_list,excl_cams=[],excl_frames=None):
        vis_temp = self.visualize
        self.visualize = False
        self.auto_save = True
        
        self.load_transforms()
        c = 1
        for obj in object_list:
            print(f"Generating transforms for {obj}...")
            self.object_name = obj
            if c == len(object_list):
                self.visualize = vis_temp
            self.calculate_transforms()
            c += 1 


    def save_json(self,out_dir=None,out_file="transforms.json"):
        if out_dir is None:
            out_dir = join(self.output_dir,self.object_name)
        
        print(f"Writing {out_file} to {out_dir}...")
        with open(join(out_dir,out_file), "w") as f:
            json.dump(self.transform_json, f, indent=4)  # Use indent for human-readable formatting
        print("Done.")


# Get CLI arguments
parser = argparse.ArgumentParser()
parser.add_argument('object_name')
parser.add_argument('-d', '--degree', type=int, default=5, help="Degree difference between each image (Default: 5)")
parser.add_argument('-v', '--visualize', action="store_true", help="Flag: Visualize the 3D position of the camera")

args = parser.parse_args()
# arg_list = sys.argv[1:]
# if len(arg_list) < 1:
#     print("Args Error: Missing object_name")
#     print("     python3 scripts/transform_generator.py [object_name]")
#     exit() 

tf_gen = MoadTransformGenerator()
# Set the directory containing calibrations and the calibration (subfolder) to use.
tf_gen.calibration_dir = "/home/csrobot/moad_cui/calibration"
tf_gen.mode = '55mm'
# Set the directory containing object data and the object (subfoler) to write to.
tf_gen.output_dir = "/home/csrobot/ns-data"
tf_gen.object_name = args.object_name #"t1_zoomcan"
# Set the angle increment of the collected image data.
tf_gen.scan_angle_inc = args.degree
tf_gen.visualize = args.visualize
tf_gen.auto_save = True
# tf_gen.load_transforms()
# tf_gen.calculate_transforms()

# Camera IDs to exclude entirely
tf_gen.exclude_cameras = [] # [1, 2, 3, 4, 5]
# Specific frames to exclude
tf_gen.exclude_frames = {
    1: [],
    2: [],
    3: [],
    4: [],
    5: []
}
excluded_frames = sum(len(positions) for positions in tf_gen.exclude_frames.values())

generate_modes = ["name", "date", "prefix"]
mode = 0

if generate_modes[mode] == "name":
    # SET OBJECT NAMES
    obj_list = [args.object_name]
    
elif generate_modes[mode] == "date":
    # SET DATE, RETURN ALL FOLDERS CREATED DURING OR AFTER
    target_date = datetime.date(2024, 12, 4)
    obj_list = list_folders_by_creation_date(tf_gen.output_dir,target_date)
    
elif generate_modes[mode] == "prefix":
    # SET PREFIX, RETURN ALL FOLDERS STARTING WITH PREFIX
    prefix = "robot"
    obj_list = get_folders_with_prefix(tf_gen.output_dir,prefix)


if len(tf_gen.exclude_cameras) != 0: 
    print(f"EXCLUDING CAMERAS {tf_gen.exclude_cameras}")
else:
    print("No cameras excluded.")
if excluded_frames != 0: 
    print(f"EXCLUDING {excluded_frames} FRAMES")
else:
    print("No frames being excluded.")
print(f"Calibration Folder: {join(tf_gen.calibration_dir,tf_gen.mode)}")
print(f"Object List: {obj_list}")
input("Continue? (Ctrl+C to cancel)")
tf_gen.batch_generate(obj_list)

print("Done.")