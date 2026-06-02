'''
UPDATED 5/28/26 TO GENERATE CAMERA TRANSFORMS USING THE NEW CALIBRATION FILE FORMAT (cam_parameters.json)

This script generates a json file (which is needed for NeRF reconstruction training)
that contains all of the "virtual" camera poses for a scan collected by the MOAD rig.

The required inputs are:
 - output folder: the object folder the transforms are being generated for
 - calibration: The calibration which corresponds to the camera positions/zoom at the time of data collection

'''

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

def get_zrot_matrix(degrees, verbose=False):
    # Convert the angle to radians
    angle_radians = np.radians(degrees)

    # Create a rotation matrix around the z-axis
    # NOTE: Direction is inverted by default to match the turntables movement
    rotation_matrix = transforms3d.axangles.axangle2mat((0, 0, 1), -angle_radians)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    if verbose: print(f"Rotation Matrix for {degrees} degrees:")
    if verbose: print(transform_matrix)
    return transform_matrix

def opencv_to_nerf(c2w: list[list[float]]) -> list[list[float]]:
    """
    Convert a camera-to-world matrix from OpenCV convention
    (X right, Y down, Z forward) to NeRF/OpenGL convention
    (X right, Y up, Z backward).
    """
    m = np.array(c2w)
    m[:3, 1] *= -1   # flip Y
    m[:3, 2] *= -1   # flip Z
    return m.tolist()

class MoadTransformGenerator:
    def __init__(self,object_name="Test"):
        # Default parameters are overwritten by command line arguments in main
        self.output_dir = "E:/MOAD"
        self.object_name = object_name
        self.pose = "pose-a"

        self.calibration_dir = "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/calibration"
        self.calibration = '18mm'

        self.scan_angle_inc = 5
        self.scan_range = 360
        self.total_frames = 360
        # self.scan_init_angle = 0
        self.num_cameras = 5

        self.exclude_cameras = []
        self.exclude_frames = None

        # self.apply_alignment = False
        self.auto_save = True
        self.visualize = True

        self.camera_tfs = None
        # self.align_tf = None
        self.transform_json = None


    def create_output_transforms_template(self):
        '''
        Create the initial template for the transforms.json output file required for NeRF reconstruction.
        '''
        # TODO: Test if applied transform and ply path are necessary to include (currently just copying Nerfstudio template)
        # Fixed fields that are constant across all transforms.json files in this rig.
        # APPLIED_TRANSFORM = [
        #     [1.0,  0.0,  0.0, 0.0],
        #     [0.0,  0.0,  1.0, 0.0],
        #     [-0.0, -1.0, -0.0, -0.0],
        # ]
        # APPLIED_TRANSFORM = [
        #     [1.0, 0.0, 0.0, 0.0],
        #     [0.0, 1.0, 0.0, 0.0],
        #     [0.0, 0.0, 1.0, 0.0],
        # ]
        # PLY_FILE_PATH = "sparse_pc.ply"

        # Get shared intrinsics from loaded cam_parameters.json
        intr = self.calibration_dict["intrinsics"]
        dist = intr["distortion"]

        # Setup initial template for output file; frames will be calculated later.
        self.transform_json = {
            "w":            intr["width"],
            "h":            intr["height"],
            "fl_x":         intr["fx"],
            "fl_y":         intr["fy"],
            "cx":           intr["cx"],
            "cy":           intr["cy"],
            "k1":           dist.get("k1", 0.0),
            "k2":           dist.get("k2", 0.0),
            "p1":           dist.get("p1", 0.0),
            "p2":           dist.get("p2", 0.0),
            "camera_model": "OPENCV",
            "frames":       []
        }
            # "applied_transform": APPLIED_TRANSFORM,
            # "ply_file_path":     PLY_FILE_PATH,
            # }

    def load_calibration(self, calibration_file="cam_parameters.json"):
        print(f"Loading Camera Calibration...")
        print(f"Calibration Folder: {self.calibration}")
        # Load Camera Calibration data
        calibration_path = join(self.calibration_dir,self.calibration,calibration_file)
        self.calibration_dict = load_json(calibration_path)

        # Create template dictionary populated with shared intrinsics
        self.create_output_transforms_template() 
        
    
    def calculate_transforms(self):
        print("Calculating Transforms...")
        
        # Assemble list of camera transforms
        cameras = self.calibration_dict["cameras"]
        self.num_cameras = len(cameras)
        self.camera_keys = sorted(self.calibration_dict["cameras"].keys())
        print(f"Found Camera Keys: {self.camera_keys}")

        # Get the base transform for each camera
        transforms = []
        for cam in self.camera_keys:
            transforms.append(cameras[cam]["extrinsics"]["c2w"])
            
        # Generate the list of rotation positions to calculate transforms for 
        # NOTE: Expects to work only with INTEGER degree angles
        position_list = []
        for i in range(self.scan_range):
            if i % self.scan_angle_inc == 0:
               position_list.append(i)

        # Print some state information (mostly for debug)
        print(f"Cameras: {self.num_cameras}")
        print(f"Angle Inc: {self.scan_angle_inc}")
        print(f"Frames Per Camera: {len(position_list)}")
        print(f"Positions: {position_list}")
        print(f"Total Frames: {self.total_frames}")
        
        def GetFrameNum(cam_idx, pos, total):
            '''
            Helper function to convert from a camera and position to a frame number (i.e. cam1_000_img.jpg -> frame_00001.jpg)
            This is dictated by the way NerfStudio creates numbered frames when downscaling the source images, 
            which renames the source images in alphabetical order (cam1 first; all positions in ascending order, then cam2, etc.).
            '''
            frame_per_cam = total // self.num_cameras
            frame_per_degree = self.scan_range // frame_per_cam
            frame_num = 1 + ((cam_idx-1) * frame_per_cam) + (pos // frame_per_degree)
            # print(f"ID:{cam_idx}\tPOS:{pos}\t`FPC:{frame_per_cam}\tFPD{frame_per_degree}\tFN:{frame_num}")
            return frame_num
        
        # Calculate all transforms and json frames list
        frames = []

        # For each camera...
        for i in range(self.num_cameras):
            current_camera = i+1
            if current_camera in self.exclude_cameras: continue # Skip an excluded camera
            # Generate a frame for each position...
            for pos in position_list:
                base_camera_tf = transforms[i]
                rotation_matrix = get_zrot_matrix(pos)
                camera_tf = np.dot(rotation_matrix, base_camera_tf)
                frame_num = GetFrameNum(current_camera,pos,self.total_frames)
                # Assemble frame data
                frame = {
                    "file_path": f"images/frame_{frame_num:05d}.jpg",
                    "camera_id": current_camera,
                    "position_deg": pos,
                    "transform_matrix": opencv_to_nerf(camera_tf.tolist())
                }

                # Only append frames to the final list which are not being excluded
                if current_camera not in self.exclude_cameras:
                    if self.exclude_frames is None or pos not in self.exclude_frames[current_camera]:
                        frames.append(frame)
                # frame_count += 1
                # current_position += self.scan_angle_inc

        # Add all calculated frames to output json
        self.transform_json["frames"] = frames
        print(f"Frames Generated: {len(frames)}\n")


    def visualize_frames(self,show_origin=False):
        print("Visualizing Camera Transforms...")
        np.set_printoptions(precision=2, suppress=True, floatmode="fixed")
        # Determine bounds 
        frames = self.transform_json["frames"]
        transformed_points = []
        for frame in frames:
            tf = np.asarray(frame["transform_matrix"]).reshape((4,4))
            point = np.dot(tf, np.array([0, 0, 0, 1]))
            # print(f"point: {point}")
            transformed_points.append(point)
        transformed_points = np.asarray(transformed_points)
        max_z  = np.max(transformed_points[:, 2])
        max_xy = np.max(np.abs(transformed_points[:, 0:2]))
        max_xy = max(1,max_xy)
        # print(f"Max Z: {max_z:.2f}")
        # print(f"Max XY: {max_xy:.2f}")

        # Visualize
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Set the range for all axes
        ax.set_xlim(-max_xy, max_xy)
        ax.set_ylim(-max_xy, max_xy)
        ax.set_zlim(-0.05, max_z)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Camera Transforms\nCalibration: {self.calibration}')
        c = 1
        for p in transformed_points:
            ax.scatter(p[0], p[1], p[2])
            if len(frames) <= 5:
                ax.text(p[0], p[1], p[2], f"cam{c}", color='black')
            # print(f'Point {c}: {point}')
            c += 1
    
        if show_origin:
            ax.scatter(0, 0, 0,c="#000000")
            ax.scatter(1, 0, 0,c="#ff0000")
            ax.scatter(0, 1, 0,c="#00ff00")
            ax.scatter(0, 0, 1,c="#0000ff")

        plt.show()

    def batch_generate(self,object_list):
        '''
        Generate all camera transforms and save them to all specified object folders
        1. Loads the specified calibration, 
        2. Generate all the transforms according to  
        3. Saves a copy to each specified object in object_list
        '''
        self.load_calibration() # Load cam_params.json (All calibration information), and create output template
        self.calculate_transforms() # Generate all transform frames, and add them to the template
        
        # Save a copy to each specified object
        for obj in object_list:
            print(f"Saving transforms for: {obj}...")
            self.object_name = obj
            self.save_json()
            
        # Visualize transforms
        if self.visualize:
            self.visualize_frames()

    def save_json(self,out_dir=None,out_file="transforms.json"):
        if out_dir is None:
            out_dir = join(self.output_dir,self.object_name, self.pose)
        
        print(f"Writing \'{out_file}\' to \'{out_dir}\'...")
        with open(join(out_dir,out_file), "w") as f:
            json.dump(self.transform_json, f, indent=4)  # Use indent for human-readable formatting
        print("Done.")


# Set some default values here for convenience
DEFAULT_DATA_DIR = "/home/csrobot/MOAD_DATA"
DEFAULT_CALIBRATION_DIR = "/home/csrobot/moad_control/moad_cui/calibration"
DEFAULT_CALIBRATION = "55mm"
# Get CLI arguments
parser = argparse.ArgumentParser()
parser.add_argument('object_name', help="Name of the scanned object")
parser.add_argument('-p', '--path', type=str, default=DEFAULT_DATA_DIR, help="Directory where the object is located")
parser.add_argument('--pose', type=str, default="pose-a", help="The target pose of the scan (Pose folder name, Default: pose-a)")
parser.add_argument('-d', '--degree', type=int, default=5, help="Degree difference between each image (Default: 5)")
parser.add_argument('-r', '--range', type=int, default=360, help="Max angle of rotation (Default: 360)")
parser.add_argument('-t', '--totalframes', type=int, default=360, help="Total images collected in the target scan (Default: 360)")
parser.add_argument('-c', '--calibration', type=str, default=DEFAULT_CALIBRATION, help="Calibration used for object data collection (Calibration folder name)")
parser.add_argument('--calibration_dir', type=str, default=DEFAULT_CALIBRATION_DIR, help="Directory where the calibration files are")
parser.add_argument('-v', '--visualize', action="store_true", help="Flag: Visualize the 3D position of the camera")
parser.add_argument('-f', '--force', action="store_true", help="Bypass the confirmation prompt before calculating the transform position")

args = parser.parse_args()
# Print values nicely in an aligned table
print("\n" + "="*30 + "\n   COMMAND LINE ARGUMENTS\n" + "="*30)
for key, value in vars(args).items():
    print(f"{key:<15} : {value}")
print("="*30 + "\n")

tf_gen = MoadTransformGenerator()
# Set the directory containing calibrations and the calibration (subfolder) to use.
tf_gen.calibration_dir = args.calibration_dir
tf_gen.calibration = args.calibration
# Set the directory containing object data and the object (subfoler) to write to.
tf_gen.output_dir = args.path
tf_gen.object_name = args.object_name #"t1_zoomcan"
tf_gen.pose = args.pose
# Set the angle increment of the collected image data.
tf_gen.scan_range = args.range
tf_gen.total_frames = args.totalframes
tf_gen.scan_angle_inc = args.degree
tf_gen.visualize = args.visualize
tf_gen.auto_save = True


# CAMERA/FRAME EXCLUSION (FOR TESTING) =========================================================
#   You can manually exclude specific cameras/frames from the generated transforms
#   This was useful for some experiments but generally doesn't need to be used.
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
# Count total excluded frames
excluded_frames = sum(len(positions) for positions in tf_gen.exclude_frames.values())

# GENERATION MODE (FOR TESTING) =================================================================
#   By default, generation mode is set to generate a single file, based on a single provided object name.
#   Alternatively, you can manually change 'mode' to generate a batch of transform files for all objects
#   created after a specified date, or by matching an object name prefix.
#   Again, this is generally not needed.
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

# Print out some information before generating
if len(tf_gen.exclude_cameras) != 0: 
    print(f"EXCLUDING CAMERAS {tf_gen.exclude_cameras}")
else:
    print("No cameras excluded.")
if excluded_frames != 0: 
    print(f"EXCLUDING {excluded_frames} FRAMES")
else:
    print("No frames being excluded.")
print(f"Calibration Folder: {join(tf_gen.calibration_dir,tf_gen.calibration)}")
print(f"Object List: {obj_list}")

# Await confirmation before generating
if not args.force:
    input("Continue? (Ctrl+C to cancel)")

# Generate transform file(s)
tf_gen.batch_generate(obj_list)

print("Done.")