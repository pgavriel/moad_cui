import os
from os.path import join
import json
import numpy as np
import transforms3d
import matplotlib.pyplot as plt

# Function to load a JSON file and return its contents as a dictionary
def load_json(file_path):
    print(f"Loading JSON: {file_path}")
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def load_align_txt(file_path):
    print(f"Loading Alignment Textfile: {file_path}")
    data = np.loadtxt(file_path, dtype=float)
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
                frames.append(frame)
                frame_count += 1
                current_position += self.scan_angle_inc

        self.transform_json["frames"] = frames
        
        if self.auto_save:
            self.save_json()

        if self.visualize:
            self.visualize_frames(transformed_matrices,frames)

    def visualize_frames(self,tfs,frames,show_origin=True):
        print("Visualizing Camera Transforms...")
        # Determine bounds 
        transformed_points = []
        for tf in tfs:
            point = np.dot(tf, np.array([0, 0, 0, 1]))
            transformed_points.append(point)
        min_val = np.min(np.asarray(transformed_points))
        max_val = np.max(np.asarray(transformed_points))
        print(f"Min: {min_val}")
        print(f"Max: {max_val}")

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
        ax.set_title('Transformed Points')
        c = 1
        for f in frames:
            point = np.dot(f["transform_matrix"], np.array([0, 0, 0, 1]))
            ax.scatter(point[0], point[1], point[2])
            print(f'Point {c}: {point}')
            c += 1

        if show_origin:
            ax.scatter(0, 0, 0,c="#000000")
            ax.scatter(1, 0, 0,c="#ff0000")
            ax.scatter(0, 1, 0,c="#00ff00")
            ax.scatter(0, 0, 1,c="#0000ff")

        plt.show()

    def save_json(self,out_dir=None,out_file="transforms.json"):
        if out_dir is None:
            out_dir = join(self.output_dir,self.object_name)
        
        print(f"Writing {out_file} to {out_dir}...")
        with open(join(out_dir,out_file), "w") as f:
            json.dump(self.transform_json, f, indent=4)  # Use indent for human-readable formatting
        print("Done.")


tf_gen = MoadTransformGenerator()
tf_gen.mode = '18mm'
tf_gen.object_name = "t3_plane_bot"
tf_gen.visualize = True
tf_gen.auto_save = False
tf_gen.load_transforms()
tf_gen.calculate_transforms()
print("Done.")