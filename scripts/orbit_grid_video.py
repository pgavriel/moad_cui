import math
import numpy as np
import os
import re
import open3d as o3d
from pathlib import Path
import cv2
import sys
import random

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

def load_meshes(file_list):
    mesh_list = []
    for f in file_list:
        print(f" > Loading \'{f}\'...")
        mesh_list.append(o3d.io.read_triangle_mesh(f))
        # mesh_list.append(o3d.io.read_point_cloud(f)) # For pointclouds

    return mesh_list

def compute_grid_dims(n, aspect=16/9):
    cols = math.sqrt(n * aspect)
    rows = n / cols
    if math.floor(rows) * math.floor(cols) >= n:
        cols = math.floor(cols)
        rows = math.floor(rows)
    elif math.floor(rows) * math.ceil(cols) >= n:
        cols = math.ceil(cols)
        rows = math.floor(rows)
    elif math.ceil(rows) * math.floor(cols) >= n:
        cols = math.floor(cols)
        rows = math.ceil(rows)
    else:
        cols = math.ceil(cols)
        rows = math.ceil(rows)
    print(f"Found Dimensions: {rows} rows x {cols} cols for N={n}, A={rows*cols}")
    return rows, cols

def generate_grid_positions(n, rows, cols, spacing=1.0):

    positions = []

    x_offset = (cols - 1) / 2
    y_offset = (rows - 1) / 2

    idx = 0
    for r in range(rows):
        for c in range(cols):
            if idx >= n:
                break

            x = (c - x_offset) * spacing    # flip so positive is upward
            # x = (x_offset - c) * spacing    # flip so positive is upward
            y = (r - y_offset) * spacing   # flip so positive is upward
            # y = (y_offset - r) * spacing   # flip so positive is upward
            z = 0.0

            positions.append(np.array([x, y, z]))
            idx += 1

    print(f"Generated {len(positions)} positions.")
    return positions

def generate_rotation_matrices(n_frames, axis=(0, 0.1, 0)):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)

    rotations = []

    for i in range(n_frames):
        theta = 2 * np.pi * i / n_frames   # full loop

        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])

        R = (
            np.eye(3) +
            np.sin(theta) * K +
            (1 - np.cos(theta)) * (K @ K)
        )

        rotations.append(R)

    return rotations

def incremental_rotation(axis=(1,1,0.5), degrees_per_frame=1.0):
    axis = np.array(axis, float)
    axis /= np.linalg.norm(axis)

    theta = np.deg2rad(degrees_per_frame)

    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    R = np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)
    return R


def axis_angle_rotation(axis=[1,0,0], degrees=90):
    ''' Expects axis = [x, y, z] ratio'''
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)

    theta = np.deg2rad(degrees)

    x, y, z = axis

    K = np.array([
        [0, -z,  y],
        [z,  0, -x],
        [-y, x,  0]
    ])

    R = (
        np.eye(3)
        + np.sin(theta) * K
        + (1 - np.cos(theta)) * (K @ K)
    )

    return R

def pad_to_size(img, target_width, target_height, color=(0, 0, 0)):
    """
    Pad img (H,W,3) to exactly target_height x target_width with a solid color.

    img: numpy array, dtype=uint8
    color: (B,G,R) tuple for OpenCV
    """
    h, w, _ = img.shape

    pad_top = (target_height - h) // 2
    pad_bottom = target_height - h - pad_top
    pad_left = (target_width - w) // 2
    pad_right = target_width - w - pad_left

    padded = np.full((target_height, target_width, 3), color, dtype=np.uint8)
    padded[pad_top:pad_top+h, pad_left:pad_left+w, :] = img

    return padded


running = True
def on_quit(vis):
    global running
    running = False
    print("[Q] Quitting.")
    return False
def animate_scene(
    meshes,
    positions,
    rotation,
    cam_z_start=3.0,
    cam_z_end=25.0,
    save_path=None,
    apply_rotation=True,
    duration_s = 4.0,
    fps=24,
    width=1920,
    height=1080
):

    # ---- apply grid translations ONCE ----
    rot_fix = axis_angle_rotation([1,0,0], 90)
    for mesh, pos in zip(meshes, positions):
        mesh.translate(pos, relative=False)
        mesh.rotate(rot_fix, center=mesh.get_center())

    global running
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=width, height=height)
    vis.register_key_callback(ord("Q"), on_quit)
    
    for m in meshes:
        vis.add_geometry(m)

    # SETUP RENDER OPTIONS
    ro = vis.get_render_option()
    ro.light_on = False
    # ro.mesh_color_option = o3d.visualization.MeshColorOption.XCoordinate
    # ro.mesh_shade_option = 
    ro.background_color = np.asarray([0.7,0.7,0.7])
    ro.background_color = np.asarray([0,0,0])
    ro.show_coordinate_frame = False
    ro.point_size = 1.0
    ro.point_show_normal = False # Adds an arrow indicating normal direction

    ctr = vis.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()
    ctr.set_lookat([0,0,0])

    if save_path:
        writer = cv2.VideoWriter(
            save_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height)
        )
    else:
        writer = None

    # n_loops = 5
    # n = len(rotations)# * n_loops
    # n = int(duration_s * fps)
    deg_per_frame = 0.5
    loops = 3
    n = int(loops*360//deg_per_frame)
    last_animation_frame = int((loops-1)*(n/loops))
    print(f"Animating over {n} frames...")
    print(f"Last animation frame: {last_animation_frame}")

    # R_step = incremental_rotation(degrees_per_frame=2.0)  # slow & smooth

    while running:   # loop animation
        for i in range(n):
            if not running:
                break

            # ---- rotate all meshes ----
            if apply_rotation:
                
                for m in meshes:
                    m.rotate(rotation, center=m.get_center())
                    # m.rotate(rotations[i], center=m.get_center())

            # ---- camera pullback ----
            if i <= last_animation_frame:
                t = i / (last_animation_frame - 1)
                # print(f"N={n},T={t}")
                # t = t*t*(3 - 2*t)   # smoothstep
                z = cam_z_start + t * (cam_z_end - cam_z_start)

                params.extrinsic = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                ])

            ctr.convert_from_pinhole_camera_parameters(params)

            vis.update_geometry(None)
            vis.poll_events()
            vis.update_renderer()

            # ---- capture frame ----
            if writer:
                img = np.asarray(vis.capture_screen_float_buffer(False))
                img = (img * 255).astype(np.uint8)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                img = pad_to_size(img,width,height)
                print(f"Writing {img.shape}")
                writer.write(img)

        if not writer:
            continue
        else:
            break

    if writer:
        writer.release()

    vis.destroy_window()

def move_obj_to_center(paths, pattern, center_idx):
    idx = None
    for i, p in enumerate(paths):
        if pattern in p:
            idx = i
            break
    # return None
    if idx is not None:
        print(f"Switching {paths[idx]} to idx {center_idx}...")
        item = paths.pop(idx)
        paths.insert(center_idx, item)
    else:
        print(f"Couldnt find: {pattern}, no changes made")
    return paths

# Setup output video parameters
OUT_FPS=24
DURATION_S = 2
TOTAL_FRAMES = OUT_FPS * DURATION_S
SAVE_OUTPUT = False

# Gather model files
root_dir = "/home/csrobot/Omniverse/Models/MOAD2_test"
mesh_files = find_files(root_dir, "clean.ply",5)
print(f"Found {len(mesh_files)} meshes.")
# print(mesh_files)
# mesh_files = mesh_files[:9]
num_meshes = len(mesh_files)

# Compute object grid positions
n_objects = num_meshes #76
rows, cols = compute_grid_dims(n_objects, 16/9)#6/9)
positions = generate_grid_positions(n_objects, rows, cols, spacing=0.15)
print(positions[0], positions[len(positions)//2])
# print(positions)

# Compute sequence of rotation matrices
# rotation_frames = 240#TOTAL_FRAMES
# rotations = generate_rotation_matrices(rotation_frames)
# print(f"Generated {len(rotations)} rotation matrices.")
# print(rotations[0], rotations[-1])
rotation = incremental_rotation(axis=(-1,-1,-0.5),degrees_per_frame=0.5)  # slow & smooth


# Adjust specific model to the middle of the grid
random.shuffle(mesh_files) # Optional Shuffle
# mesh_files = sorted(mesh_files)
center_index = (rows // 2) * cols + (cols // 2)
mesh_files = move_obj_to_center(mesh_files, "conn-wp/", center_index)
meshes = load_meshes(mesh_files)
print(f"Loaded {num_meshes} meshes.")

# Animate scene
animate_scene(
    meshes,
    positions,
    rotation,
    cam_z_start=0.1,
    cam_z_end=1,
    save_path="/home/csrobot/Videos/moadv2_grid/test4.mp4"#"grid_spin.mp4"   # set None for preview only
)