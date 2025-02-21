import open3d as o3d
import numpy as np
import cv2
import os
from tkinter import Tk, filedialog
from pathlib import Path

def get_first_level_subdir(root, path):
    root = Path(root).resolve()
    path = Path(path).resolve()

    if not path.is_relative_to(root):
        raise ValueError(f"{path} is not within {root}")

    relative_parts = path.relative_to(root).parts
    return root / relative_parts[0] if relative_parts else None

def ensure_directory_exists(directory):
    path = Path(directory)
    if path.exists():
        print(f"Directory already exists: {directory}")
    else:
        path.mkdir(parents=True, exist_ok=True)
        print(f"Created directory: {directory}")

def select_pointcloud(init_dir="/home/csrobot"):
    """Open a file dialog to select a point cloud file."""
    Tk().withdraw()  # Hide the root window
    file_path = filedialog.askopenfilename(initialdir=init_dir,filetypes=[("Point Cloud Files", "*.ply *.pcd *.xyz *.pts *.txt")])
    return file_path

def capture_frames(vis, image_list, render=True):
    """Capture a frame from Open3D visualizer."""
    img = np.asarray(vis.capture_screen_float_buffer(do_render=render)) * 255
    img = img.astype(np.uint8)
    image_list.append(img)
    return False  # Returning False ensures the callback continues

def capture_frame(vis, render=True):
    """Capture a frame from Open3D visualizer."""
    img = np.asarray(vis.capture_screen_float_buffer(do_render=render)) * 255
    img = img.astype(np.uint8)
    return img

def orbit_camera(vis, point_cloud, output_video="output.mp4", step=10, total_frames=220):
    """Orbit the camera around the object and record frames."""
    ctr = vis.get_view_control()
    param = ctr.convert_to_pinhole_camera_parameters()

    images = []
    ctr.rotate(0.0, -450.0)
    ctr.translate(0.0, 200.0)
    ctr.scale(0.8)
    vis.poll_events()
    vis.update_renderer()
    for i in range(total_frames):
        capture_frames(vis, images)  # Capture current frame
        ctr.rotate(step, 0.0)  # Rotate the camera horizontally
        vis.poll_events()
        vis.update_renderer()

    # Save frames as a video
    height, width, _ = images[0].shape
    video_writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*"mp4v"), 24, (width, height))

    for img in images:
        #TODO: Add function to draw overlay on top of each frame
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))  # Convert to BGR for OpenCV

    video_writer.release()
    print(f"Saved video as {output_video}")

def rotate_geometry(geometry,x=0,y=0,z=0):
    R = geometry.get_rotation_matrix_from_xyz((np.deg2rad(x), np.deg2rad(y), np.deg2rad(z)))
    geometry.rotate(R)

def capture_camera_params(vis):
        """Prints the current camera parameters in a copy-paste friendly format."""
        ctr = vis.get_view_control()
        cam_params = ctr.convert_to_pinhole_camera_parameters()
        
        extrinsic = cam_params.extrinsic.tolist()  # Convert to list for easy printing
        intrinsic = cam_params.intrinsic.intrinsic_matrix.tolist()

        print("\nCamera Parameters:\n")
        print(f"Extrinsic: {extrinsic}")
        print(f"Intrinsic: {intrinsic}\n")
        print("Copy these values and use them in `convert_from_pinhole_camera_parameters()`.")

def main():
    root_dir = "/home/csrobot/data-mount"
    file_path = select_pointcloud(root_dir)
    # file_path = "/home/csrobot/data-mount/a3_mustard_pose-a/exports/manual-align-mesh.ply"

    if not file_path:
        print("No file selected.")
        return
    else:
        print(f"Selected: {file_path}")

    object_folder = get_first_level_subdir(root_dir,file_path)
    print(f"Object Folder: {object_folder}")
    output_dir = os.path.join(object_folder,"media")
    ensure_directory_exists(output_dir)
    output_file = os.path.basename(file_path).split(".")[0]+"_orbit.mp4"
    output_path = os.path.join(output_dir,output_file)
    print(f"Output Path: {output_path}")

    # TODO: Implement programmatic check for whether file is mesh or cloud
    
    if "mesh" in  os.path.basename(file_path).lower():
        print("\'mesh\' in file name, attempting to load as mesh...")
        pcd = o3d.io.read_triangle_mesh(file_path)
    else: # Load as pointcloud
        print("\'mesh\' not in file name, attempting to load as cloud...")
        pcd = o3d.io.read_point_cloud(file_path)

    if pcd.is_empty():
        print("Failed to load file.")
        return
    
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    # Register keybindings
    vis.register_key_callback(ord("C"), lambda vis: capture_camera_params(vis))

    # Add our object
    vis.add_geometry(pcd)

    # SETUP RENDER OPTIONS
    ro = vis.get_render_option()
    ro.light_on = False
    # ro.mesh_color_option = o3d.visualization.MeshColorOption.XCoordinate
    # ro.mesh_shade_option = 
    ro.background_color = np.asarray([0.7,0.7,0.7])
    ro.show_coordinate_frame = False
    ro.point_size = 1.0
    ro.point_show_normal = False


    
    # SETUP INTITAL CAMERA POSE
    # Center camera
    ctr = vis.get_view_control()
    ctr.set_lookat(pcd.get_center())
    ctr.rotate(0.0, -350.0)
    ctr.translate(0.0, 100.0)
    ctr.scale(5.0)
    # Explicitly setting initial camera pose, get extrinsic from pressing 'C'
    # cam_params = ctr.convert_to_pinhole_camera_parameters()
    # cam_params.extrinsic = np.array([[0.9991301898048487, 0.04132926684272, -0.005545766202292168, 6.446856964342426e-05], [-0.037467686786386456, 0.8313793262619816, -0.5544407888233431, -0.008564534417497827], [-0.018303995940899983, 0.5541663176036747, 0.8322046960731347, 0.31708622503657724], [0.0, 0.0, 0.0, 1.0]])
    # ctr.convert_from_pinhole_camera_parameters(cam_params)

    vis.poll_events()
    vis.update_renderer()

    img = capture_frame(vis)
    # Save frames as a video
    height, width, _ = img.shape
    video_writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*"mp4v"), 24, (width, height))

    img_list = []
    angle_inc = 5
    # Rotate Z Loop
    for i in range(360//angle_inc):
        # print(i)
        rotate_geometry(pcd,0,0,angle_inc)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        img = capture_frames(vis,img_list)
    # Rotate X loop
    # for i in range(360//angle_inc):
    #     # print(i)
    #     rotate_geometry(pcd,-angle_inc,0,0)
    #     vis.update_geometry(pcd)
    #     vis.poll_events()
    #     vis.update_renderer()
    #     img = capture_frames(vis,img_list)
    
    print(f"Imgs:{len(img_list)}")
    for img in img_list:
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))  # Convert to BGR for OpenCV

    video_writer.release()
    print(f"Saved video as {output_path}")
    
    # orbit_camera(vis, pcd, output_video=output_path)


    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()
