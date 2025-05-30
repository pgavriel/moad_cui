import open3d as o3d
import numpy as np
import os
from os.path import join
import copy 
import tkinter as tk
from tkinter import filedialog

def select_point_cloud_files(root_dir, select="Point Cloud File"):
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    files = filedialog.askopenfilename(
        initialdir=root_dir,
        title=f"Select {select}",
        filetypes=[("Point Cloud Files", "*.ply *.pcd *.xyz *.las *.laz")]
    )
    print (f"{select}: {files}")
    return files

def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    Computes the 3x3 rotation matrix from Euler angles (roll, pitch, yaw).
    
    Args:
        roll_deg (float): Rotation angle around the x-axis in degrees (roll).
        pitch_deg (float): Rotation angle around the y-axis in degrees (pitch).
        yaw_deg (float): Rotation angle around the z-axis in degrees (yaw).
    
    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    # Convert angles from degrees to radians
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    
    # Rotation matrices for each axis
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combine the rotation matrices (Z * Y * X order)
    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
    
    return rotation_matrix

def filter_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Filters a point cloud to remove statistical outliers.
    
    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud.
        nb_neighbors (int): Number of neighbors to consider for each point.
        std_ratio (float): Threshold for the standard deviation. 
                           Points with a distance greater than this threshold are considered outliers.
                           
    Returns:
        o3d.geometry.PointCloud: Filtered point cloud without outliers.
        o3d.geometry.PointCloud: Outlier points for visualization (optional).
    """
    # Remove statistical outliers
    inlier_cloud, outlier_cloud = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    
    # Optional: Color inliers and outliers for visualization
    # inlier_cloud.paint_uniform_color([0.0, 1.0, 0.0])  # Green for inliers
    # outlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for outliers
    
    return inlier_cloud, outlier_cloud


def generate_bounding_box(center, x, y, z_bot, height):
    """
    Crop a point cloud to the specified dimensions.
    
    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud.
        center (tuple): Center of the cropping box (x, y, z).
        width (float): Width of the cropping box along the X-axis.
        height (float): Height of the cropping box along the Y-axis.
        depth (float): Depth of the cropping box along the Z-axis.

    Returns:
        o3d.geometry.PointCloud: Cropped point cloud.
    """
    # Calculate the bounds of the cropping box
    x_min = center[0] - x / 2
    x_max = center[0] + x / 2
    y_min = center[1] - y / 2
    y_max = center[1] + y / 2
    z_min = center[2] + z_bot
    z_max = z_min + height

    # Define the axis-aligned bounding box
    bounding_box = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array([x_min, y_min, z_min]),
        max_bound=np.array([x_max, y_max, z_max])
    )
    bounding_box.color = [1.0,0.0,0.0]
    return bounding_box

def crop_cloud(pcd, bounding_box):
    # Crop the point cloud
    cropped_pcd = pcd.crop(bounding_box)
    return cropped_pcd

def analyze_pointcloud(pcd,cloud_name=None):
    """
    Analyze a point cloud and print useful metrics.
    
    Args:
        pcd (o3d.geometry.PointCloud): The point cloud to analyze.
    
    Returns:
        dict: A dictionary of computed metrics.
    """
    if cloud_name is not None:
        print(f"Getting Cloud Information for: {cloud_name}...")

    metrics = {}

    # Number of points
    metrics["num_points"] = np.asarray(pcd.points).shape[0]
    
    # Fields present
    metrics["has_normals"] = pcd.has_normals()
    metrics["has_colors"] = pcd.has_colors()

    # Bounding box
    aabb = pcd.get_axis_aligned_bounding_box()
    metrics["bounding_box_extent"] = aabb.get_extent()
    metrics["bounding_box_volume"] = np.prod(aabb.get_extent())
    metrics['bounding_box_center'] = aabb.get_center()
    
    # Center of mass (mean of all points)
    points = np.asarray(pcd.points)
    metrics["center_of_mass"] = np.mean(points, axis=0)
    
    # Density (approximation: points per volume of bounding box)
    metrics["density"] = metrics["num_points"] / metrics["bounding_box_volume"]

    # Principal axes and orientation
    # covariance = np.cov(points, rowvar=False)
    # eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    # metrics["principal_axes"] = eigenvectors
    # metrics["eigenvalues"] = eigenvalues

    # Radius and volume of convex hull (optional, computationally intensive)
    try:
        hull = pcd.compute_convex_hull()[0]
        hull_volume = hull.get_volume()
        metrics["convex_hull_volume"] = hull_volume
        metrics["convex_hull_density"] = metrics["num_points"] / hull_volume if hull_volume > 0 else None
    except Exception as e:
        metrics["convex_hull_volume"] = None
        metrics["convex_hull_density"] = None
        print(f"Warning: Could not compute convex hull. Reason: {e}")

    # Print all metrics
    for key, value in metrics.items():
        print(f"{key}: {value}")
    print("")
    return metrics

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])
    
def colored_cloud_registration(source,target,vx_rad_list,max_iter_list,viz_downsamples=False):
    '''
    source: Pointcloud to align
    target: Ground truth, cloud to align to
    colored pointcloud registration
    This is implementation of following paper
    J. Park, Q.-Y. Zhou, V. Koltun,
    Colored Point Cloud Registration Revisited, ICCV 2017
    https://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html     
    '''
    voxel_radius = vx_rad_list
    print(f"Voxel Radii: {voxel_radius}")
    max_iter = max_iter_list
    print(f"Max Iter: {max_iter}")
    assert len(voxel_radius) == len(max_iter)

    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    for scale in range(len(voxel_radius)):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.4f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        # Visualize downsample
        if viz_downsamples:
            o3d.visualization.draw_geometries([source_down, target_down])

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp)

    # draw_registration_result_original_color(source, target, result_icp.transformation)
    return result_icp.transformation


# Main visualization function
def visualize_with_keybindings(pcd_list,crop_bb=None):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    # Get the render options and set the point size
    render_option = vis.get_render_option()
    render_option.point_size = 2.0  # Set the desired point size (default is usually 5.0)

    # Dictionary to track point cloud visibility
    visibility_dict = {i: True for i in range(len(pcd_list))}

    # Add point clouds to the visualizer
    for i, pcd in enumerate(pcd_list):
        vis.add_geometry(pcd)#, reset_bounding_box=(i == 0))  # Reset view for the first point cloud

    # Create a coordinate frame as a persistent placeholder to prevent crashing
    placeholder = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    vis.add_geometry(placeholder)

    # if crop_bb is not None:
    #     vis.add_geometry(crop_bb)

    # Store camera parameters to prevent view reset
    def capture_camera_params(vis):
        return vis.get_view_control().convert_to_pinhole_camera_parameters()

    def set_camera_params(vis, params):
        vis.get_view_control().convert_from_pinhole_camera_parameters(params)

    camera_params = capture_camera_params(vis)
    # print("Camera Parameters")
    # print(f"Intrinsics:\n{camera_params.intrinsic}")
    # print(f"Extrinsics:\n{camera_params.extrinsic}\n")


    # Keybinding: Toggle visibility for each point cloud
    def make_toggle_callback(idx):
        def toggle_visibility(vis):
            nonlocal camera_params
            camera_params = capture_camera_params(vis)
            visibility_dict[idx] = not visibility_dict[idx]
            if visibility_dict[idx]:
                vis.add_geometry(pcd_list[idx])  # Re-add if it was removed
            else:
                vis.remove_geometry(pcd_list[idx])  # Remove from the visualizer

            # Prevent crash by keeping at least one geometry
            # if not any(visibility_dict.values()):
            #     vis.add_geometry(placeholder)
            # elif placeholder in vis.get_render_option().geometry_to_render:
            #     vis.remove_geometry(placeholder)

            # Restore camera parameters to avoid view reset
            print("INSIDE Camera Parameters")
            print(f"Intrinsics:\n{camera_params.intrinsic}")
            print(f"Extrinsics:\n{camera_params.extrinsic}\n")
            set_camera_params(vis, camera_params)
            print(f"Point cloud {idx + 1} visibility: {visibility_dict[idx]}")
            return False
        return toggle_visibility

    # Register callbacks for toggling visibility
    for i in range(len(pcd_list)):
        vis.register_key_callback(ord(str(i + 1)), make_toggle_callback(i))  # Press "1", "2", etc.

    if crop_bb is not None:
        vis.add_geometry(crop_bb)
        fixed_center = crop_bb.get_center()
        def adjust_bbox_size(vis, axis, delta, crop_bb):
            """Adjust the bounding box size along a specified axis."""
            # nonlocal crop_bb, fixed_center
            nonlocal camera_params
            camera_params = capture_camera_params(vis)
            bb_center = crop_bb.get_center()
            bb_extent = crop_bb.get_extent()
            print(f"Center: {bb_center}")
            print(f"Extent: {bb_extent}")
            min_bound = np.array(crop_bb.min_bound)
            print(f"Min: {min_bound}")
            # max_bound = np.array(bbox.max_bound)
            if axis == 'z_height':
                bb_extent[2] +=  delta
                print(f"New Extent: {bb_extent}")
            # elif axis == 'y':
            #     max_bound[1] += delta
            # elif axis == 'z':
            #     max_bound[2] += delta
            vis.remove_geometry(crop_bb)
            crop_bb = generate_bounding_box(fixed_center,bb_extent[0],bb_extent[1],min_bound[2],bb_extent[2])
            # bbox.min_bound = o3d.utility.Vector3dVector(min_bound)
            # bbox.max_bound = o3d.utility.Vector3dVector(max_bound)
            # vis.update_geometry(crop_bb)
            vis.add_geometry(crop_bb)
            vis.update_renderer()
            set_camera_params(vis, camera_params)
        delta = 0.1
        vis.register_key_callback(ord("A"), lambda vis: adjust_bbox_size(vis, 'z_height', delta, crop_bb))  # Increase zbot
        vis.register_key_callback(ord("Z"), lambda vis: adjust_bbox_size(vis, 'z_height', -delta, crop_bb)) # Decrease zbot
        # vis.register_key_callback(ord("Y"), lambda vis: adjust_bbox_size(vis, 'y', delta))  # Increase Y
        # vis.register_key_callback(ord("H"), lambda vis: adjust_bbox_size(vis, 'y', -delta)) # Decrease Y
        # vis.register_key_callback(ord("U"), lambda vis: adjust_bbox_size(vis, 'z', delta))  # Increase Z
        # vis.register_key_callback(ord("J"), lambda vis: adjust_bbox_size(vis, 'z', -delta)) # Decrease Z


    print("Controls:")
    for i in range(len(pcd_list)):
        print(f"  {i + 1} : Toggle visibility of point cloud {i + 1}")

    # Run the visualizer
    vis.run()
    vis.destroy_window()

if __name__ == '__main__':
    # Load the point clouds
    root_dir = "/home/csrobot/data-mount/a3_windex_pose-c/exports"
    bot_pc = "a3_windex_pose-c-cloud-1-normals.ply"
    pcd2 = o3d.io.read_point_cloud(join(root_dir,bot_pc))

    root_dir = "/home/csrobot/data-mount/a3_windex_pose-a/exports"
    # root_dir = "/home/csrobot/data-mount/fused_data/engine/dense2"
    # pc1_file = filedialog.askopenfilename(title="Select PC1", initialdir=root_dir,filetypes=accepted_filetypes)
    # pc1_file = select_point_cloud_files(root_dir, "Point Cloud 1")
    # pc2_file = select_point_cloud_files(root_dir, "Point Cloud 2 (Align to PC1)")
    top_pc = "a3_windex_pose-a-cloud-1-normals.ply"
    pcd1 = o3d.io.read_point_cloud(join(root_dir,top_pc))
    
    output_file = 'conn-wp_aligned.ply'
    origin_mode = "centermass" # [floor, bbcenter, centermass]
    # exit()
    # pcd1 = o3d.io.read_point_cloud(pc1_file)
    # pcd2 = o3d.io.read_point_cloud(pc2_file)
    metrics1 = analyze_pointcloud(pcd1,"PC1")
    metrics2 = analyze_pointcloud(pcd2,"PC2")

    # Stage 1
    # Crop Bounding Box
    bounds = {
        'x': 0.23,
        'y': 0.36,
        'z_bot': -0.16,
        'height': 0.4
    }
    bb = generate_bounding_box(metrics1['bounding_box_center'],bounds['x'],bounds['y'],bounds['z_bot'],bounds['height'])

    #Stage 2
    #Filter Settings
    filter_neighbors = 30
    filter_ratio = 1.5

    # Stage 3
    # Initial rotation matrix for roughly aligning PCD2
    rotation_matrix = euler_to_rotation_matrix(-90, 180, 0)
    # Parameters for ICP Registration
    voxel_radius = [0.005, 0.002, 0.001]
    max_iter = [500, 100, 100]
    # voxel_radius = [0.001]
    # max_iter = [1000]


    stage = 4#4


    viz_all = True

    # # Visualize aligned point clouds
    # o3d.visualization.draw_geometries([pcd1, pcd2])
    #STAGE 1: Determine Appropriate Crop Bounding Box
    if stage >= 1:
        pass
        # if viz_all: visualize_with_keybindings([pcd1, pcd2],crop_bb=bb)
        if viz_all: visualize_with_keybindings([pcd1],crop_bb=bb)
    # exit()
    #STAGE 2: Apply Crop & Filtering
    if stage >= 2:
        pcd1 = crop_cloud(pcd1, bb)
        pcd2 = crop_cloud(pcd2, bb)
        
        if viz_all: visualize_with_keybindings([pcd1, pcd2])

    #STAGE 3: Automatic Alignment
    if stage >= 3: 
        print("Stage 3")
        # Apply filtering
        pcd1_filtered, outliers1 = filter_statistical_outliers(pcd1, nb_neighbors=filter_neighbors, std_ratio=filter_ratio)
        pcd2_filtered, outliers2 = filter_statistical_outliers(pcd2, nb_neighbors=filter_neighbors, std_ratio=filter_ratio)
        if viz_all: visualize_with_keybindings([pcd1_filtered, pcd2_filtered])
        # Initial 180 degree rotation
        pcd2.rotate(rotation_matrix, center=metrics2['bounding_box_center'])
        pcd2_filtered.rotate(rotation_matrix, center=metrics2['bounding_box_center'])
        # Align Bounding Boxes
        aabb = pcd1_filtered.get_axis_aligned_bounding_box()
        pcd1_center = aabb.get_center()
        aabb = pcd2_filtered.get_axis_aligned_bounding_box()
        pcd2_center = aabb.get_center()
        bb_translate = pcd1_center - pcd2_center
        print(f"PCD1 Center: {pcd1_center}")
        print(f"PCD2 Center: {pcd2_center}")
        print(f"Align Translate: {bb_translate}")
        pcd2.translate(bb_translate)
        pcd2_filtered.translate(bb_translate)
        if viz_all: visualize_with_keybindings([pcd1_filtered, pcd2_filtered])
        # exit()
        # Calculate Alignment transform using filtered clouds
        transformation = colored_cloud_registration(pcd2_filtered, pcd1_filtered, voxel_radius, max_iter)
        # Apply transformation to the original point cloud
        pcd2.transform(transformation)
        pcd2_filtered.transform(transformation)

    #STAGE 4: Fusion, Global Alignment, and export
    if stage == 4:
        # Fuse the point clouds
        fused_pcd = pcd1 + pcd2
        
        fused_filtered, outliers2 = filter_statistical_outliers(fused_pcd, nb_neighbors=filter_neighbors, std_ratio=filter_ratio)
        # Get the bounding box of the filtered fused cloud
        bbox = fused_filtered.get_axis_aligned_bounding_box()
        bbox_min = bbox.get_min_bound()  # [x_min, y_min, z_min]
        bb_center = bbox.get_center() # bounding box center
        cloud_center = fused_filtered.get_center() # center of mass

        print(f"Origin Mode: {origin_mode}")
        if origin_mode == "floor:":
            origin = cloud_center
            origin[2] = bbox_min[2]
        elif origin_mode == "bbcenter":
            origin = bb_center
        elif origin_mode == "centermass":
            origin = cloud_center
        else:
            print("WARNING: origin mode not recognized, set to [0,0,0]")
            origin = [0,0,0]
        
        # Apply origin translation
        translation_vector = np.array(-origin)
        print(f"Translating Fused Cloud: {translation_vector}")
        fused_pcd.translate(translation_vector)

        visualize_with_keybindings([fused_pcd],crop_bb=None)

        # Save the fused and recentered point cloud to the specified file
        output_path = join(root_dir,output_file)
        o3d.io.write_point_cloud(output_path, fused_pcd)
        print(f"Fused point cloud saved to {output_path}")

    # if stage > 1: bb = None
    # if stage <= 3:
    #     visualize_with_keybindings([pcd1, pcd2],crop_bb=bb)
    # else:
    #     visualize_with_keybindings([pcd1_filtered, pcd2_filtered],crop_bb=bb)