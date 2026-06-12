'''
GENERATE SCENE ANNOTATIONS
Script used for generating ground truth object 6D pose data on an entire scan
of images collected by the MOAD rig. Before running, objects on the turntable
should have been carefully aligned to match a simulated scene with known
ground truth.

INPUTS:
 - Scene Folder/Pose Folder (Path):
        The target directory containing all scan images of the scene. Also
        serves as the output directory.
 - PyBullet Scene (.npz):
        Specifies which objects are a part of the scene and how they are
        positioned.
 - Scene Config (.json):
        Specifies a calibration offset and camera offset to refine the alignment
        of the virtual scene to the camera calibration poses.
 - Camera Calibration (.json):
        Provides the camera intrinsics/extrinsics with poses relative to the
        origin at the turntable center.
 - Scan Resolution (degrees/move, num_moves):
        Specifies which scene transforms need to be generated.

OUTPUT:
 - Scene Scan Poses (object_poses.json):
        For each image frame, the 6D pose (R, t) of each scene object expressed
        in that camera's coordinate frame, suitable for projecting bounding boxes.
 - Object Masks (Folder of images):   [not yet implemented]
 - Bounding Boxes (JSON):             [not yet implemented]

FRAME / CAMERA CONVENTION
    Images are named frame_NNNNN.jpg, 1-indexed.
    Cameras cycle through in order: frames 1..72 = cam1, 73..144 = cam2, etc.
    Within each camera block, turntable angle advances by STEP_DEG each frame,
    starting at 0° (frame 1 of the block) up to 355° (frame 72 of the block).

COORDINATE CONVENTIONS
    - cam_parameters.json stores w2c in OpenCV convention (X right, Y down, Z fwd).
    - get_camera_pose_from_extrinsics() mirrors live_view_demo.py exactly:
        * applies scale to translation
        * applies CV_TO_GL (identity in current rig config — handled elsewhere)
        * composes with a Z-rotation for the turntable angle + calibration offset
    - Object poses are retrieved from PyBullet in world frame, then transformed
      into the OpenCV camera frame to give R_obj_in_cam, t_obj_in_cam.
      These can be used directly to project 3D points into image space with the
      camera intrinsics.
'''

import os
import re
import sys
import glob
import json
import argparse
import numpy as np
import pybullet as p
import cv2

# Import Needed From: https://github.com/JohnBrann/scene_replica_moad 
SCENE_REPLICA_REPO = "/home/csrobot/moad_control/scene_replica_moad"  # adjust as needed
if not os.path.isdir(SCENE_REPLICA_REPO):
    print(f"[ERROR] scene_replica repo not found at: {SCENE_REPLICA_REPO}")
    print("        Update SCENE_REPLICA_REPO at the top of this script.")
    sys.exit(1)

sys.path.insert(0, SCENE_REPLICA_REPO)
from scene_replica_notag import TaglessSceneReplica, Rt_to_T, T_inv


# ---------------------------------------------------------------------------
# Constants — must stay in sync with live_view_demo.py
# ---------------------------------------------------------------------------

NUM_CAMERAS     = 5          # cams 1-5
FRAMES_PER_CAM  = 72         # frames per camera per full 360° scan
STEP_DEG        = 5.0        # turntable degrees per frame
CV_TO_GL        = np.eye(4)  # identity — matches live_view_demo.py


# ---------------------------------------------------------------------------
# Helpers shared with live_view_demo.py
# ---------------------------------------------------------------------------

def make_z_rotation(degrees: float) -> np.ndarray:
    """4x4 rotation matrix for a Z-axis rotation by `degrees`."""
    rad = np.radians(degrees)
    c, s = np.cos(rad), np.sin(rad)
    R = np.eye(4)
    R[0, 0] =  c;  R[0, 1] = -s
    R[1, 0] =  s;  R[1, 1] =  c
    return R


def get_camera_pose_from_extrinsics(
    cam_key:       str,
    cameras:       dict,
    turntable_deg: float,
    global_offset: np.ndarray,
    scale:         float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute R_cw_cv, t_cw_cv for a given camera and turntable angle.
    Mirrors live_view_demo.py exactly so renders match the collected images.

    Returns:
        R_cw_cv : (3,3) rotation  — world-to-camera, OpenCV convention
        t_cw_cv : (3,)  translation
    """
    T_w2c = cameras[cam_key]["w2c"].copy()
    T_w2c[:3, 3] *= scale

    T_w2c_cv = CV_TO_GL @ T_w2c

    T_scene = make_z_rotation(turntable_deg)
    T_scene[:3, 3] = global_offset * scale

    T_final = T_w2c_cv @ T_scene
    return T_final[:3, :3], T_final[:3, 3]


def build_cam_K(intrinsics: dict) -> np.ndarray:
    return np.array([
        [intrinsics["fx"],             0.0, intrinsics["cx"]],
        [            0.0, intrinsics["fy"], intrinsics["cy"]],
        [            0.0,             0.0,             1.0],
    ], dtype=np.float64)


ASPECT_TOL = 0.01   # mirrors live_view_demo.py

def check_aspect_ratio(intrinsics: dict, frame: np.ndarray) -> float:
    """
    Verify a frame's aspect ratio matches the calibration intrinsics and
    return the scale factor (frame_width / calib_width).
    Raises ValueError if the aspect ratios diverge beyond ASPECT_TOL.
    Mirrors live_view_demo.py check_aspect_ratio exactly.
    """
    calib_w  = intrinsics["width"]
    calib_h  = intrinsics["height"]
    frame_h, frame_w = frame.shape[:2]

    calib_ar = calib_w / calib_h
    frame_ar = frame_w / frame_h

    if abs(calib_ar - frame_ar) > ASPECT_TOL:
        raise ValueError(
            f"Aspect ratio mismatch: calibration={calib_w}x{calib_h} ({calib_ar:.4f}) "
            f"vs image={frame_w}x{frame_h} ({frame_ar:.4f}). "
            f"Check which images_N folder you are targeting."
        )

    scale = frame_w / calib_w
    print(f"  Aspect ratio OK ({calib_ar:.4f}). "
          f"Intrinsic scale: {calib_w}x{calib_h} → {frame_w}x{frame_h}  (×{scale:.4f})")
    return scale


def scale_intrinsics(intrinsics: dict, scale: float) -> dict:
    """
    Return a copy of the intrinsics dict with all resolution-dependent fields
    scaled uniformly. Mirrors live_view_demo.py scale_intrinsics exactly.
    """
    return {
        **intrinsics,
        "fx":     intrinsics["fx"]     * scale,
        "fy":     intrinsics["fy"]     * scale,
        "cx":     intrinsics["cx"]     * scale,
        "cy":     intrinsics["cy"]     * scale,
        "width":  int(round(intrinsics["width"]  * scale)),
        "height": int(round(intrinsics["height"] * scale)),
    }


def _parse_obj_vertices(obj_path: str) -> np.ndarray:
    """Extract vertex positions from a .obj file. Returns (N, 3) array."""
    verts = []
    with open(obj_path, "r") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not verts:
        raise ValueError(f"No vertices found in: {obj_path}")
    return np.array(verts, dtype=np.float64)

# ---------------------------------------------------------------------------
# Frame index → (cam_key, turntable_deg)
# ---------------------------------------------------------------------------

def frame_index_to_cam_and_angle(
    frame_idx: int,
    num_cameras:    int   = NUM_CAMERAS,
    frames_per_cam: int   = FRAMES_PER_CAM,
    step_deg:       float = STEP_DEG,
) -> tuple[str, float]:
    """
    Convert a 0-based frame index to a camera key and turntable angle.

    Frame files are 1-indexed (frame_00001.jpg = index 0 here).
    Layout:
        indices  0 .. 71  → cam1, angles 0°, 5°, ..., 355°
        indices 72 .. 143 → cam2, angles 0°, 5°, ..., 355°
        ...
    """
    cam_number    = (frame_idx // frames_per_cam) + 1
    pos_in_block  = frame_idx %  frames_per_cam
    cam_key       = f"cam{cam_number}"
    turntable_deg = pos_in_block * step_deg
    return cam_key, turntable_deg


# ---------------------------------------------------------------------------
# Object pose extraction
# ---------------------------------------------------------------------------
# Global cache for storing model extents (included in annotations)
_size_cache = {}
def get_object_poses_in_camera_frame(
    R_cw_cv: np.ndarray,
    t_cw_cv: np.ndarray,
) -> list[dict]:
    """
    Query every non-visual body from PyBullet, express its pose in the
    OpenCV camera frame, and return annotation dicts.

    The world frame here is the PyBullet simulation world, which shares
    its origin and axes with the calibration world frame (turntable centre,
    Z-up) after the scene offsets are applied at load time.

    Object pose in camera frame is computed as:
        T_obj_in_cam = T_cw @ T_obj_in_world

    where T_cw is the world-to-camera transform (OpenCV convention).

    Args:
        R_cw_cv : (3,3) world-to-camera rotation  (OpenCV)
        t_cw_cv : (3,)  world-to-camera translation (OpenCV)

    Returns:
        List of dicts, one per scene object:
            {
                "object_name": str,
                "body_id":     int,
                "R":           [[...]] (3x3, row-major),
                "t":           [x, y, z]   (metres, in camera frame),
                "size":        [x, y, z]   (full XYZ extents in metres, object-aligned)
            }
    """
    
    T_cw = Rt_to_T(R_cw_cv, t_cw_cv)   # 4x4 world-to-camera (OpenCV)

    annotations = []
    num_bodies = p.getNumBodies()

    for i in range(num_bodies):
        body_id   = p.getBodyUniqueId(i)
        body_name = p.getBodyInfo(body_id)[1].decode("utf-8")
        if body_name == "":
            continue  # skip visual-only markers (circles, bars, etc.)

        pos_w, quat_w = p.getBasePositionAndOrientation(body_id)
        R_mat = np.array(p.getMatrixFromQuaternion(quat_w)).reshape(3, 3)
        T_obj_w = Rt_to_T(R_mat, np.array(pos_w))

        T_obj_cam = T_cw @ T_obj_w

        # --- Object extents ---
        # getAABB returns an axis-aligned bounding box in world space.
        # To get object-aligned extents, we temporarily reset the object to
        # identity pose, query the AABB (which is now object-aligned), then
        # restore it. This gives us the true mesh extents independent of
        # the object's current world rotation.
        # p.resetBasePositionAndOrientation(body_id, [0, 0, 0], [0, 0, 0, 1])
        # aabb_min, aabb_max = p.getAABB(body_id)
        # p.resetBasePositionAndOrientation(body_id, pos_w, quat_w)

        # size = (np.array(aabb_max) - np.array(aabb_min)).tolist()
        if body_name not in _size_cache:
            visual_shapes = p.getVisualShapeData(body_id)
            shape      = visual_shapes[0]
            mesh_path  = shape[4].decode("utf-8")
            mesh_scale = np.array(shape[3])
            verts      = _parse_obj_vertices(mesh_path)
            verts     *= mesh_scale
            _size_cache[body_name] = (verts.max(axis=0) - verts.min(axis=0)).tolist()
            print(f"[CACHED OBJECT SIZE] {body_name}: {_size_cache[body_name]}")

        size = _size_cache.get(body_name, [0.1, 0.1, 0.1])

        annotations.append({
            "object_name": body_name,
            "body_id":     body_id,
            "R":           T_obj_cam[:3, :3].tolist(),
            "t":           T_obj_cam[:3,  3].tolist(),
            "size":        size,# (hardcoded for testing, PyBullet bounding box doesnt seem accurate)
        })

    return annotations

# ---------------------------------------------------------------------------
# Main annotation generator
# ---------------------------------------------------------------------------

def generate_pose_data(
    root_folder:    str,
    scene_replica:  TaglessSceneReplica,
    cameras:        dict,
    scene_cfg:      dict,
    scale:          float,
    intrinsics:     dict,
    visualize:      bool = False,
) -> None:
    """
    Generates one JSON annotation file per frame in root_folder/scene_replica/pose/.

    For every frame_NNNNN.jpg found in root_folder/images_8, computes the
    world-to-camera pose for the corresponding camera / turntable angle,
    optionally renders a PyBullet overlay for visual verification, and
    records the 6D pose of every scene object in that camera frame.

    The scene_replica is initialised at full calibration resolution. If the
    target images are a downscaled version, intrinsics are scaled to match
    before the first render (mirroring live_view_demo.py's behaviour), and
    the replica is resized accordingly so the overlay composites correctly.

    Output format (per-frame JSON, e.g. frame_00001.json):
    {
        "frame":         "frame_00001.jpg",
        "cam_key":       "cam1",
        "turntable_deg": 0.0,
        "objects": [
            {
                "object_name": "gear_large",
                "body_id": 2,
                "R": [[...], [...], [...]],
                "t": [x, y, z]
            },
            ...
        ]
    }
    """
    images_dir = os.path.join(root_folder, "images_4")
    if not os.path.isdir(images_dir):
        raise FileNotFoundError(f"Images folder not found: {images_dir}")

    image_paths = sorted(glob.glob(os.path.join(images_dir, "frame_*.jpg")))
    if not image_paths:
        raise FileNotFoundError(f"No frame_*.jpg images found in: {images_dir}")

    print(f"  Found {len(image_paths)} images in {images_dir}")

    # Establish output root, creating it if needed
    output_root = os.path.join(root_folder, "scene_replica", "pose")
    os.makedirs(output_root, exist_ok=True)
    print(f"  Output directory: {output_root}")

    # ── Scale intrinsics to match actual image resolution ─────────────────────
    # Read one image to detect the rendered resolution, then scale the replica
    # to match — identical to the live_view_demo.py startup sequence.
    if visualize:
        import cv2
        ref_frame = cv2.imread(image_paths[0])
        if ref_frame is None:
            raise RuntimeError(f"Could not read reference frame: {image_paths[0]}")
        intr_scale        = check_aspect_ratio(intrinsics, ref_frame)
        scaled_intrinsics = scale_intrinsics(intrinsics, intr_scale)
        scene_replica.cam_K = build_cam_K(scaled_intrinsics)
        scene_replica.W     = scaled_intrinsics["width"]
        scene_replica.H     = scaled_intrinsics["height"]
        scene_replica._compute_projection_matrix()
        print(f"  Render resolution set to: {scene_replica.W}×{scene_replica.H}")

    global_offset = np.asarray(scene_cfg["CALIBRATION_OFFSET"], dtype=np.float64)

    for img_path in image_paths:
        filename = os.path.basename(img_path)

        # Parse 1-based frame number from filename, convert to 0-based index
        match = re.search(r"frame_(\d+)\.jpg", filename)
        if not match:
            print(f"  [SKIP] Unexpected filename format: {filename}")
            continue
        frame_number = int(match.group(1))          # 1-based
        frame_idx    = frame_number - 1             # 0-based

        cam_key, turntable_deg = frame_index_to_cam_and_angle(frame_idx)

        if cam_key not in cameras:
            print(f"  [SKIP] {filename} → {cam_key} not in calibration")
            continue

        R_cw_cv, t_cw_cv = get_camera_pose_from_extrinsics(
            cam_key, cameras, turntable_deg, global_offset, scale
        )

        scene_replica.update_camera(R_cw_cv, t_cw_cv)

        if visualize:
            rgba = scene_replica.render_scene_image()
            bgr  = cv2.imread(img_path)
            if bgr is not None:
                # Resize real image to render resolution if needed (e.g. images_4 vs images_8)
                rh, rw = scene_replica.H, scene_replica.W
                if bgr.shape[1] != rw or bgr.shape[0] != rh:
                    bgr = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_LINEAR)
                bg    = bgr.astype(np.float32)
                ov    = rgba[:, :, [2, 1, 0, 3]].astype(np.float32)
                alpha = ov[:, :, 3:4] / 255.0
                comp  = np.clip(ov[:, :, :3] * alpha + bg * (1.0 - alpha), 0, 255).astype(np.uint8)
                cv2.imshow("Annotation Verify", comp)
                cv2.waitKey(1)

        frame_annotations = {
            "frame":         filename,
            "cam_key":       cam_key,
            "turntable_deg": turntable_deg,
            "objects":       get_object_poses_in_camera_frame(R_cw_cv, t_cw_cv),
        }

        output_path = os.path.join(output_root, f"frame_{frame_number:05d}.json")
        with open(output_path, "w") as f:
            json.dump(frame_annotations, f, indent=2)

        print(f"  [{frame_number:>5}] {cam_key}  {turntable_deg:>5.1f}°  "
              f"→ {len(frame_annotations['objects'])} object(s)  →  {os.path.basename(output_path)}")

    if visualize:
        cv2.destroyAllWindows()

    print(f"\n  Annotations written to: {output_root}")

def generate_bb_data(root_folder, scene_replica):
    # Will implement later
    print("[MESSAGE]: Bounding Box Annotations NOT YET IMPLEMENTED")
    input("(Enter to continue)")


def generate_mask_data(root_folder, scene_replica):
    # Will implement later
    print("[MESSAGE]: Mask Annotations NOT YET IMPLEMENTED")
    input("(Enter to continue)")


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------

def validate_inputs(data_root: str, object_name: str, pose_name: str) -> tuple[str, str, str]:
    """
    Validates the expected directory structure and returns paths to the
    scene replica folder, scene .npz file, and scene config .json file.

    Expected layout:
        <data_root>/<object_name>/<pose_name>/scene_replica/
            ├── <scene>.npz
            └── <config>.json
    """
    pose_path    = os.path.join(data_root, object_name, pose_name)
    replica_path = os.path.join(pose_path, "scene_replica")

    if not os.path.isdir(pose_path):
        raise FileNotFoundError(
            f"Object/pose folder not found: {pose_path}"
        )
    if not os.path.isdir(replica_path):
        raise FileNotFoundError(
            f"scene_replica folder not found: {replica_path}"
        )

    npz_files  = glob.glob(os.path.join(replica_path, "*.npz"))
    json_files = glob.glob(os.path.join(replica_path, "*.json"))

    if not npz_files:
        raise FileNotFoundError(f"No .npz scene file found in: {replica_path}")
    if not json_files:
        raise FileNotFoundError(f"No .json config file found in: {replica_path}")

    return pose_path, npz_files[0], json_files[0]


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args):
    # ── Validate directory structure ──────────────────────────────────────────
    pose_path, npz_path, cfg_path = validate_inputs(
        args.data_root, args.object, args.pose
    )
    print(f"  Scene file  : {npz_path}")
    print(f"  Config file : {cfg_path}")
    print(f"  Pose folder : {pose_path}\n")

    # ── Load scene config ─────────────────────────────────────────────────────
    with open(cfg_path, "r") as f:
        scene_cfg = json.load(f)

    # ── Load camera calibration ───────────────────────────────────────────────
    calib_path = os.path.join(args.calib_root,args.calibration,"cam_parameters.json")
    with open(calib_path, "r") as f:
        cal = json.load(f)

    intrinsics = cal["intrinsics"]
    scale_info = cal["_info"]["scaling"]
    scale      = scale_info["scale"]

    cameras = {}
    for cam_name, cam_data in cal["cameras"].items():
        cameras[cam_name] = {
            "c2w": np.array(cam_data["extrinsics"]["c2w"], dtype=np.float64),
            "w2c": np.array(cam_data["extrinsics"]["w2c"], dtype=np.float64),
        }
    print(f"  Loaded calibration for: {sorted(cameras.keys())}")

    # ── Initialise scene replica ──────────────────────────────────────────────
    cam_K     = build_cam_K(intrinsics)
    scene_file = os.path.basename(npz_path)

    # Use cam1 at 0° as a reasonable initial pose for scene load
    initial_offset = np.asarray(scene_cfg["CALIBRATION_OFFSET"], dtype=np.float64)
    R_init, t_init = get_camera_pose_from_extrinsics(
        "cam1", cameras, 0.0, initial_offset, scale
    )
    scene = TaglessSceneReplica(
        cam_K        = cam_K,
        W            = intrinsics["width"],
        H            = intrinsics["height"],
        R_cw_cv      = R_init,
        t_cw_cv      = t_init,
        scene_config = scene_cfg,
        scene_path   = os.path.join(pose_path,"scene_replica"),
        model_library_path = args.model_library
    )
    scene.load_scene(scene_file)
    print("  Scene loaded.\n")

    # ── Generate annotations ──────────────────────────────────────────────────
    if args.generate_poses:
        print("  Generating pose annotations...")
        generate_pose_data(
            root_folder   = pose_path,
            scene_replica = scene,
            cameras       = cameras,
            scene_cfg     = scene_cfg,
            scale         = scale,
            intrinsics    = intrinsics,
            visualize     = args.visualize,
        )

    if args.generate_bbs:
        print("  Bounding box generation not yet implemented.")
        generate_bb_data(pose_path, scene)

    if args.generate_masks:
        print("  Mask generation not yet implemented.")
        generate_mask_data(pose_path, scene)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    try:
        p.disconnect(scene.pb)
    except Exception:
        pass
    print("\n  Done.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate ground truth 6D pose annotations for a MOAD scan."
    )

    parser.add_argument("--data-root",   default="/home/csrobot/MOAD_DATA",
                        help="Root data directory")
    parser.add_argument("--object",      default="batch1_001",
                        help="Object subfolder name")
    parser.add_argument("--pose",        default="pose-a",
                        help="Pose subfolder name")
    parser.add_argument("--calib-root",   default="/home/csrobot/moad_control/moad_cui/calibration",
                        help="Root calibration directory")
    parser.add_argument("--calibration",  default="55mm",
                        help="Folder name of camera calibration to use")
    parser.add_argument("--model-library", default="/home/csrobot/moad_control/scene_replica_moad/assets/object_sets/moad",
                        help="Folder containing all models referenced by the scene being loaded")
    
    # Output Options
    parser.add_argument("--generate-poses",  action="store_true", default=True,
                        help="Generate 6D pose annotations (default: on)")
    parser.add_argument("--generate-bbs",    action="store_true", default=False,
                        help="Generate bounding box annotations (not yet implemented)")
    parser.add_argument("--generate-masks",  action="store_true", default=False,
                        help="Generate segmentation masks (not yet implemented)")
    parser.add_argument("--visualize",   action="store_true", default=True,
                        help="Show PyBullet overlay composited on each image while processing")

    args = parser.parse_args()
    main(args)