'''
GENERATE SCENE ANNOTATIONS
Script used for generating ground truth object 6D pose data on an entire scan
of images collected by the MOAD rig. Before running, objects on the turntable
should have been carefully aligned to match a simulated scene with known
ground truth, and the alignment confirmed via the live view interface.

INPUTS:
 - Scene Folder/Pose Folder (Path):
        The target directory containing all scan images of the scene. Also
        serves as the output directory.
 - PyBullet Scene (.npz):
        Specifies which objects are a part of the scene and how they are
        positioned.
 - Scene Config (.json):
        Specifies a calibration offset to refine the alignment of the virtual
        scene to the camera calibration poses.
 - Calibration Folder:
        Contains one or more cam_parameters JSON files. Which sensors get
        annotated is determined automatically by which calibration files and
        image folders are present — see SENSOR_CONFIGS below.

OUTPUT:
 - Per-frame pose annotation JSON files, one per source image, written to:
        <pose_path>/scene_replica/pose_dslr/        ← DSLR annotations
        <pose_path>/scene_replica/pose_realsense/   ← RealSense annotations

   Each annotation file is named after its source image stem:
        frame_00001.jpg   →  frame_00001.json
        rs1_000_color.png →  rs1_000_color.json

   Output format per file:
   {
       "frame":         "<source filename>",
       "cam_key":       "cam1" / "rs1" / ...,
       "turntable_deg": 0.0,
       "objects": [
           {
               "object_name": str,
               "body_id":     int,
               "R":           [[...], [...], [...]],   (3x3, object-to-camera)
               "t":           [x, y, z],               (metres, in camera frame)
               "size":        [x, y, z]                (full XYZ extents, metres)
           },
           ...
       ]
   }

SENSOR CONVENTIONS
  DSLR:
    Calibration file : cam_parameters.json
    Images subfolder : images_4/
    Image pattern    : frame_*.jpg   (1-indexed sequential)
    cam_key / angle  : derived arithmetically from frame number
                       frames 1..72 = cam1 at 0°..355°, 73..144 = cam2, etc.

  RealSense:
    Calibration file : realsense_cam_parameters.json
    Images subfolder : realsense/
    Image pattern    : rs*_color.png
    cam_key / angle  : parsed directly from filename (rsN_DDD_color.png)

ADDING A NEW SENSOR TYPE
  Add one entry to SENSOR_CONFIGS below and implement a corresponding
  iter_<sensor>_frames() function. No other changes are needed.

COORDINATE CONVENTIONS
    - cam_parameters.json stores w2c in OpenCV convention (X right, Y down, Z fwd).
    - get_camera_pose_from_extrinsics() mirrors live_view_demo.py exactly:
        * applies scale to translation
        * applies CV_TO_GL (identity in current rig — handled elsewhere)
        * composes with a Z-rotation for the turntable angle + calibration offset
    - Object poses are retrieved from PyBullet in world frame, then transformed
      into the OpenCV camera frame to give R_obj_in_cam, t_obj_in_cam.
'''

import os
import re
import sys
import glob
import json
import argparse
import itertools
import numpy as np
import pybullet as p
import cv2
from pathlib import Path

# ---------------------------------------------------------------------------
# Scene replica repo import
# ---------------------------------------------------------------------------

script_dir = Path(__file__).resolve().parent
SCENE_REPLICA_REPO = script_dir / "../scene_replica_moad"
if not os.path.isdir(SCENE_REPLICA_REPO):
    print(f"[ERROR] scene_replica repo not found at: {SCENE_REPLICA_REPO}")
    print("        Update SCENE_REPLICA_REPO at the top of this script.")
    sys.exit(1)

sys.path.insert(0, str(SCENE_REPLICA_REPO))
from scene_replica_notag import TaglessSceneReplica, Rt_to_T, T_inv


# ---------------------------------------------------------------------------
# Sensor configuration table
# ---------------------------------------------------------------------------
# Each entry defines everything needed to locate images and calibration data
# for one sensor type, and where to write its annotations.
# To add a new sensor: add an entry here and implement iter_<sensor>_frames().
#
# Fields:
#   calib_file    : filename within the calibration folder
#   images_subdir : subfolder within the pose folder containing source images
#   image_glob    : glob pattern for finding images within images_subdir
#   output_subdir : subfolder within scene_replica/ for annotation JSON files
#   frame_iter    : name of the iterator function to call (resolved at runtime)

SENSOR_CONFIGS = {
    "dslr": {
        "calib_file":    "cam_parameters.json",
        "images_subdir": "images_4",
        "image_glob":    "frame_*.jpg",
        "output_subdir": "pose_dslr",
        "frame_iter":    "iter_dslr_frames",
    },
    "realsense": {
        "calib_file":    "realsense_cam_parameters.json",
        "images_subdir": "realsense",
        "image_glob":    "rs*_color.png",
        "output_subdir": "pose_realsense",
        "frame_iter":    "iter_realsense_frames",
    },
}


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# DSLR scan layout — must match the collection convention in moad_cui
DSLR_FRAMES_PER_CAM = 72     # frames per camera per full 360° scan
DSLR_STEP_DEG       = 5.0   # turntable degrees between frames

CV_TO_GL  = np.eye(4)   # coordinate convention flip — identity in current rig
ASPECT_TOL = 0.01       # tolerance for aspect ratio mismatch check


# ---------------------------------------------------------------------------
# Math helpers — mirrored from live_view_demo.py
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

    Args:
        cam_key       : camera identifier e.g. "cam1" or "rs3"
        cameras       : dict of cam_key → {"c2w": ndarray, "w2c": ndarray}
        turntable_deg : turntable rotation angle in degrees
        global_offset : calibration XYZ offset (from scene config)
        scale         : COLMAP-to-metric scale factor

    Returns:
        R_cw_cv : (3,3) world-to-camera rotation  (OpenCV convention)
        t_cw_cv : (3,)  world-to-camera translation
    """
    T_w2c = cameras[cam_key]["w2c"].copy()
    T_w2c[:3, 3] *= scale

    T_w2c_cv = CV_TO_GL @ T_w2c

    T_scene = make_z_rotation(turntable_deg)
    T_scene[:3, 3] = global_offset * scale

    T_final = T_w2c_cv @ T_scene
    return T_final[:3, :3], T_final[:3, 3]


def build_cam_K(intrinsics: dict) -> np.ndarray:
    """Build a 3x3 camera intrinsic matrix from an intrinsics dict."""
    return np.array([
        [intrinsics["fx"],             0.0, intrinsics["cx"]],
        [            0.0, intrinsics["fy"], intrinsics["cy"]],
        [            0.0,             0.0,             1.0],
    ], dtype=np.float64)


def check_and_scale_intrinsics(
    intrinsics: dict,
    frame:      np.ndarray,
) -> dict:
    """
    Verify a frame's aspect ratio matches the calibration intrinsics.
    If the resolution differs (e.g. downscaled DSLR images), return a
    uniformly scaled copy of the intrinsics dict. If scale == 1.0, returns
    the original dict unchanged without any redundant computation.

    Raises ValueError on aspect ratio mismatch beyond ASPECT_TOL.
    """
    calib_w  = intrinsics["width"]
    calib_h  = intrinsics["height"]
    frame_h, frame_w = frame.shape[:2]

    calib_ar = calib_w / calib_h
    frame_ar = frame_w / frame_h

    if abs(calib_ar - frame_ar) > ASPECT_TOL:
        raise ValueError(
            f"Aspect ratio mismatch: calibration={calib_w}×{calib_h} ({calib_ar:.4f}) "
            f"vs image={frame_w}×{frame_h} ({frame_ar:.4f})."
        )

    s = frame_w / calib_w
    if abs(s - 1.0) < 1e-6:
        # Resolution matches exactly — no scaling needed
        print(f"  Intrinsics match image resolution ({frame_w}×{frame_h}), no scaling needed.")
        return intrinsics

    print(f"  Scaling intrinsics: {calib_w}×{calib_h} → {frame_w}×{frame_h}  (×{s:.4f})")
    return {
        **intrinsics,
        "fx":     intrinsics["fx"]     * s,
        "fy":     intrinsics["fy"]     * s,
        "cx":     intrinsics["cx"]     * s,
        "cy":     intrinsics["cy"]     * s,
        "width":  frame_w,
        "height": frame_h,
    }


# ---------------------------------------------------------------------------
# Calibration loader
# ---------------------------------------------------------------------------

def load_calibration(calib_root: str, calib_filename: str) -> tuple[dict, dict, float]:
    """
    Load a cam_parameters JSON file and return (cameras, intrinsics, scale).

    Args:
        calib_root     : calibration folder path
        calib_filename : filename within that folder (from SENSOR_CONFIGS)

    Returns:
        cameras    : dict of cam_key → {"c2w": ndarray, "w2c": ndarray}
        intrinsics : intrinsics dict
        scale      : COLMAP-to-metric scale factor
    """
    calib_path = os.path.join(calib_root, calib_filename)
    with open(calib_path, "r") as f:
        cal = json.load(f)

    intrinsics = cal["intrinsics"]
    scale      = cal["_info"]["scaling"]["scale"]

    cameras = {}
    for cam_name, cam_data in cal["cameras"].items():
        cameras[cam_name] = {
            "c2w": np.array(cam_data["extrinsics"]["c2w"], dtype=np.float64),
            "w2c": np.array(cam_data["extrinsics"]["w2c"], dtype=np.float64),
        }

    print(f"  Calibration: {calib_path}")
    print(f"  Cameras:     {sorted(cameras.keys())}")
    print(f"  Scale:       {scale:.8f}")
    return cameras, intrinsics, scale


# ---------------------------------------------------------------------------
# Frame iterators — one per sensor type
# Each yields (img_path, cam_key, turntable_deg) for one image at a time.
# generate_pose_data() consumes these without knowing which sensor they came from.
# ---------------------------------------------------------------------------

def iter_dslr_frames(
    images_dir:     str,
    cameras:        dict,
    frames_per_cam: int   = DSLR_FRAMES_PER_CAM,
    step_deg:       float = DSLR_STEP_DEG,
):
    """
    Yield (img_path, cam_key, turntable_deg) for DSLR frame_NNNNN.jpg images.

    Camera identity and turntable angle are derived arithmetically from the
    1-based frame number:
        frame 1   → cam1, 0°
        frame 72  → cam1, 355°
        frame 73  → cam2, 0°
        frame 360 → cam5, 355°
    """
    image_paths = sorted(glob.glob(os.path.join(images_dir, "frame_*.jpg")))
    if not image_paths:
        raise FileNotFoundError(f"No frame_*.jpg images in: {images_dir}")
    print(f"  Found {len(image_paths)} DSLR images in {images_dir}")

    for img_path in image_paths:
        filename = os.path.basename(img_path)
        match    = re.search(r"frame_(\d+)\.jpg", filename)
        if not match:
            print(f"  [SKIP] Unexpected filename: {filename}")
            continue

        frame_number  = int(match.group(1))           # 1-based
        frame_idx     = frame_number - 1              # 0-based
        cam_number    = (frame_idx // frames_per_cam) + 1
        pos_in_block  = frame_idx %  frames_per_cam
        cam_key       = f"cam{cam_number}"
        turntable_deg = pos_in_block * step_deg

        if cam_key not in cameras:
            print(f"  [SKIP] {filename} → {cam_key} not in calibration")
            continue

        yield img_path, cam_key, turntable_deg


def iter_realsense_frames(
    images_dir: str,
    cameras:    dict,
):
    """
    Yield (img_path, cam_key, turntable_deg) for RealSense rs*_color.png images.

    Both cam_key and turntable angle are parsed directly from the filename:
        rs1_000_color.png → cam_key="rs1", turntable_deg=0.0
        rs3_045_color.png → cam_key="rs3", turntable_deg=45.0

    No arithmetic mapping is needed — the naming convention from the RealSense
    data collection pipeline encodes this information explicitly.
    """
    image_paths = sorted(glob.glob(os.path.join(images_dir, "rs*_color.png")))
    if not image_paths:
        raise FileNotFoundError(f"No rs*_color.png images in: {images_dir}")
    print(f"  Found {len(image_paths)} RealSense images in {images_dir}")

    pattern = re.compile(r"^(rs\d+)_(\d{3})_color\.png$")

    for img_path in image_paths:
        filename = os.path.basename(img_path)
        match    = pattern.match(filename)
        if not match:
            print(f"  [SKIP] Unexpected filename: {filename}")
            continue

        cam_key       = match.group(1)           # e.g. "rs1"
        turntable_deg = float(match.group(2))    # e.g. 45.0

        if cam_key not in cameras:
            print(f"  [SKIP] {filename} → {cam_key} not in calibration")
            continue

        yield img_path, cam_key, turntable_deg


# Registry maps the string names in SENSOR_CONFIGS to the actual functions.
# Add new iterator functions here when adding new sensor types.
FRAME_ITER_REGISTRY = {
    "iter_dslr_frames":       iter_dslr_frames,
    "iter_realsense_frames":  iter_realsense_frames,
}


# ---------------------------------------------------------------------------
# Object pose extraction
# ---------------------------------------------------------------------------

# Module-level cache: populated once per object model per run.
# Keyed by body_name; value is the [sx, sy, sz] full extents in metres.
_size_cache: dict = {}


def _parse_obj_vertices(obj_path: str) -> np.ndarray:
    """Extract vertex positions from a .obj file. Returns (N, 3) float64 array."""
    verts = []
    with open(obj_path, "r") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not verts:
        raise ValueError(f"No vertices found in: {obj_path}")
    return np.array(verts, dtype=np.float64)


def get_object_poses_in_camera_frame(
    R_cw_cv: np.ndarray,
    t_cw_cv: np.ndarray,
) -> list[dict]:
    """
    Query every named body from PyBullet, transform its pose into the OpenCV
    camera frame, and return a list of annotation dicts.

    Object pose in camera frame:
        T_obj_cam = T_cw @ T_obj_world

    Object extents are read from the visual mesh via getVisualShapeData(),
    parsed once and cached in _size_cache.

    Args:
        R_cw_cv : (3,3) world-to-camera rotation  (OpenCV)
        t_cw_cv : (3,)  world-to-camera translation

    Returns:
        List of dicts with keys: object_name, body_id, R, t, size
    """
    T_cw = Rt_to_T(R_cw_cv, t_cw_cv)

    annotations = []
    for i in range(p.getNumBodies()):
        body_id   = p.getBodyUniqueId(i)
        body_name = p.getBodyInfo(body_id)[1].decode("utf-8")
        if body_name == "":
            continue  # skip visual-only markers (circles, bars, etc.)

        pos_w, quat_w = p.getBasePositionAndOrientation(body_id)
        R_mat     = np.array(p.getMatrixFromQuaternion(quat_w)).reshape(3, 3)
        T_obj_w   = Rt_to_T(R_mat, np.array(pos_w))
        T_obj_cam = T_cw @ T_obj_w

        # Resolve object extents from visual mesh (cached after first access)
        if body_name not in _size_cache:
            visual_shapes = p.getVisualShapeData(body_id)
            shape      = visual_shapes[0]
            mesh_path  = shape[4].decode("utf-8")
            mesh_scale = np.array(shape[3])
            verts      = _parse_obj_vertices(mesh_path)
            verts     *= mesh_scale
            _size_cache[body_name] = (verts.max(axis=0) - verts.min(axis=0)).tolist()
            print(f"  [SIZE CACHE] {body_name}: {_size_cache[body_name]}")

        annotations.append({
            "object_name": body_name,
            "body_id":     body_id,
            "R":           T_obj_cam[:3, :3].tolist(),
            "t":           T_obj_cam[:3,  3].tolist(),
            "size":        _size_cache[body_name],
        })

    return annotations


# ---------------------------------------------------------------------------
# Core annotation engine — sensor-agnostic
# ---------------------------------------------------------------------------

def generate_pose_data(
    pose_path:     str,
    scene_replica: TaglessSceneReplica,
    cameras:       dict,
    scene_cfg:     dict,
    scale:         float,
    intrinsics:    dict,
    frame_iter,
    output_subdir: str,
    visualize:     bool = False,
) -> int:
    """
    Sensor-agnostic annotation engine.

    Iterates over frame_iter (which yields (img_path, cam_key, turntable_deg)),
    computes the world-to-camera pose for each frame, queries PyBullet for
    object poses in that camera frame, and writes one JSON annotation file per
    frame to pose_path/scene_replica/<output_subdir>/.

    Annotation files are named after the source image stem so the mapping is
    unambiguous:
        frame_00001.jpg   →  frame_00001.json
        rs1_000_color.png →  rs1_000_color.json

    If visualize=True, scales scene replica render resolution to match the
    actual image resolution (intrinsics are scaled only if needed; scale=1.0
    is a no-op). The first frame is consumed to determine resolution then
    re-injected via itertools.chain so it still gets annotated.

    Args:
        pose_path     : base pose folder path
        scene_replica : TaglessSceneReplica instance (sensor-specific)
        cameras       : calibration cameras dict
        scene_cfg     : scene config dict (for CALIBRATION_OFFSET)
        scale         : COLMAP-to-metric scale factor
        intrinsics    : full-resolution intrinsics dict for this sensor
        frame_iter    : iterator yielding (img_path, cam_key, turntable_deg)
        output_subdir : annotation output subfolder name under scene_replica/
        visualize     : render PyBullet overlay on each frame while processing

    Returns:
        Number of annotation files written.
    """
    output_root = os.path.join(pose_path, "scene_replica", output_subdir)
    os.makedirs(output_root, exist_ok=True)
    print(f"  Output: {output_root}")

    global_offset = np.asarray(scene_cfg["CALIBRATION_OFFSET"], dtype=np.float64)

    # ── Resolve render resolution (visualize only) ────────────────────────────
    if visualize:
        # Peek at the first frame to determine actual image resolution,
        # then re-inject it so it still gets annotated.
        try:
            first = next(frame_iter)
        except StopIteration:
            print("  [WARNING] frame_iter is empty — no annotations generated.")
            return 0

        first_img_path = first[0]
        ref_frame = cv2.imread(first_img_path)
        if ref_frame is None:
            raise RuntimeError(f"Cannot read reference frame: {first_img_path}")

        scaled_intr = check_and_scale_intrinsics(intrinsics, ref_frame)
        if scaled_intr is not intrinsics:
            # Resolution differs — update scene replica projection
            scene_replica.cam_K = build_cam_K(scaled_intr)
            scene_replica.W     = scaled_intr["width"]
            scene_replica.H     = scaled_intr["height"]
            scene_replica._compute_projection_matrix()
            print(f"  Render resolution: {scene_replica.W}×{scene_replica.H}")

        # Re-inject the first frame
        frame_iter = itertools.chain([first], frame_iter)

    # ── Main annotation loop ──────────────────────────────────────────────────
    frame_count = 0
    for img_path, cam_key, turntable_deg in frame_iter:
        filename   = os.path.basename(img_path)
        image_stem = os.path.splitext(filename)[0]

        R_cw_cv, t_cw_cv = get_camera_pose_from_extrinsics(
            cam_key, cameras, turntable_deg, global_offset, scale
        )
        scene_replica.update_camera(R_cw_cv, t_cw_cv)

        if visualize:
            rgba = scene_replica.render_scene_image()
            bgr  = cv2.imread(img_path)
            if bgr is not None:
                rh, rw = scene_replica.H, scene_replica.W
                if bgr.shape[1] != rw or bgr.shape[0] != rh:
                    bgr = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_LINEAR)
                bg    = bgr.astype(np.float32)
                ov    = rgba[:, :, [2, 1, 0, 3]].astype(np.float32)
                alpha = ov[:, :, 3:4] / 255.0
                comp  = np.clip(
                    ov[:, :, :3] * alpha + bg * (1.0 - alpha), 0, 255
                ).astype(np.uint8)
                cv2.imshow("Annotation Verify", comp)
                cv2.waitKey(1)

        objects = get_object_poses_in_camera_frame(R_cw_cv, t_cw_cv)

        annotation = {
            "frame":         filename,
            "cam_key":       cam_key,
            "turntable_deg": turntable_deg,
            "objects":       objects,
        }

        output_path = os.path.join(output_root, f"{image_stem}.json")
        with open(output_path, "w") as f:
            json.dump(annotation, f, indent=2)

        print(f"  {cam_key:>4}  {turntable_deg:>5.1f}°  {filename}"
              f"  → {len(objects)} obj  →  {os.path.basename(output_path)}")
        frame_count += 1

    if visualize:
        cv2.destroyAllWindows()

    print(f"\n  {frame_count} annotation(s) written to: {output_root}\n")
    return frame_count


# ---------------------------------------------------------------------------
# Stub implementations (not yet implemented)
# ---------------------------------------------------------------------------

def generate_bb_data(root_folder, scene_replica):
    print("[NOT IMPLEMENTED] Bounding box annotations")
    input("(Enter to continue)")


def generate_mask_data(root_folder, scene_replica):
    print("[NOT IMPLEMENTED] Mask annotations")
    input("(Enter to continue)")


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------

def validate_inputs(data_root: str, object_name: str, pose_name: str) -> tuple[str, str, str]:
    """
    Validate the expected scene_replica directory structure and return paths
    to the pose folder, scene .npz file, and scene config .json file.

    Expected layout:
        <data_root>/<object_name>/<pose_name>/scene_replica/
            ├── <scene>.npz
            └── <config>.json
    """
    pose_path    = os.path.join(data_root, object_name, pose_name)
    replica_path = os.path.join(pose_path, "scene_replica")

    if not os.path.isdir(pose_path):
        raise FileNotFoundError(f"Pose folder not found: {pose_path}")
    if not os.path.isdir(replica_path):
        raise FileNotFoundError(f"scene_replica folder not found: {replica_path}")

    npz_files  = glob.glob(os.path.join(replica_path, "*.npz"))
    json_files = glob.glob(os.path.join(replica_path, "*.json"))

    if not npz_files:
        raise FileNotFoundError(f"No .npz scene file in: {replica_path}")
    if not json_files:
        raise FileNotFoundError(f"No .json config file in: {replica_path}")

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

    with open(cfg_path, "r") as f:
        scene_cfg = json.load(f)

    scene_file = os.path.basename(npz_path)

    # ── Process each sensor type defined in SENSOR_CONFIGS ───────────────────
    # A sensor is processed only if BOTH its calibration file and image folder
    # are present. Missing either produces a clear skip message.
    for sensor_name, cfg in SENSOR_CONFIGS.items():
        print(f"\n{'─'*60}")
        print(f"  Sensor: {sensor_name.upper()}")
        print(f"{'─'*60}")

        # Check calibration file
        calib_file_path = os.path.join(args.calib_root, args.calibration, cfg["calib_file"])
        if not os.path.isfile(calib_file_path):
            print(f"  [SKIP] Calibration file not found: {calib_file_path}")
            continue

        # Check image folder
        images_dir = os.path.join(pose_path, cfg["images_subdir"])
        if not os.path.isdir(images_dir):
            print(f"  [SKIP] Images folder not found: {images_dir}")
            continue

        # Check at least one image exists
        image_paths = glob.glob(os.path.join(images_dir, cfg["image_glob"]))
        if not image_paths:
            print(f"  [SKIP] No images matching '{cfg['image_glob']}' in: {images_dir}")
            continue

        # ── Load calibration ──────────────────────────────────────────────────
        try:
            cameras, intrinsics, scale = load_calibration(
                os.path.join(args.calib_root, args.calibration), cfg["calib_file"]
            )
        except Exception as e:
            print(f"  [SKIP] Failed to load calibration: {e}")
            continue

        # ── Build frame iterator ──────────────────────────────────────────────
        iter_fn = FRAME_ITER_REGISTRY.get(cfg["frame_iter"])
        if iter_fn is None:
            print(f"  [SKIP] Unknown frame iterator: {cfg['frame_iter']}")
            continue

        try:
            frame_iter = iter_fn(images_dir, cameras)
        except FileNotFoundError as e:
            print(f"  [SKIP] {e}")
            continue

        # ── Initialise scene replica for this sensor ──────────────────────────
        # Each sensor gets its own TaglessSceneReplica instance because they
        # have different intrinsics and render resolutions. PyBullet connects
        # in __init__ and disconnects explicitly after annotation generation.
        initial_offset = np.asarray(scene_cfg["CALIBRATION_OFFSET"], dtype=np.float64)
        first_cam_key  = next(iter(cameras))   # first key from calibration dict

        R_init, t_init = get_camera_pose_from_extrinsics(
            first_cam_key, cameras, 0.0, initial_offset, scale
        )
        scene = TaglessSceneReplica(
            cam_K              = build_cam_K(intrinsics),
            W                  = intrinsics["width"],
            H                  = intrinsics["height"],
            R_cw_cv            = R_init,
            t_cw_cv            = t_init,
            scene_config       = scene_cfg,
            scene_path         = os.path.join(pose_path, "scene_replica"),
            model_library_path = args.model_library,
        )
        scene.load_scene(scene_file)
        print(f"  Scene loaded.\n")

        # ── Generate annotations ──────────────────────────────────────────────
        if args.generate_poses:
            generate_pose_data(
                pose_path     = pose_path,
                scene_replica = scene,
                cameras       = cameras,
                scene_cfg     = scene_cfg,
                scale         = scale,
                intrinsics    = intrinsics,
                frame_iter    = frame_iter,
                output_subdir = cfg["output_subdir"],
                visualize     = args.visualize,
            )

        if args.generate_bbs:
            generate_bb_data(pose_path, scene)

        if args.generate_masks:
            generate_mask_data(pose_path, scene)

        # ── Disconnect PyBullet before next sensor ────────────────────────────
        try:
            p.disconnect(scene.pb)
        except Exception:
            pass

    print("\n  Done.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate ground truth 6D pose annotations for a MOAD scan. "
                    "Processes all sensor types for which both a calibration file "
                    "and image folder are found — no sensor flag needed."
    )

    # ── Target data ───────────────────────────────────────────────────────────
    parser.add_argument("--data-root", default="/home/csrobot/MOAD_DATA",
                        help="Root data directory")
    parser.add_argument("--object",    default="batch1_007",
                        help="Object subfolder name")
    parser.add_argument("--pose",      default="pose-b",
                        help="Pose subfolder name")

    # ── Calibration ───────────────────────────────────────────────────────────
    parser.add_argument("--calib-root", default="/home/csrobot/moad_control/moad_cui/calibration",
                        help="Calibration root folder (contains all calibrations)")
    parser.add_argument("--calibration", default="55mm_joint",
                        help="The specific calibration folder (must contain the cam_parameters files "
                             "listed in SENSOR_CONFIGS)")

    # ── Model library ─────────────────────────────────────────────────────────
    parser.add_argument("--model-library",
                        default="/home/csrobot/moad_control/scene_replica_moad/assets/object_sets/moad-atb1",
                        help="Folder containing URDF models referenced by the scene")

    # ── Output options ────────────────────────────────────────────────────────
    parser.add_argument("--generate-poses",  action="store_true", default=True,
                        help="Generate 6D pose annotations (default: on)")
    parser.add_argument("--generate-bbs",    action="store_true", default=False,
                        help="Generate bounding box annotations (not yet implemented)")
    parser.add_argument("--generate-masks",  action="store_true", default=False,
                        help="Generate segmentation masks (not yet implemented)")
    parser.add_argument("--visualize",       action="store_true", default=True,
                        help="Show PyBullet overlay composited on each image")

    args = parser.parse_args()
    main(args)