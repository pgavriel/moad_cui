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
import time
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

# Module-level caches: populated once per object model per run.
# _size_cache   : body_name → [sx, sy, sz] full extents in metres
# _vertex_cache : body_name → (N,3) float64 vertices in metres (scaled by mesh_scale)
_size_cache:   dict = {}
_vertex_cache: dict = {}


def _parse_obj_vertices(obj_path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Extract vertex positions and triangle face indices from a .obj file.
    Faces are triangulated — quads and n-gons are fan-triangulated from
    vertex 0 of each face.

    Returns:
        verts : (N, 3) float64 vertex positions
        faces : (M, 3) int32 zero-based triangle indices
    """
    verts = []
    faces = []
    with open(obj_path, "r") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith("f "):
                # OBJ indices are 1-based; entries may be "v/vt/vn" format
                parts   = line.split()[1:]
                indices = [int(p.split("/")[0]) - 1 for p in parts]
                # Fan triangulate: (0,1,2), (0,2,3), (0,3,4) ...
                for i in range(1, len(indices) - 1):
                    faces.append([indices[0], indices[i], indices[i + 1]])
    if not verts:
        raise ValueError(f"No vertices found in: {obj_path}")
    return np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int32)


def _bbox_from_mask(mask: np.ndarray) -> list[int] | None:
    """
    Return [x, y, width, height] (top-left origin, BOP convention) for the
    bounding box of all non-zero pixels in `mask`, or None if the mask is empty.
    """
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return None
    row_indices = np.where(rows)[0]
    col_indices = np.where(cols)[0]
    rmin, rmax  = int(row_indices[0]),  int(row_indices[-1])
    cmin, cmax  = int(col_indices[0]),  int(col_indices[-1])
    return [cmin, rmin, cmax - cmin + 1, rmax - rmin + 1]


def compute_visibility_from_seg(
    seg:      np.ndarray,
    body_ids: list[int],
    H:        int,
    W:        int,
) -> dict[int, dict]:
    """
    Extract visible pixel counts and visible bounding boxes for all objects
    in a single pass over the PyBullet segmentation image.

    PyBullet's segmentation image has each pixel set to the body_id of the
    object visible at that pixel, or -1 for background. This is the cheapest
    possible source of per-object visibility — no extra renders needed.

    Args:
        seg      : (H, W) int32 segmentation image from PyBullet
        body_ids : list of body_ids to compute visibility for
        H, W     : image dimensions

    Returns:
        dict keyed by body_id, each value:
            {
                "px_count_visib": int,
                "bbox_visib":     [x, y, w, h] or None
            }
    """
    result = {}
    for body_id in body_ids:
        mask           = (seg == body_id)
        px_count_visib = int(mask.sum())
        bbox_visib     = _bbox_from_mask(mask)
        result[body_id] = {
            "px_count_visib": px_count_visib,
            "bbox_visib":     bbox_visib,
        }
    return result


def compute_full_silhouette(
    body_name:  str,
    R_obj_cam:  np.ndarray,
    t_obj_cam:  np.ndarray,
    cam_K:      np.ndarray,
    H:          int,
    W:          int,
    debug_masks: bool = False,
) -> dict:
    """
    Compute the full object silhouette (including occluded parts) by rasterising
    the projected mesh triangles. More accurate than a convex hull for objects
    with concave geometry (gears, connectors, etc.).

    For each triangle in the mesh, all three vertices are projected through the
    camera. Triangles with any vertex behind the camera are skipped. The
    remaining triangles are filled with cv2.fillConvexPoly (each triangle is
    trivially convex), building up the complete silhouette mask.

    Args:
        body_name   : object name (used to look up _vertex_cache)
        R_obj_cam   : (3,3) object-to-camera rotation
        t_obj_cam   : (3,)  object-to-camera translation
        cam_K       : (3,3) camera intrinsic matrix (at render resolution)
        H, W        : image dimensions
        debug_masks : if True, show the full silhouette mask in a CV window

    Returns:
        {
            "px_count_all" : int,
            "bbox_obj"     : [x, y, w, h] or None,
            "full_mask"    : (H,W) uint8 — included when debug_masks=True, else None
        }
    """
    cached = _vertex_cache.get(body_name)
    if cached is None:
        return {"px_count_all": 0, "bbox_obj": None, "full_mask": None}

    verts_obj, faces = cached
    if len(verts_obj) == 0:
        return {"px_count_all": 0, "bbox_obj": None, "full_mask": None}

    # Project all vertices to image plane in one batched operation
    verts_cam = (R_obj_cam @ verts_obj.T).T + t_obj_cam     # (N, 3) camera space
    valid     = verts_cam[:, 2] > 0                          # in-front mask

    px_h = np.zeros((len(verts_obj), 3), dtype=np.float64)
    if valid.any():
        px_h[valid] = (cam_K @ verts_cam[valid].T).T

    px_all = np.zeros((len(verts_obj), 2), dtype=np.float32)

    # COUNT PROJECTED PIXELS WHICH ARE OUT OF FRAME INTO THE TOTAL
    px_all[valid, 0] = px_h[valid, 0] / px_h[valid, 2]   # no clip
    px_all[valid, 1] = px_h[valid, 1] / px_h[valid, 2]   # no clip

    # Determine how far vertices extend beyond the image
    x_min = int(np.floor(px_all[valid, 0].min())) if valid.any() else 0
    y_min = int(np.floor(px_all[valid, 1].min())) if valid.any() else 0
    x_max = int(np.ceil( px_all[valid, 0].max())) if valid.any() else W
    y_max = int(np.ceil( px_all[valid, 1].max())) if valid.any() else H

    # Pad the canvas to cover out-of-frame extent
    pad_left  = max(0, -x_min)
    pad_top   = max(0, -y_min)
    pad_right  = max(0, x_max - (W - 1))
    pad_bottom = max(0, y_max - (H - 1))

    canvas_W = W + pad_left + pad_right
    canvas_H = H + pad_top  + pad_bottom

    # Offset all projected coordinates into the padded canvas
    px_canvas = px_all.copy()
    px_canvas[:, 0] += pad_left
    px_canvas[:, 1] += pad_top

    # Rasterise into the padded canvas
    mask_full = np.zeros((canvas_H, canvas_W), dtype=np.uint8)
    if len(faces) > 0:
        v0, v1, v2  = faces[:, 0], faces[:, 1], faces[:, 2]
        tri_valid   = valid[v0] & valid[v1] & valid[v2]
        for tri in faces[tri_valid]:
            pts = px_canvas[tri].astype(np.int32)
            cv2.fillConvexPoly(mask_full, pts, 1)

    px_count_all = int(mask_full.sum())   # true total including out-of-frame

    # For bbox_obj, crop back to image region and use only in-frame pixels
    mask_inframe = mask_full[pad_top:pad_top + H, pad_left:pad_left + W]
    bbox_obj = _bbox_from_mask(mask_inframe)

    # Optional: display the silhouette mask for debugging
    if debug_masks:
        display = np.zeros((H, W, 3), dtype=np.uint8)
        display[:, :, 1] = mask_inframe * 255   # green channel = full silhouette
        cv2.imshow(f"Full silhouette - {body_name}", display)
        cv2.waitKey(0)

    return {
        "px_count_all": px_count_all,
        "bbox_obj":     bbox_obj,
        "full_mask":    mask_inframe if debug_masks else None,
    }


def get_object_poses_in_camera_frame(
    R_cw_cv:     np.ndarray,
    t_cw_cv:     np.ndarray,
    seg:         np.ndarray | None = None,
    cam_K:       np.ndarray | None = None,
    H:           int = 0,
    W:           int = 0,
    debug_masks: bool = False,
) -> list[dict]:
    """
    Query every named body from PyBullet, transform its pose into the OpenCV
    camera frame, and return a list of annotation dicts.

    Object pose in camera frame:
        T_obj_cam = T_cw @ T_obj_world

    Object extents are read from the visual mesh via getVisualShapeData(),
    parsed once and cached in _size_cache / _vertex_cache.

    When `seg`, `cam_K`, `H`, and `W` are provided, the following additional
    fields are computed per object at negligible extra cost:
        - px_count_visib / bbox_visib  : from the segmentation image (single pass)
        - px_count_all   / bbox_obj    : from mesh vertex projection (no extra render)
        - visib_fract                  : px_count_visib / px_count_all

    Args:
        R_cw_cv : (3,3) world-to-camera rotation  (OpenCV)
        t_cw_cv : (3,)  world-to-camera translation
        seg     : (H,W) int32 PyBullet segmentation image, or None to skip
        cam_K   : (3,3) camera intrinsic matrix at render resolution, or None
        H, W    : render image dimensions

    Returns:
        List of dicts with keys: object_name, body_id, R, t, size,
        and optionally: px_count_all, px_count_visib, visib_fract,
                        bbox_obj, bbox_visib
    """
    T_cw      = Rt_to_T(R_cw_cv, t_cw_cv)
    compute_vis = (seg is not None and cam_K is not None and H > 0 and W > 0)

    # Collect all named bodies first so we can do the seg pass in one shot
    named_bodies = []
    for i in range(p.getNumBodies()):
        body_id   = p.getBodyUniqueId(i)
        body_name = p.getBodyInfo(body_id)[1].decode("utf-8")
        if body_name == "":
            continue  # skip visual-only markers

        # Populate mesh caches on first encounter
        if body_name not in _size_cache:
            visual_shapes = p.getVisualShapeData(body_id)
            shape      = visual_shapes[0]
            mesh_path  = shape[4].decode("utf-8")
            mesh_scale = np.array(shape[3])
            verts, faces = _parse_obj_vertices(mesh_path)
            verts       *= mesh_scale
            _size_cache[body_name]   = (verts.max(axis=0) - verts.min(axis=0)).tolist()
            _vertex_cache[body_name] = (verts, faces)   # (N,3) verts + (M,3) face indices
            print(f"  [CACHE] {body_name}: size={[f'{v:.4f}' for v in _size_cache[body_name]]}"
                  f"  verts={len(verts)}  faces={len(faces)}")

        named_bodies.append((body_id, body_name))

    # Single-pass segmentation visibility for all objects at once
    vis_data = {}
    if compute_vis:
        body_ids = [bid for bid, _ in named_bodies]
        vis_data = compute_visibility_from_seg(seg, body_ids, H, W)

    # Build annotation list
    annotations = []
    for body_id, body_name in named_bodies:
        pos_w, quat_w = p.getBasePositionAndOrientation(body_id)
        R_mat     = np.array(p.getMatrixFromQuaternion(quat_w)).reshape(3, 3)
        T_obj_w   = Rt_to_T(R_mat, np.array(pos_w))
        T_obj_cam = T_cw @ T_obj_w

        R_obj_cam = T_obj_cam[:3, :3]
        t_obj_cam = T_obj_cam[:3,  3]

        ann = {
            "object_name": body_name,
            "body_id":     body_id,
            "R":           R_obj_cam.tolist(),
            "t":           t_obj_cam.tolist(),
            "size":        _size_cache[body_name],
        }

        if compute_vis:
            # Visible fields from segmentation (free — already computed above)
            vd = vis_data.get(body_id, {})
            px_visib = vd.get("px_count_visib", 0)
            bbox_vis = vd.get("bbox_visib", None)

            # Full silhouette from mesh triangle rasterisation (no extra render)
            sil = compute_full_silhouette(
                body_name, R_obj_cam, t_obj_cam, cam_K, H, W,
                debug_masks=debug_masks,
            )
            px_all   = sil["px_count_all"]
            bbox_obj = sil["bbox_obj"]

            visib_fract = (px_visib / px_all) if px_all > 0 else 0.0

            # Optionally show the visible seg mask alongside the full silhouette
            if debug_masks:
                vis_mask = (seg == body_id).astype(np.uint8)
                display  = np.zeros((H, W, 3), dtype=np.uint8)
                display[:, :, 1] = sil["full_mask"] * 255 if sil["full_mask"] is not None else 0
                display[:, :, 2] = vis_mask * 255
                # Result: green = full silhouette only, yellow = visible (both channels)
                cv2.imshow(f"Visibility - {body_name}-{body_id}", display)
                cv2.waitKey(1)
                print(f"  [DEBUG] {body_name}: "
                      f"px_all={px_all}  px_visib={px_visib}  "
                      f"visib_fract={visib_fract:.3f}")

            ann.update({
                "px_count_all":   px_all,
                "px_count_visib": px_visib,
                "visib_fract":    round(visib_fract, 6),
                "bbox_obj":       bbox_obj,
                "bbox_visib":     bbox_vis,
            })

        annotations.append(ann)

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
    debug_masks:   bool = False,
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

    # ── Resolve render resolution ─────────────────────────────────────────────
    # Always peek at the first frame to determine actual image resolution so
    # intrinsics are correctly scaled for visibility computation regardless of
    # whether visualize is enabled.
    try:
        first = next(frame_iter)
    except StopIteration:
        print("  [WARNING] frame_iter is empty — no annotations generated.")
        return 0

    ref_frame = cv2.imread(first[0])
    if ref_frame is None:
        raise RuntimeError(f"Cannot read reference frame: {first[0]}")

    scaled_intr = check_and_scale_intrinsics(intrinsics, ref_frame)
    if scaled_intr is not intrinsics:
        scene_replica.cam_K = build_cam_K(scaled_intr)
        scene_replica.W     = scaled_intr["width"]
        scene_replica.H     = scaled_intr["height"]
        scene_replica._compute_projection_matrix()
        print(f"  Render resolution: {scene_replica.W}×{scene_replica.H}")

    frame_iter = itertools.chain([first], frame_iter)

    render_W = scene_replica.W
    render_H = scene_replica.H
    render_K = scene_replica.cam_K

    # ── Visual marker handling ────────────────────────────────────────────────
    # Collect all unnamed bodies (visual markers — circles, X indicators, etc.)
    # and save their original colours. These bodies appear in the segmentation
    # image and would incorrectly reduce visible pixel counts for real objects.
    #
    # Strategy depends on the visualize flag:
    #   visualize=False → hide markers permanently now; single render per frame,
    #                     zero per-frame overhead.
    #   visualize=True  → hide markers only during the computation render, then
    #                     restore them for the display render each frame.
    marker_ids       = []
    marker_originals = {}
    for i in range(p.getNumBodies()):
        bid  = p.getBodyUniqueId(i)
        name = p.getBodyInfo(bid)[1].decode("utf-8")
        if name == "":
            marker_ids.append(bid)
            visual_data = p.getVisualShapeData(bid)
            marker_originals[bid] = (
                list(visual_data[0][7]) if visual_data else [1, 0, 0, 1]
            )

    if not visualize and marker_ids:
        # Hide permanently — no restore needed, no per-frame cost
        for bid in marker_ids:
            p.changeVisualShapeColor(bid, -1, rgbaColor=[0, 0, 0, 0])
        print(f"  Visual markers hidden ({len(marker_ids)}) — visualize=False")

    # ── Main annotation loop ──────────────────────────────────────────────────
    frame_count   = 0
    total_start   = time.perf_counter()
    frame_times   = []

    for img_path, cam_key, turntable_deg in frame_iter:
        frame_start = time.perf_counter()
        filename    = os.path.basename(img_path)
        image_stem  = os.path.splitext(filename)[0]

        R_cw_cv, t_cw_cv = get_camera_pose_from_extrinsics(
            cam_key, cameras, turntable_deg, global_offset, scale
        )
        scene_replica.update_camera(R_cw_cv, t_cw_cv)

        if visualize:
            # Render 1 — full scene including markers, for display overlay
            rgba = scene_replica.render_scene_image()

            # Hide markers, render again for clean segmentation
            for bid in marker_ids:
                p.changeVisualShapeColor(bid, -1, rgbaColor=[0, 0, 0, 0])
            scene_replica.render_scene_image()
            seg = scene_replica.seg   # clean segmentation, no marker pixels

            # Restore markers for next frame's display render
            for bid in marker_ids:
                p.changeVisualShapeColor(bid, -1, rgbaColor=marker_originals[bid])

            # Show display overlay
            bgr = cv2.imread(img_path)
            if bgr is not None:
                if bgr.shape[1] != render_W or bgr.shape[0] != render_H:
                    bgr = cv2.resize(bgr, (render_W, render_H),
                                     interpolation=cv2.INTER_LINEAR)
                bg    = bgr.astype(np.float32)
                ov    = rgba[:, :, [2, 1, 0, 3]].astype(np.float32)
                alpha = ov[:, :, 3:4] / 255.0
                comp  = np.clip(
                    ov[:, :, :3] * alpha + bg * (1.0 - alpha), 0, 255
                ).astype(np.uint8)
                cv2.imshow("Annotation Verify", comp)
                cv2.waitKey(1)
        else:
            # Markers already permanently hidden — single render, clean seg
            rgba = scene_replica.render_scene_image()
            seg  = scene_replica.seg

        objects = get_object_poses_in_camera_frame(
            R_cw_cv, t_cw_cv,
            seg         = seg,
            cam_K       = render_K,
            H           = render_H,
            W           = render_W,
            debug_masks = debug_masks,
        )

        # ── 1. Image dimensions included for self-contained annotations ───────
        annotation = {
            "frame":         filename,
            "cam_key":       cam_key,
            "turntable_deg": turntable_deg,
            "width":         render_W,
            "height":        render_H,
            "objects":       objects,
        }

        output_path = os.path.join(output_root, f"{image_stem}.json")
        with open(output_path, "w") as f:
            json.dump(annotation, f, indent=2)

        frame_time = time.perf_counter() - frame_start
        frame_times.append(frame_time)
        frame_count += 1

        print(f"  {cam_key:>4}  {turntable_deg:>5.1f}°  {filename}"
              f"  → {len(objects)} obj"
              f"  [{frame_time*1000:.0f}ms]"
              f"  →  {os.path.basename(output_path)}")

    if visualize:
        cv2.destroyAllWindows()

    # ── Timing summary ────────────────────────────────────────────────────────
    total_s   = time.perf_counter() - total_start
    avg_ms    = (sum(frame_times) / len(frame_times) * 1000) if frame_times else 0.0
    total_min = int(total_s // 60)
    total_sec = total_s % 60

    print(f"\n  {'─'*50}")
    print(f"  Annotations written : {frame_count}")
    print(f"  Output folder       : {output_root}")
    print(f"  Average frame time  : {avg_ms:.0f} ms")
    print(f"  Total time          : {total_min}m {total_sec:.1f}s")
    print(f"  {'─'*50}\n")

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
                debug_masks   = args.debug_masks,
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
    parser.add_argument("--debug-masks",     action="store_true", default=False,
                        help="[Not working] Show visibility mask debug windows (full silhouette vs "
                             "visible seg mask) for each object each frame. "
                             "Slows down generation — use on a single frame for debugging.")

    args = parser.parse_args()
    main(args)