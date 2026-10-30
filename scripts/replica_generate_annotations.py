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

FRAME IDS
    Every frame has a 1-based frame_id equal to its position in the sorted
    image list — the same number the BOP converter uses as image_id, the [i/N]
    index in vis_bop_predictions.py, and vis_index in analyze_poses.py:
      DSLR      : the number in frame_NNNNN.jpg
      RealSense : position in the sorted rs*_color.png list (camera-major, so
                  frame_id N is the same camera / turntable angle as DSLR N)
    IDs are assigned BEFORE any frame is skipped, so they never drift.

DEBUG MODE  (--debug-frames 1 137 222)
    Processes ONLY the listed frames and writes detailed diagnostics instead of
    annotations. Real annotation files are never touched in this mode.
    Output: <debug-dir>/<sensor>/{frame_id:06d}_{description}.{ext}
        _info.json               everything numeric (see write_frame_debug)
        _annotation.json         the annotation the normal run would write
        _image.jpg / _render.png / _overlay.png / _seg.png / _depth.png
        _<object>_mask_full.png      projected silhouette  (the denominator)
        _<object>_mask_visible.png   PyBullet seg mask     (the numerator)
        _<object>_breakdown.png      every silhouette pixel coloured by what
                                     the seg image says is there
        _<object>_crop.png           breakdown around the object, zoomed
    The per-sensor subfolder keeps DSLR and RealSense output apart; frame ids
    match across sensors, so 000137 in both folders is the same view.

VISIBILITY  (visib_fract = px_count_visib / px_count_all)
    Both counts come from PyBullet, on the same pixel grid, with the same
    rasteriser — so for an unoccluded object they agree exactly:
      px_count_visib : pixels labelled with the object in the normal scene render
      px_count_all   : pixels of the object rendered ALONE (every other body,
                       objects and markers, moved out of view)
    The in-frame part of each full mask is rendered at EXACTLY the scene
    render's size with EXACTLY its projection matrix. That matters: the
    hardware (EGL) renderer does not land renders of different sizes on the
    same pixel grid (measured: ~0.5 px shift between a full-image render and a
    cropped-window render of the same view; TinyRenderer is exact either way).
    Identical render calls are identical on both renderers.
    When an object extends past the image border, a second render of a window
    around its projected extent counts the pixels OUTSIDE the image, so
    truncated parts still count — as in BOP, truncation lowers visibility.
    --full-mask-method projection restores the old cv2 mesh rasterisation.

MARKERS
    Visual-only markers are excluded from every annotation render by MOVING
    them out of view, not by setting alpha to 0: whether an alpha-0 body still
    writes to the seg / depth buffers differs between TinyRenderer and the
    hardware (EGL) renderer.

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
import contextlib
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
    Yield (frame_id, img_path, cam_key, turntable_deg) for DSLR frame_NNNNN.jpg
    images. frame_id is the 1-based number in the filename.

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

        yield frame_number, img_path, cam_key, turntable_deg


def iter_realsense_frames(
    images_dir: str,
    cameras:    dict,
):
    """
    Yield (frame_id, img_path, cam_key, turntable_deg) for RealSense
    rs*_color.png images. frame_id is the 1-based position in the sorted file
    list, assigned before any skip so it matches the BOP converter's image_id.

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

    for frame_id, img_path in enumerate(image_paths, start=1):
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

        yield frame_id, img_path, cam_key, turntable_deg


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
# _mesh_info_cache : body_name → mesh file, scale, vertex/face counts, and the
#                    local visual / inertial frame offsets (debug only: a
#                    non-zero offset means the silhouette is projected from a
#                    different origin than PyBullet renders from)
_mesh_info_cache: dict = {}


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
    keep_mask:  bool = False,
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
        keep_mask   : if True, also return the in-frame mask (debug mode)

    Returns:
        {
            "px_count_all"   : int   (includes out-of-frame pixels),
            "bbox_obj"       : [x, y, w, h] or None,
            "full_mask"      : (H,W) uint8 in-frame mask if keep_mask, else None,
            "n_tris_drawn"   : int,
            "silhouette_ms"  : float,
        }
    """
    t_start = time.perf_counter()
    empty = {"px_count_all": 0, "bbox_obj": None, "full_mask": None,
             "n_tris_drawn": 0, "silhouette_ms": 0.0}
    cached = _vertex_cache.get(body_name)
    if cached is None:
        return empty

    verts_obj, faces = cached
    if len(verts_obj) == 0:
        return empty

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
    n_tris_drawn = 0
    if len(faces) > 0:
        v0, v1, v2  = faces[:, 0], faces[:, 1], faces[:, 2]
        tri_valid   = valid[v0] & valid[v1] & valid[v2]
        n_tris_drawn = int(tri_valid.sum())
        # NOTE (known issue, kept as-is so debug output reflects production):
        # astype(int32) truncates sub-pixel vertex positions, and
        # fillConvexPoly fills every pixel an edge touches, whereas PyBullet
        # samples pixel centres. Both inflate px_count_all, most for small objects.
        for tri in faces[tri_valid]:
            pts = px_canvas[tri].astype(np.int32)
            cv2.fillConvexPoly(mask_full, pts, 1)

    px_count_all = int(mask_full.sum())   # true total including out-of-frame

    # For bbox_obj, crop back to image region and use only in-frame pixels
    mask_inframe = mask_full[pad_top:pad_top + H, pad_left:pad_left + W]
    bbox_obj = _bbox_from_mask(mask_inframe)

    return {
        "px_count_all":  px_count_all,
        "bbox_obj":      bbox_obj,
        "full_mask":     mask_inframe.copy() if keep_mask else None,
        "n_tris_drawn":  n_tris_drawn,
        "silhouette_ms": (time.perf_counter() - t_start) * 1000.0,
    }


# ---------------------------------------------------------------------------
# PyBullet full masks — each object rendered alone
# ---------------------------------------------------------------------------

# Bodies are "parked" at (0, 0, -PARK_FACTOR * far): far beyond the far clip
# plane of any camera within reach of the turntable, so they render nowhere.
PARK_FACTOR = 10.0
# Extra pixels around an object's projected extent in its full-mask window.
FULL_MASK_PAD = 3
# Cap on a full-mask window side, as a multiple of the image's larger side.
# Only reached when an object nearly touches the camera plane.
FULL_MASK_MAX_FACTOR = 4


def park_position(scene_replica) -> list[float]:
    return [0.0, 0.0, -PARK_FACTOR * float(scene_replica.far)]


@contextlib.contextmanager
def parked(body_ids, park_pos):
    """Move bodies out of view for the duration of the block, then restore
    their exact poses (base frame in, base frame out — lossless)."""
    saved = {b: p.getBasePositionAndOrientation(b) for b in body_ids}
    try:
        for b in body_ids:
            p.resetBasePositionAndOrientation(b, park_pos, [0, 0, 0, 1])
        yield
    finally:
        for b, (pos, orn) in saved.items():
            p.resetBasePositionAndOrientation(b, pos, orn)


def _window_projection(scene_replica, cam_K: np.ndarray, x0: int, y0: int,
                       w: int, h: int):
    """
    Projection matrix covering the pixel window [x0, x0+w) x [y0, y0+h) of
    the full image. Identical to TaglessSceneReplica._compute_projection_matrix
    with the principal point shifted by the window origin, so window pixels
    coincide exactly with full-render pixels.
    """
    near, far = scene_replica.near, scene_replica.far
    fx, fy = cam_K[0, 0], cam_K[1, 1]
    cx, cy = cam_K[0, 2] - x0, cam_K[1, 2] - y0
    left = -cx * near / fx
    right = (w - cx) * near / fx
    bottom = -(h - cy) * near / fy
    top = cy * near / fy
    return p.computeProjectionMatrix(left, right, bottom, top, near, far)


def _projected_window(verts_obj, R_obj_cam, t_obj_cam, cam_K, W, H):
    """Integer pixel window enclosing the object's projected vertices, or None
    if the object is entirely behind the camera."""
    vc = verts_obj @ R_obj_cam.T + t_obj_cam
    vc = vc[vc[:, 2] > 1e-6]
    if len(vc) == 0:
        return None
    uv = (vc @ cam_K.T)[:, :2] / vc[:, 2:3]
    x0 = int(np.floor(uv[:, 0].min())) - FULL_MASK_PAD
    y0 = int(np.floor(uv[:, 1].min())) - FULL_MASK_PAD
    x1 = int(np.ceil(uv[:, 0].max())) + FULL_MASK_PAD + 1
    y1 = int(np.ceil(uv[:, 1].max())) + FULL_MASK_PAD + 1
    cap = FULL_MASK_MAX_FACTOR * max(W, H)
    x0, y0 = max(x0, -cap), max(y0, -cap)
    x1, y1 = min(x1, W + cap), min(y1, H + cap)
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def render_full_masks(
    scene_replica,
    targets:     list[tuple[int, str, np.ndarray, np.ndarray]],
    all_bodies:  list[int],
    cam_K:       np.ndarray,
    H:           int,
    W:           int,
    keep_mask:   bool = False,
) -> dict[int, dict]:
    """
    Full (unoccluded) mask of each target, rendered by PyBullet with every
    other body parked out of view.

    targets    : (body_id, name, R_obj_cam, t_obj_cam) per object
    all_bodies : every body in the scene (objects + markers) — all are parked,
                 then each target is brought back alone for its render(s)

    Per object:
      1. In-frame mask: the SAME getCameraImage call as the scene render
         (size, view and projection matrices), so it shares the scene seg's
         pixel grid exactly on every renderer.
      2. Only if the object's projected extent leaves the image: a window
         render around it, used purely to count pixels OUTSIDE the image.

    True poses are read before parking and restored afterwards by parked(),
    so the scene is left exactly as it was found.

    Returns per body_id:
        px_count_all    : object pixels, INCLUDING any outside the image
        out_of_frame_px : the part of px_count_all outside the image
        bbox_obj        : [x, y, w, h] of the in-frame part, or None
        full_mask       : (H, W) uint8 in-frame mask if keep_mask, else None
        window          : [x0, y0, x1, y1] of the truncation render, or None
        renders         : PyBullet renders used (1, or 2 when truncated)
        render_ms       : time for this object
    """
    renderer = getattr(p, scene_replica.PB_RENDER)
    park = park_position(scene_replica)
    view, proj = scene_replica.view_matrix, scene_replica.projection_matrix
    true_pose = {b: p.getBasePositionAndOrientation(b) for b, *_ in targets}
    out: dict[int, dict] = {}

    with parked(all_bodies, park):
        for body_id, name, R_oc, t_oc in targets:
            t0 = time.perf_counter()
            rec = {"px_count_all": 0, "out_of_frame_px": 0, "bbox_obj": None,
                   "full_mask": None, "window": None, "renders": 0, "render_ms": 0.0}
            p.resetBasePositionAndOrientation(body_id, *true_pose[body_id])

            # 1. In-frame mask on the scene's exact pixel grid
            img = p.getCameraImage(W, H, viewMatrix=view, projectionMatrix=proj,
                                   shadow=0, renderer=renderer)
            inframe = (np.asarray(img[4], dtype=np.int64).reshape(H, W) == body_id)
            rec["renders"] = 1

            # 2. Truncated? Count the pixels outside the image with a window render.
            cached = _vertex_cache.get(name)
            win = (_projected_window(cached[0], R_oc, t_oc, cam_K, W, H)
                   if cached is not None and len(cached[0]) else None)
            if win is not None and (win[0] < 0 or win[1] < 0 or win[2] > W or win[3] > H):
                x0, y0, x1, y1 = win
                w, h = x1 - x0, y1 - y0
                wimg = p.getCameraImage(
                    w, h, viewMatrix=view,
                    projectionMatrix=_window_projection(scene_replica, cam_K, x0, y0, w, h),
                    shadow=0, renderer=renderer)
                sub = np.asarray(wimg[4], dtype=np.int64).reshape(h, w) == body_id
                ys, xs = np.nonzero(sub)
                xs, ys = xs + x0, ys + y0
                rec["out_of_frame_px"] = int(((xs < 0) | (xs >= W) | (ys < 0) | (ys >= H)).sum())
                rec["window"] = [x0, y0, x1, y1]
                rec["renders"] = 2

            p.resetBasePositionAndOrientation(body_id, park, [0, 0, 0, 1])

            rec["px_count_all"] = int(inframe.sum()) + rec["out_of_frame_px"]
            rec["bbox_obj"] = _bbox_from_mask(inframe.astype(np.uint8))
            if keep_mask:
                rec["full_mask"] = inframe.astype(np.uint8)
            rec["render_ms"] = (time.perf_counter() - t0) * 1000.0
            out[body_id] = rec
    return out


def visibility_breakdown(
    full_mask:  np.ndarray,
    seg:        np.ndarray,
    body_id:    int,
    body_names: dict[int, str],
    marker_ids: set[int],
) -> dict:
    """
    Attribute every in-frame silhouette pixel to what the seg image shows there.

    For a fully visible object everything should land in "visible". The other
    buckets each point at a different cause:
        occluded_by_object : genuine occlusion — or interpenetration / z-fighting
        occluded_by_marker : visual-only markers still writing to the seg image
        background         : seg is empty where the silhouette says the object
                             is — the two renderers DISAGREE (rasterisation or
                             projection mismatch), not occlusion
        other_id           : seg ids that are neither objects nor markers
        seg_outside_silhouette : object pixels in seg the silhouette misses —
                             also a renderer disagreement, in the other direction
    """
    full = full_mask.astype(bool)
    own  = seg == body_id
    under = seg[full]                       # seg value at every silhouette pixel

    occl_obj: dict[str, int] = {}
    occl_mrk: dict[str, int] = {}
    other = 0
    ids, counts = np.unique(under, return_counts=True)
    for sid, n in zip(ids.tolist(), counts.tolist()):
        if sid == body_id or sid == -1:
            continue
        if sid in body_names:
            occl_obj[f"{body_names[sid]} ({sid})"] = n
        elif sid in marker_ids:
            occl_mrk[str(sid)] = n
        else:
            other += n

    inter = int((full & own).sum())
    union = int((full | own).sum())
    cent = None
    if full.any() and own.any():
        fy, fx = np.argwhere(full).mean(axis=0)
        oy, ox = np.argwhere(own).mean(axis=0)
        cent = [round(float(ox - fx), 3), round(float(oy - fy), 3)]

    return {
        "silhouette_px_in_frame": int(full.sum()),
        "visible":                inter,
        "occluded_by_object":     occl_obj,
        "occluded_by_marker":     occl_mrk,
        "background":             int((under == -1).sum()),
        "other_id":               other,
        "seg_outside_silhouette": int((own & ~full).sum()),
        "iou_silhouette_vs_seg":  round(inter / union, 4) if union else None,
        "centroid_offset_px_seg_minus_silhouette": cent,
    }


def get_object_poses_in_camera_frame(
    R_cw_cv:     np.ndarray,
    t_cw_cv:     np.ndarray,
    seg:         np.ndarray | None = None,
    cam_K:       np.ndarray | None = None,
    H:           int = 0,
    W:           int = 0,
    debug:       dict | None = None,
    marker_ids:  set[int] | None = None,
    scene_replica = None,
    full_mask_method: str = "pybullet",
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
        debug   : optional dict. When given, it is filled with per-object
                  diagnostics keyed by body_id: masks, the occlusion
                  breakdown, mesh info and timings. The returned annotation
                  list is identical with or without it.
        marker_ids : body ids of visual-only markers (for the breakdown)
        scene_replica    : needed for full_mask_method="pybullet"
        full_mask_method : "pybullet" (each object rendered alone; default) or
                           "projection" (legacy cv2 mesh rasterisation)

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
            # Debug metadata. getBasePositionAndOrientation() returns the base
            # INERTIAL frame; the silhouette uses raw mesh coordinates. If the
            # local visual frame or inertial offset is non-zero, the two differ.
            dyn = p.getDynamicsInfo(body_id, -1)
            _mesh_info_cache[body_name] = {
                "mesh_file":                 mesh_path,
                "mesh_scale":                mesh_scale.tolist(),
                "n_visual_shapes":           len(visual_shapes),
                "n_vertices":                int(len(verts)),
                "n_faces":                   int(len(faces)),
                "local_visual_frame_pos":    list(shape[5]),
                "local_visual_frame_orn":    list(shape[6]),
                "local_inertial_pos":        list(dyn[3]),
                "local_inertial_orn":        list(dyn[4]),
            }
            print(f"  [CACHE] {body_name}: size={[f'{v:.4f}' for v in _size_cache[body_name]]}"
                  f"  verts={len(verts)}  faces={len(faces)}")

        named_bodies.append((body_id, body_name))

    # Single-pass segmentation visibility for all objects at once
    vis_data = {}
    if compute_vis:
        body_ids = [bid for bid, _ in named_bodies]
        vis_data = compute_visibility_from_seg(seg, body_ids, H, W)

    # Object poses in the camera frame (needed by both full-mask methods)
    poses_cam = {}
    for body_id, body_name in named_bodies:
        pos_w, quat_w = p.getBasePositionAndOrientation(body_id)
        R_mat     = np.array(p.getMatrixFromQuaternion(quat_w)).reshape(3, 3)
        T_obj_cam = T_cw @ Rt_to_T(R_mat, np.array(pos_w))
        poses_cam[body_id] = (T_obj_cam[:3, :3], T_obj_cam[:3, 3])

    # Full masks rendered by PyBullet: every body parked, each object alone
    full_masks = {}
    if compute_vis and full_mask_method == "pybullet":
        if scene_replica is None:
            raise ValueError("full_mask_method='pybullet' needs scene_replica")
        all_bodies = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())]
        full_masks = render_full_masks(
            scene_replica,
            [(bid, nm, *poses_cam[bid]) for bid, nm in named_bodies],
            all_bodies, cam_K, H, W, keep_mask=debug is not None)

    # Build annotation list
    annotations = []
    for body_id, body_name in named_bodies:
        R_obj_cam, t_obj_cam = poses_cam[body_id]

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

            if full_mask_method == "pybullet":
                fm = full_masks[body_id]
                sil = {"px_count_all": fm["px_count_all"], "bbox_obj": fm["bbox_obj"],
                       "full_mask": fm["full_mask"], "n_tris_drawn": None,
                       "silhouette_ms": fm["render_ms"], "window": fm["window"],
                       "renders": fm["renders"]}
            else:
                # Legacy: full silhouette from cv2 mesh rasterisation
                sil = compute_full_silhouette(
                    body_name, R_obj_cam, t_obj_cam, cam_K, H, W,
                    keep_mask=debug is not None,
                )
                sil["window"] = None
            px_all   = sil["px_count_all"]
            bbox_obj = sil["bbox_obj"]

            visib_fract = (px_visib / px_all) if px_all > 0 else 0.0

            if debug is not None:
                names = {bid: nm for bid, nm in named_bodies}
                debug[body_id] = {
                    "object_name":  body_name,
                    "full_mask":    sil["full_mask"],
                    "visible_mask": (seg == body_id).astype(np.uint8),
                    "breakdown":    visibility_breakdown(
                        sil["full_mask"], seg, body_id, names, marker_ids or set()),
                    "out_of_frame_px": px_all - int(sil["full_mask"].sum()),
                    "n_tris_drawn":  sil["n_tris_drawn"],
                    "silhouette_ms": round(sil["silhouette_ms"], 2),
                    "full_mask_method": full_mask_method,
                    "full_mask_window": sil["window"],
                    "full_mask_renders": sil.get("renders"),
                    "mesh":          _mesh_info_cache.get(body_name, {}),
                }

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
# Debug output — only used with --debug-frames
# ---------------------------------------------------------------------------

# Breakdown colours (BGR). Each bucket of visibility_breakdown() has one.
BREAKDOWN_COLORS = {
    "visible":                (60, 200, 60),     # green   — silhouette AND seg agree
    "occluded_by_object":     (230, 120, 30),    # blue    — another object in front
    "occluded_by_marker":     (230, 40, 230),    # magenta — a hidden marker in seg
    "background":             (40, 40, 230),     # red     — seg empty: renderers disagree
    "other_id":               (0, 200, 230),     # yellow  — unknown seg id
    "seg_outside_silhouette": (240, 240, 240),   # white   — seg has object, silhouette doesn't
}
DEBUG_CROP_PAD  = 6       # px of context around the object in _crop.png
DEBUG_CROP_ZOOM = 8       # nearest-neighbour zoom factor for _crop.png
DEBUG_CROP_MAX  = 1600    # cap on the zoomed crop's longest side
_GEOM_NAMES = {2: "sphere", 3: "box", 4: "cylinder", 5: "mesh", 6: "plane", 7: "capsule"}


def _jsonable(o):
    """json.dump default= hook for numpy types."""
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    return str(o)


def render_debug_depth(scene_replica) -> np.ndarray:
    """
    Metric depth (metres, z along the optical axis) of the current view,
    from ONE extra PyBullet render. Debug frames only, so the normal
    annotation path is unchanged. Background pixels are NaN.

    PyBullet returns a non-linear OpenGL depth buffer d in [0, 1]:
        z = far * near / (far - (far - near) * d)
    """
    img = p.getCameraImage(
        scene_replica.W, scene_replica.H,
        viewMatrix=scene_replica.view_matrix,
        projectionMatrix=scene_replica.projection_matrix,
        shadow=0, renderer=getattr(p, scene_replica.PB_RENDER))
    d = np.asarray(img[3], dtype=np.float64).reshape(scene_replica.H, scene_replica.W)
    near, far = scene_replica.near, scene_replica.far
    z = far * near / (far - (far - near) * d)
    z[d >= 1.0] = np.nan
    return z


def marker_inventory(marker_ids: list[int]) -> dict[str, dict]:
    """What each unnamed (visual-only) body is, so seg hits can be identified."""
    out = {}
    for bid in marker_ids:
        vs = p.getVisualShapeData(bid)
        pos, _ = p.getBasePositionAndOrientation(bid)
        shape = vs[0] if vs else None
        out[str(bid)] = {
            "geometry":   _GEOM_NAMES.get(shape[2], str(shape[2])) if shape else None,
            "dimensions": list(shape[3]) if shape else None,
            "mesh_file":  shape[4].decode("utf-8") if shape and shape[4] else None,
            "position":   [round(v, 5) for v in pos],
            "rgba":       list(shape[7]) if shape else None,
        }
    return out


def _colorize_seg(seg: np.ndarray, names: dict[int, str], markers: set[int]) -> np.ndarray:
    """Seg image with a fixed colour per id; markers striped magenta-ish."""
    out = np.zeros((*seg.shape, 3), np.uint8)
    rng = np.random.default_rng(7)
    for sid in np.unique(seg):
        if sid == -1:
            continue
        if sid in markers:
            col = (200, 40, 200)
        else:
            col = tuple(int(c) for c in rng.integers(60, 255, 3))
        out[seg == sid] = col
    y = 22
    for sid in sorted(set(np.unique(seg).tolist()) - {-1}):
        label = names.get(sid, "MARKER" if sid in markers else "?")
        cv2.putText(out, f"{sid}: {label}", (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 20
    return out


def _colorize_depth(z: np.ndarray) -> np.ndarray:
    """Turbo-coloured depth, near = red, far = blue; background black."""
    valid = np.isfinite(z)
    out = np.zeros((*z.shape, 3), np.uint8)
    if valid.any():
        lo, hi = np.percentile(z[valid], [1, 99])
        norm = np.clip((z - lo) / max(hi - lo, 1e-9), 0, 1)
        out = cv2.applyColorMap(((1 - np.nan_to_num(norm)) * 255).astype(np.uint8),
                                cv2.COLORMAP_TURBO)
        out[~valid] = 0
    return out


def _breakdown_image(full: np.ndarray, seg: np.ndarray, body_id: int,
                     names: dict[int, str], markers: set[int]) -> np.ndarray:
    """Every silhouette pixel coloured by its visibility_breakdown() bucket."""
    full = full.astype(bool)
    own = seg == body_id
    img = np.zeros((*seg.shape, 3), np.uint8)
    named_other = np.isin(seg, [i for i in names if i != body_id])
    marker = np.isin(seg, list(markers)) if markers else np.zeros_like(own)
    img[full & own] = BREAKDOWN_COLORS["visible"]
    img[full & named_other] = BREAKDOWN_COLORS["occluded_by_object"]
    img[full & marker] = BREAKDOWN_COLORS["occluded_by_marker"]
    img[full & (seg == -1)] = BREAKDOWN_COLORS["background"]
    img[full & ~own & ~named_other & ~marker & (seg != -1)] = BREAKDOWN_COLORS["other_id"]
    img[own & ~full] = BREAKDOWN_COLORS["seg_outside_silhouette"]
    return img


def _zoom_crop(img: np.ndarray, mask_any: np.ndarray, label: str) -> np.ndarray | None:
    """Crop around the object and enlarge with nearest-neighbour for pixel-level reading."""
    bb = _bbox_from_mask(mask_any)
    if bb is None:
        return None
    x, y, w, h = bb
    H, W = img.shape[:2]
    x0, y0 = max(0, x - DEBUG_CROP_PAD), max(0, y - DEBUG_CROP_PAD)
    x1, y1 = min(W, x + w + DEBUG_CROP_PAD), min(H, y + h + DEBUG_CROP_PAD)
    crop = img[y0:y1, x0:x1]
    zoom = max(1, min(DEBUG_CROP_ZOOM, DEBUG_CROP_MAX // max(crop.shape[:2])))
    big = cv2.resize(crop, (crop.shape[1] * zoom, crop.shape[0] * zoom),
                     interpolation=cv2.INTER_NEAREST)
    legend = np.zeros((22 * (len(BREAKDOWN_COLORS) + 1) + 6, max(big.shape[1], 420), 3), np.uint8)
    cv2.putText(legend, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for i, (k, c) in enumerate(BREAKDOWN_COLORS.items(), start=1):
        cv2.rectangle(legend, (6, 8 + 22 * i), (22, 22 + 22 * i), c, -1)
        cv2.putText(legend, k, (30, 21 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (230, 230, 230), 1, cv2.LINE_AA)
    pad = np.zeros((big.shape[0], legend.shape[1] - big.shape[1], 3), np.uint8) \
        if legend.shape[1] > big.shape[1] else None
    big = np.hstack([big, pad]) if pad is not None else big
    return np.vstack([big, legend])


def _safe(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", name)


def write_frame_debug(
    out_dir:     str,
    frame_id:    int,
    context:     dict,
    annotation:  dict,
    images:      dict,
    seg:         np.ndarray,
    per_object:  dict,
    names:       dict[int, str],
    markers:     set[int],
    depth:       np.ndarray | None,
) -> list[str]:
    """
    Write every debug file for one frame. Returns the files written.

    _info.json holds everything numeric:
        frame       : sensor, ids, paths, camera, turntable angle, resolution, K,
                      near/far, renderer
        timing_ms   : render, pose query + visibility, each object's silhouette,
                      debug depth render, file writing
        seg_ids     : pixel count per seg id, labelled object / marker / unknown
        markers     : what each hidden marker is and how many seg pixels it holds
                      (should be 0 — non-zero means markers still occlude)
        objects     : per object: the annotation numbers, the occlusion
                      breakdown, silhouette-vs-seg IoU and centroid offset,
                      out-of-frame pixels, depth stats, mesh / frame-offset info
    """
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, f"{frame_id:06d}")
    written = []

    def _write_png(desc, img):
        path = f"{stem}_{desc}"
        if img is not None and cv2.imwrite(path, img):
            written.append(os.path.basename(path))

    # --- frame-level images ---------------------------------------------------
    _write_png("image.jpg", images.get("image"))
    _write_png("render.png", images.get("render"))
    _write_png("overlay.png", images.get("overlay"))
    _write_png("seg.png", _colorize_seg(seg, names, markers))
    if depth is not None:
        _write_png("depth.png", _colorize_depth(depth))
        mm = np.nan_to_num(depth * 1000.0, nan=0.0)
        _write_png("depth_mm.png", np.clip(mm, 0, 65535).astype(np.uint16))

    # --- per-object masks -----------------------------------------------------
    obj_info = {}
    for body_id, d in per_object.items():
        name = _safe(d["object_name"])
        full, vis = d["full_mask"], d["visible_mask"]
        _write_png(f"{name}_mask_full.png", (full > 0).astype(np.uint8) * 255)
        _write_png(f"{name}_mask_visible.png", (vis > 0).astype(np.uint8) * 255)
        bd_img = _breakdown_image(full, seg, body_id, names, markers)
        _write_png(f"{name}_breakdown.png", bd_img)
        ann = next(a for a in annotation["objects"] if a["body_id"] == body_id)
        label = (f"{d['object_name']}  visib {ann['visib_fract']:.3f}  "
                 f"all {ann['px_count_all']}  visib {ann['px_count_visib']}")
        _write_png(f"{name}_crop.png",
                   _zoom_crop(bd_img, (full > 0) | (vis > 0), label))

        depth_stats = None
        if depth is not None and vis.any():
            zv = depth[vis.astype(bool)]
            zv = zv[np.isfinite(zv)]
            if zv.size:
                depth_stats = {"min_m": float(zv.min()), "median_m": float(np.median(zv)),
                               "max_m": float(zv.max())}
        obj_info[str(body_id)] = {
            "object_name":     d["object_name"],
            "px_count_all":    ann["px_count_all"],
            "px_count_visib":  ann["px_count_visib"],
            "visib_fract":     ann["visib_fract"],
            "bbox_obj":        ann["bbox_obj"],
            "bbox_visib":      ann["bbox_visib"],
            "t_cam_m":         ann["t"],
            "out_of_frame_px": d["out_of_frame_px"],
            "breakdown":       d["breakdown"],
            "visible_depth":   depth_stats,
            "full_mask_method": d.get("full_mask_method"),
            "full_mask_window": d.get("full_mask_window"),
            "n_tris_drawn":    d["n_tris_drawn"],
            "full_mask_ms":    d["silhouette_ms"],
            "mesh":            d["mesh"],
        }

    # --- seg id histogram -----------------------------------------------------
    ids, counts = np.unique(seg, return_counts=True)
    seg_ids = {}
    for sid, n in zip(ids.tolist(), counts.tolist()):
        kind = ("background" if sid == -1 else "object" if sid in names
                else "marker" if sid in markers else "unknown")
        seg_ids[str(sid)] = {"pixels": n, "kind": kind, "name": names.get(sid)}
    marker_px = {str(m): seg_ids.get(str(m), {}).get("pixels", 0) for m in markers}

    info = {
        "frame":     context["frame"],
        "timing_ms": context["timing_ms"],
        "seg_ids":   seg_ids,
        "markers": {
            "count": len(markers),
            "pixels_in_seg": marker_px,
            "any_marker_in_seg": any(v > 0 for v in marker_px.values()),
            "inventory": context.get("marker_inventory", {}),
        },
        "objects":   obj_info,
    }
    with open(f"{stem}_info.json", "w") as f:
        json.dump(info, f, indent=2, default=_jsonable)
    written.append(os.path.basename(f"{stem}_info.json"))
    with open(f"{stem}_annotation.json", "w") as f:
        json.dump(annotation, f, indent=2, default=_jsonable)
    written.append(os.path.basename(f"{stem}_annotation.json"))
    return written


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
    save_video:    bool = False,
    sensor_name:   str  = "sensor",
    debug_frames:  set[int] | None = None,
    debug_dir:     str | None = None,
    full_mask_method: str = "pybullet",
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
        sensor_name   : "dslr" / "realsense"; names the debug subfolder
        debug_frames  : if given, process ONLY these frame ids and write debug
                        output instead of annotations (see DEBUG MODE above)
        debug_dir     : debug output root (a <sensor_name>/ subfolder is used)
        full_mask_method : "pybullet" (default) or "projection" (legacy)

    Returns:
        Number of frames processed (annotations or debug sets written).
    """
    debug_mode  = bool(debug_frames)
    output_root = os.path.join(pose_path, "scene_replica", output_subdir)
    if debug_mode:
        debug_out = os.path.join(debug_dir or os.path.join(pose_path, "scene_replica", "debug"),
                                 sensor_name)
        os.makedirs(debug_out, exist_ok=True)
        save_video = False        # a handful of frames is not worth a video
        print(f"  DEBUG MODE — frames {sorted(debug_frames)}")
        print(f"  Debug output: {debug_out}  (annotations NOT written)")
    else:
        os.makedirs(output_root, exist_ok=True)
        print(f"  Output: {output_root}")

    global_offset = np.asarray(scene_cfg["CALIBRATION_OFFSET"], dtype=np.float64)

    if save_video:
        print(" = SAVING VIDEO FRAMES = ")
        video_path = os.path.join(pose_path, "scene_replica", f"{output_subdir}_video.mp4")
        video_writer = None

    # ── Resolve render resolution ─────────────────────────────────────────────
    # Always peek at the first frame to determine actual image resolution so
    # intrinsics are correctly scaled for visibility computation regardless of
    # whether visualize is enabled.
    found_ids: set[int] = set()
    if debug_mode:
        # Filter on frame_id; ids come from the iterator, so skipping can't shift them.
        frame_iter = (f for f in frame_iter if f[0] in debug_frames)

    try:
        first = next(frame_iter)
    except StopIteration:
        print("  [WARNING] no frames to process"
              + (f" — none of {sorted(debug_frames)} exist for this sensor" if debug_mode else ""))
        return 0

    ref_frame = cv2.imread(first[1])
    if ref_frame is None:
        raise RuntimeError(f"Cannot read reference frame: {first[1]}")

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
    # Markers are excluded by MOVING them out of view (see MARKERS above), not
    # by alpha 0, which some renderers still write to the seg buffer.
    #   visualize=False → park markers permanently now; one render per frame.
    #   visualize=True  → keep them for the display render, park them only for
    #                     the segmentation render each frame.
    marker_ids = []
    for i in range(p.getNumBodies()):
        bid  = p.getBodyUniqueId(i)
        name = p.getBodyInfo(bid)[1].decode("utf-8")
        if name == "":
            marker_ids.append(bid)
    park = park_position(scene_replica)

    if not visualize and marker_ids:
        for bid in marker_ids:
            p.resetBasePositionAndOrientation(bid, park, [0, 0, 0, 1])
        print(f"  Visual markers moved out of view ({len(marker_ids)}) — visualize=False")
    print(f"  Full masks: {full_mask_method}"
          + ("  (each object rendered alone by PyBullet)" if full_mask_method == "pybullet"
             else "  (legacy cv2 mesh rasterisation)"))

    # ── Main annotation loop ──────────────────────────────────────────────────
    frame_count   = 0
    total_start   = time.perf_counter()
    frame_times   = []

    for frame_id, img_path, cam_key, turntable_deg in frame_iter:
        frame_start = time.perf_counter()
        filename    = os.path.basename(img_path)
        image_stem  = os.path.splitext(filename)[0]
        found_ids.add(frame_id)
        timing: dict[str, float] = {}

        R_cw_cv, t_cw_cv = get_camera_pose_from_extrinsics(
            cam_key, cameras, turntable_deg, global_offset, scale
        )
        scene_replica.update_camera(R_cw_cv, t_cw_cv)
        comp = None

        t_render = time.perf_counter()
        if visualize:
            # Render 1 — full scene including markers, for display overlay
            rgba = scene_replica.render_scene_image()

            # Markers out of view for the segmentation render, then back
            with parked(marker_ids, park):
                scene_replica.render_scene_image()
                seg = scene_replica.seg   # clean segmentation, no marker pixels

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
            # Markers already parked — single render, clean seg
            rgba = scene_replica.render_scene_image()
            seg  = scene_replica.seg

            if save_video:
                bgr = cv2.imread(img_path)
                if bgr is not None:
                    if bgr.shape[1] != render_W or bgr.shape[0] != render_H:
                        bgr = cv2.resize(bgr, (render_W, render_H),
                                         interpolation=cv2.INTER_LINEAR)
                    bg   = bgr.astype(np.float32)
                    ov   = rgba[:, :, [2, 1, 0, 3]].astype(np.float32)
                    alpha = ov[:, :, 3:4] / 255.0
                    comp = np.clip(
                        ov[:, :, :3] * alpha + bg * (1.0 - alpha), 0, 255
                    ).astype(np.uint8)
                else:
                    comp = None

        
        timing["render"] = (time.perf_counter() - t_render) * 1000.0

        if save_video:
            if comp is not None:
                if video_writer is None:
                    fourcc       = cv2.VideoWriter_fourcc(*"mp4v")
                    video_writer = cv2.VideoWriter(
                        video_path, fourcc, 15.0, (render_W, render_H)
                    )
                video_writer.write(comp)
        
        t_query  = time.perf_counter()
        dbg_objs = {} if debug_mode else None
        objects = get_object_poses_in_camera_frame(
            R_cw_cv, t_cw_cv,
            seg         = seg,
            cam_K       = render_K,
            H           = render_H,
            W           = render_W,
            debug       = dbg_objs,
            marker_ids  = set(marker_ids),
            scene_replica    = scene_replica,
            full_mask_method = full_mask_method,
        )
        timing["pose_query_and_visibility"] = (time.perf_counter() - t_query) * 1000.0

        # ── 1. Image dimensions included for self-contained annotations ───────
        annotation = {
            "frame":         filename,
            "cam_key":       cam_key,
            "turntable_deg": turntable_deg,
            "width":         render_W,
            "height":        render_H,
            "objects":       objects,
        }

        if debug_mode:
            t_dbg = time.perf_counter()
            depth = render_debug_depth(scene_replica)
            timing["debug_depth_render"] = (time.perf_counter() - t_dbg) * 1000.0
            for oid, d in dbg_objs.items():
                timing[f"full_mask[{d['object_name']}]"] = d["silhouette_ms"]

            src = cv2.imread(img_path)
            if src is not None and (src.shape[1] != render_W or src.shape[0] != render_H):
                src = cv2.resize(src, (render_W, render_H), interpolation=cv2.INTER_AREA)
            render_bgr = cv2.cvtColor(np.ascontiguousarray(scene_replica.rgb[:, :, :3]),
                                      cv2.COLOR_RGB2BGR)
            overlay = comp
            if overlay is None and src is not None:
                ov = rgba[:, :, [2, 1, 0, 3]].astype(np.float32)
                a = ov[:, :, 3:4] / 255.0
                overlay = np.clip(ov[:, :, :3] * a + src.astype(np.float32) * (1 - a),
                                  0, 255).astype(np.uint8)

            names = {o["body_id"]: o["object_name"] for o in objects}
            context = {
                "frame": {
                    "frame_id":        frame_id,
                    "sensor":          sensor_name,
                    "image":           img_path,
                    "cam_key":         cam_key,
                    "turntable_deg":   turntable_deg,
                    "render_size":     [render_W, render_H],
                    "cam_K":           render_K,
                    "near_far_m":      [scene_replica.near, scene_replica.far],
                    "renderer":        scene_replica.PB_RENDER,
                    "R_cw_cv":         R_cw_cv,
                    "t_cw_cv":         t_cw_cv,
                },
                "timing_ms": {k: round(v, 2) for k, v in timing.items()},
                "marker_inventory": marker_inventory(marker_ids),
            }
            t_w = time.perf_counter()
            written = write_frame_debug(
                debug_out, frame_id, context, annotation,
                {"image": src, "render": render_bgr, "overlay": overlay},
                seg, dbg_objs, names, set(marker_ids), depth)
            context["timing_ms"]["write_debug_files"] = round((time.perf_counter() - t_w) * 1000.0, 2)
            dest = f"{len(written)} debug files"

            for oid, d in dbg_objs.items():
                bd = d["breakdown"]
                ann = next(a for a in objects if a["body_id"] == oid)
                print(f"      {d['object_name']:<22} visib {ann['visib_fract']:.3f}"
                      f"  all {ann['px_count_all']:>7}  visib {ann['px_count_visib']:>7}"
                      f"  | bg {bd['background']:>6}  obj {sum(bd['occluded_by_object'].values()):>6}"
                      f"  marker {sum(bd['occluded_by_marker'].values()):>6}"
                      f"  seg-outside {bd['seg_outside_silhouette']:>5}"
                      f"  IoU {bd['iou_silhouette_vs_seg']}")
        else:
            output_path = os.path.join(output_root, f"{image_stem}.json")
            with open(output_path, "w") as f:
                json.dump(annotation, f, indent=2)
            dest = os.path.basename(output_path)

        frame_time = time.perf_counter() - frame_start
        frame_times.append(frame_time)
        frame_count += 1

        print(f"  {frame_id:06d}  {cam_key:>4}  {turntable_deg:>5.1f}°  {filename}"
              f"  → {len(objects)} obj"
              f"  [{frame_time*1000:.0f}ms]"
              f"  →  {dest}")

    if visualize:
        cv2.destroyAllWindows()

    if debug_mode:
        missing = sorted(set(debug_frames) - found_ids)
        if missing:
            print(f"  [WARNING] requested frames not found for {sensor_name}: {missing}")

    if save_video:
        if video_writer is not None:
            video_writer.release()
            print(f"  Video saved: {video_path}")
        else:
            print("  [WARNING] save_video=True but no frames were written "
                  "(frame_iter may have been empty or all images failed to load)")

    # ── Timing summary ────────────────────────────────────────────────────────
    total_s   = time.perf_counter() - total_start
    avg_ms    = (sum(frame_times) / len(frame_times) * 1000) if frame_times else 0.0
    total_min = int(total_s // 60)
    total_sec = total_s % 60

    print(f"\n  {'─'*50}")
    print(f"  {'Debug frames' if debug_mode else 'Annotations written'} : {frame_count}")
    print(f"  Output folder       : {debug_out if debug_mode else output_root}")
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
                save_video    = args.save_video,
                sensor_name   = sensor_name,
                debug_frames  = set(args.debug_frames) if args.debug_frames else None,
                debug_dir     = args.debug_dir,
                full_mask_method = args.full_mask_method,
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
    parser.add_argument("--object",    default="ex2_006",
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
                        default="/home/csrobot/moad_control/clutter_metrics/data_collection/object_sets/experiment2-dec",
                        help="Folder containing URDF models referenced by the scene")

    # ── Output options ────────────────────────────────────────────────────────
    parser.add_argument("--generate-poses",  action="store_true", default=True,
                        help="Generate 6D pose annotations (default: on)")
    parser.add_argument("--generate-bbs",    action="store_true", default=False,
                        help="Generate bounding box annotations (not yet implemented)")
    parser.add_argument("--generate-masks",  action="store_true", default=False,
                        help="Generate segmentation masks (not yet implemented)")
    parser.add_argument("--visualize",       action="store_true", default=False,
                        help="Show PyBullet overlay composited on each image")
    parser.add_argument("--debug-frames", type=int, nargs="+", default=None,
                        help="Process ONLY these frame ids (1-based; same numbering as the "
                             "BOP converter and vis_bop_predictions.py) and write detailed "
                             "debug output instead of annotations. Applies to every sensor; "
                             "frame N is the same camera/turntable view in each.")
    parser.add_argument("--full-mask-method", choices=["pybullet", "projection"],
                        default="pybullet",
                        help="How px_count_all is computed. pybullet (default): each object "
                             "rendered alone by PyBullet on the scene's pixel grid, so it "
                             "matches px_count_visib exactly. projection: legacy cv2 mesh "
                             "rasterisation (off by up to a pixel at every edge).")
    parser.add_argument("--debug-dir", default=None,
                        help="Debug output root (default <pose>/scene_replica/debug). "
                             "A subfolder per sensor keeps DSLR and RealSense apart.")
    parser.add_argument("--debug-masks", action="store_true", default=False,
                        help=argparse.SUPPRESS)   # removed; superseded by --debug-frames
    parser.add_argument("--save-video",       action="store_true", default=True,
                        help="Outputs a video of each visualized frame")

    args = parser.parse_args()
    if args.debug_masks:
        print("  [NOTE] --debug-masks has been replaced by --debug-frames "
              "(writes masks to files instead of blocking windows); ignoring it.")
    main(args)