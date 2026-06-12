'''
VIEW SCENE ANNOTATIONS
Companion script to generate_scene_annotations.py.

Loads per-frame pose annotation JSONs and draws labelled 3D bounding boxes
projected onto the corresponding images. No PyBullet or scene replica
dependencies — pure NumPy + OpenCV.

CONTROLS
    Space       Toggle play / pause
    Right →     Step forward one frame   (works in both play and pause modes)
    Left  ←     Step backward one frame
    Q / Esc     Quit

PATHS EXPECTED (must match generate_scene_annotations.py conventions)
    Images:      <data_root>/<object>/<pose>/images_<scale>/frame_NNNNN.jpg
    Annotations: <data_root>/<object>/<pose>/scene_replica/pose/frame_NNNNN.json
    Calibration: <calib_root>/<calibration>/cam_parameters.json
'''

import os
import re
import sys
import glob
import json
import argparse
import time
import numpy as np
import cv2


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def build_cam_K(intrinsics: dict) -> np.ndarray:
    return np.array([
        [intrinsics["fx"],             0.0, intrinsics["cx"]],
        [            0.0, intrinsics["fy"], intrinsics["cy"]],
        [            0.0,             0.0,             1.0],
    ], dtype=np.float64)


ASPECT_TOL = 0.01

def check_and_scale_intrinsics(intrinsics: dict, frame: np.ndarray) -> dict:
    """
    Verify the frame's aspect ratio against the calibration and return a
    version of the intrinsics dict scaled to the frame's actual resolution.
    Raises ValueError on aspect ratio mismatch.
    """
    calib_w = intrinsics["width"]
    calib_h = intrinsics["height"]
    frame_h, frame_w = frame.shape[:2]

    calib_ar = calib_w / calib_h
    frame_ar = frame_w / frame_h

    if abs(calib_ar - frame_ar) > ASPECT_TOL:
        raise ValueError(
            f"Aspect ratio mismatch: calibration={calib_w}x{calib_h} ({calib_ar:.4f}) "
            f"vs image={frame_w}x{frame_h} ({frame_ar:.4f}). "
            f"Check which images_N folder you are targeting."
        )

    s = frame_w / calib_w
    return {
        **intrinsics,
        "fx":     intrinsics["fx"] * s,
        "fy":     intrinsics["fy"] * s,
        "cx":     intrinsics["cx"] * s,
        "cy":     intrinsics["cy"] * s,
        "width":  frame_w,
        "height": frame_h,
    }


def bbox_corners(size: list[float]) -> np.ndarray:
    """
    Return the 8 corners of an axis-aligned box centred at the origin,
    given full extents [sx, sy, sz].  Shape: (8, 3).

        Corner layout (right-hand, Z-up):
             6----7
            /|   /|
           4----5 |
           | 2--|-3
           |/   |/
           0----1
    """
    sx, sy, sz = np.array(size) / 2.0
    corners = np.array([
        [-sx, -sy, -sz],  # 0
        [ sx, -sy, -sz],  # 1
        [-sx,  sy, -sz],  # 2
        [ sx,  sy, -sz],  # 3
        [-sx, -sy,  sz],  # 4
        [ sx, -sy,  sz],  # 5
        [-sx,  sy,  sz],  # 6
        [ sx,  sy,  sz],  # 7
    ], dtype=np.float64)
    return corners


# Pairs of corner indices that form the 12 edges of the box
BBOX_EDGES = [
    (0, 1), (2, 3), (4, 5), (6, 7),   # four X-direction edges
    (0, 2), (1, 3), (4, 6), (5, 7),   # four Y-direction edges
    (0, 4), (1, 5), (2, 6), (3, 7),   # four Z-direction edges
]


def project_points(points_3d: np.ndarray, cam_K: np.ndarray) -> np.ndarray:
    """
    Project (N, 3) points in camera frame to (N, 2) pixel coordinates.
    Points behind the camera (z <= 0) are not filtered here — callers
    should check z before drawing if needed.
    """
    pts = (cam_K @ points_3d.T).T          # (N, 3)
    uv  = pts[:, :2] / pts[:, 2:3]         # (N, 2)
    return uv


def transform_points(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Apply rotation R (3x3) and translation t (3,) to (N, 3) points."""
    return (R @ points.T).T + t


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

# One distinct colour per object (BGR). Cycles if there are more objects.
OBJECT_COLORS = [
    (  0, 200, 255),  # amber
    ( 50, 255,  50),  # green
    (255,  80,  80),  # blue
    (255,  50, 200),  # pink
    (  0, 255, 180),  # yellow-green
    (200, 100, 255),  # purple
]


def draw_bbox_3d(
    img:      np.ndarray,
    R:        np.ndarray,
    t:        np.ndarray,
    size:     list[float],
    cam_K:    np.ndarray,
    color:    tuple[int, int, int],
    label:    str  = "",
    line_thickness: int = 2,
) -> np.ndarray:
    """
    Project and draw a 3D bounding box onto img (in-place copy returned).

    Args:
        img    : BGR image
        R      : (3,3) object-to-camera rotation  (OpenCV convention)
        t      : (3,)  object-to-camera translation
        size   : [sx, sy, sz] full extents in metres
        cam_K  : (3,3) camera intrinsic matrix (scaled to img resolution)
        color  : BGR draw colour
        label  : text label drawn near the top-front edge of the box
        line_thickness : cv2 line thickness

    Returns:
        Annotated copy of img.
    """
    img = img.copy()

    corners_obj = bbox_corners(size)                        # (8, 3) in object frame
    corners_cam = transform_points(corners_obj, R, t)       # (8, 3) in camera frame

    # Skip box entirely if all corners are behind the camera
    if np.all(corners_cam[:, 2] <= 0):
        return img

    px = project_points(corners_cam, cam_K)                 # (8, 2) pixel coords
    px_int = px.astype(int)

    # Draw edges — skip any edge where either endpoint is behind the camera
    for i, j in BBOX_EDGES:
        if corners_cam[i, 2] <= 0 or corners_cam[j, 2] <= 0:
            continue
        cv2.line(img, tuple(px_int[i]), tuple(px_int[j]), color, line_thickness, cv2.LINE_AA)

    # Label — anchor to the projected centroid so it stays near the box
    visible_mask = corners_cam[:, 2] > 0
    if visible_mask.any() and label:
        centroid_px = px[visible_mask].mean(axis=0).astype(int)
        draw_label(img, label, tuple(centroid_px), color)

    return img


def draw_label(
    img:    np.ndarray,
    text:   str,
    pos:    tuple[int, int],
    color:  tuple[int, int, int],
    font_scale: float = 0.55,
    thickness:  int   = 1,
) -> None:
    """Draw a text label with a dark background rect for legibility (in-place)."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = pos
    pad = 3
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + baseline + pad),
                  (0, 0, 0), cv2.FILLED)
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_status(
    img:           np.ndarray,
    frame_name:    str,
    cam_key:       str,
    turntable_deg: float,
    paused:        bool,
    frame_idx:     int,
    total_frames:  int,
) -> np.ndarray:
    """
    Draw a semi-transparent status bar at the bottom of the image.
    Returns an annotated copy.
    """
    img = img.copy()
    h, w = img.shape[:2]

    bar_h   = 36
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    play_icon = "⏸ PAUSED" if paused else "▶ PLAYING"
    status = (f"   {frame_name}   |   "
              f"cam: {cam_key}   |   turntable: {turntable_deg:.1f}   |   "
              f"frame {frame_idx + 1}/{total_frames}")

    font       = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.50
    thickness  = 1
    text_y     = h - bar_h // 2 + 5

    cv2.putText(img, status, (8, text_y), font, font_scale,
                (200, 200, 200), thickness, cv2.LINE_AA)

    # Keyboard hint in the top-left corner
    hint = "Space: play/pause    A/D: step    Q/Esc: quit"
    cv2.putText(img, hint, (8, 18), font, 0.42,
                (100, 100, 100), 1, cv2.LINE_AA)

    return img


# ---------------------------------------------------------------------------
# Core viewer
# ---------------------------------------------------------------------------

def run_viewer(
    pose_path:   str,
    images_dir:  str,
    annot_dir:   str,
    intrinsics:  dict,
    play_fps:    float = 10.0,
) -> None:
    """
    Main viewer loop.

    Args:
        pose_path   : base pose folder (for display only)
        images_dir  : folder containing frame_NNNNN.jpg images
        annot_dir   : folder containing frame_NNNNN.json annotations
        intrinsics  : full-resolution intrinsics dict from cam_parameters.json
        play_fps    : playback speed when not paused
    """
    image_paths = sorted(glob.glob(os.path.join(images_dir, "frame_*.jpg")))
    if not image_paths:
        raise FileNotFoundError(f"No frame_*.jpg images found in: {images_dir}")

    # Verify annotation files exist for every image
    missing = []
    for img_path in image_paths:
        stem    = os.path.splitext(os.path.basename(img_path))[0]
        ann_path = os.path.join(annot_dir, f"{stem}.json")
        if not os.path.isfile(ann_path):
            missing.append(stem)
    if missing:
        print(f"[WARNING] {len(missing)} annotation file(s) missing, e.g.: {missing[0]}.json")
        print(f"          Run generate_scene_annotations.py first.")

    # Scale intrinsics to actual image resolution using the first frame
    ref_bgr = cv2.imread(image_paths[0])
    if ref_bgr is None:
        raise RuntimeError(f"Cannot read reference frame: {image_paths[0]}")
    scaled_intr = check_and_scale_intrinsics(intrinsics, ref_bgr)
    cam_K       = build_cam_K(scaled_intr)
    print(f"  Intrinsics scaled: {intrinsics['width']}×{intrinsics['height']} "
          f"→ {scaled_intr['width']}×{scaled_intr['height']}")

    total   = len(image_paths)
    idx     = 0
    paused  = False
    frame_delay_ms = max(1, int(1000.0 / play_fps))

    cv2.namedWindow("Annotation Viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Annotation Viewer", scaled_intr["width"], scaled_intr["height"])

    print(f"\n  Loaded {total} frames. Starting viewer…")
    print(f"  Space: play/pause  |  ←/→: step  |  Q/Esc: quit\n")

    while True:
        img_path  = image_paths[idx]
        frame_name = os.path.basename(img_path)
        stem       = os.path.splitext(frame_name)[0]
        ann_path   = os.path.join(annot_dir, f"{stem}.json")

        bgr = cv2.imread(img_path)
        if bgr is None:
            print(f"  [WARN] Could not read {img_path}, skipping.")
            idx = (idx + 1) % total
            continue

        # Resize if necessary to match scaled intrinsics
        ih, iw = bgr.shape[:2]
        if iw != scaled_intr["width"] or ih != scaled_intr["height"]:
            bgr = cv2.resize(bgr, (scaled_intr["width"], scaled_intr["height"]),
                             interpolation=cv2.INTER_LINEAR)

        # Load annotation and draw bounding boxes
        cam_key       = "?"
        turntable_deg = 0.0

        if os.path.isfile(ann_path):
            with open(ann_path, "r") as f:
                ann = json.load(f)

            cam_key       = ann.get("cam_key",       "?")
            turntable_deg = ann.get("turntable_deg", 0.0)

            for obj_idx, obj in enumerate(ann.get("objects", [])):
                R     = np.array(obj["R"],    dtype=np.float64)
                t     = np.array(obj["t"],    dtype=np.float64)
                size  = obj.get("size", [0.1, 0.1, 0.1])
                label = obj.get("object_name", f"obj_{obj_idx}")
                color = OBJECT_COLORS[obj_idx % len(OBJECT_COLORS)]
                bgr   = draw_bbox_3d(bgr, R, t, size, cam_K, color, label)
        else:
            # Annotation missing — draw a red warning on the frame
            cv2.putText(bgr, f"No annotation: {stem}.json",
                        (10, scaled_intr["height"] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2, cv2.LINE_AA)

        bgr = draw_status(bgr, frame_name, cam_key, turntable_deg,
                          paused, idx, total)

        cv2.imshow("Annotation Viewer", bgr)

        # ── Keyboard handling ──────────────────────────────────────────────
        wait_ms = frame_delay_ms if not paused else 30
        key = cv2.waitKey(wait_ms) & 0xFF

        if key in (ord('q'), 27):           # Q or Esc → quit
            break
        elif key == ord(' '):               # Space → toggle play/pause
            paused = not paused
        elif key == 81 or key == ord('a'):  # Left arrow or A → prev frame
            idx = (idx - 1) % total
            paused = True
        elif key == 83 or key == ord('d'):  # Right arrow or D → next frame
            idx = (idx + 1) % total
            paused = True
        elif not paused:                    # Auto-advance during playback
            idx = (idx + 1) % total

    cv2.destroyAllWindows()
    print("  Viewer closed.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args):
    # ── Validate pose folder ──────────────────────────────────────────────────
    pose_path = os.path.join(args.data_root, args.object, args.pose)
    if not os.path.isdir(pose_path):
        print(f"[ERROR] Pose folder not found: {pose_path}")
        sys.exit(1)

    # ── Validate annotation folder ────────────────────────────────────────────
    annot_dir = os.path.join(pose_path, "scene_replica", "pose")
    if not os.path.isdir(annot_dir):
        print(f"[ERROR] Annotation folder not found: {annot_dir}")
        print(f"        Run generate_scene_annotations.py for this object/pose first.")
        sys.exit(1)

    ann_files = glob.glob(os.path.join(annot_dir, "frame_*.json"))
    if not ann_files:
        print(f"[ERROR] No frame_*.json files found in: {annot_dir}")
        print(f"        Run generate_scene_annotations.py for this object/pose first.")
        sys.exit(1)
    print(f"  Found {len(ann_files)} annotation file(s) in {annot_dir}")

    # ── Validate images folder ────────────────────────────────────────────────
    images_dir = os.path.join(pose_path, f"images_{args.scale}")
    if not os.path.isdir(images_dir):
        print(f"[ERROR] Images folder not found: {images_dir}")
        print(f"        Available images_N folders:")
        for d in glob.glob(os.path.join(pose_path, "images_*")):
            print(f"          {os.path.basename(d)}")
        sys.exit(1)

    # ── Load calibration ──────────────────────────────────────────────────────
    calib_path = os.path.join(args.calib_root, args.calibration, "cam_parameters.json")
    if not os.path.isfile(calib_path):
        print(f"[ERROR] Calibration file not found: {calib_path}")
        sys.exit(1)

    with open(calib_path, "r") as f:
        cal = json.load(f)
    intrinsics = cal["intrinsics"]
    print(f"  Calibration loaded: {calib_path}")
    print(f"  Full-res intrinsics: {intrinsics['width']}×{intrinsics['height']}")

    # ── Launch viewer ─────────────────────────────────────────────────────────
    run_viewer(
        pose_path  = pose_path,
        images_dir = images_dir,
        annot_dir  = annot_dir,
        intrinsics = intrinsics,
        play_fps   = args.fps,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="View ground truth 3D bounding box annotations overlaid on scan frames."
    )
    parser.add_argument("--data-root",    default="/home/csrobot/MOAD_DATA",
                        help="Root data directory")
    parser.add_argument("--object",       default="batch1_001",
                        help="Object subfolder name")
    parser.add_argument("--pose",         default="pose-a",
                        help="Pose subfolder name")
    parser.add_argument("--calib-root",   default="/home/csrobot/moad_control/moad_cui/calibration",
                        help="Root calibration directory")
    parser.add_argument("--calibration",  default="55mm",
                        help="Folder name of camera calibration to use")
    parser.add_argument("--scale",        default="4",
                        help="Image downscale factor suffix, e.g. '8' → images_8")
    parser.add_argument("--fps",          type=float, default=10.0,
                        help="Playback speed in frames per second (default: 10)")

    args = parser.parse_args()
    main(args)
