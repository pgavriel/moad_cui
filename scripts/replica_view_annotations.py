'''
VIEW SCENE ANNOTATIONS
Companion script to replica_generate_annotations.py.

Loads per-frame pose annotation JSONs and draws labelled 3D bounding boxes
projected onto the corresponding images. No PyBullet or scene replica
dependencies — pure NumPy + OpenCV.

Which sensor sources are shown is determined automatically by scanning for
annotation subfolders and matching image folders. Tab switches between
available sensors at the same frame index, allowing direct comparison of
DSLR and RealSense annotations for the same physical scene position.

CONTROLS
    Space       Toggle play / pause
    D / →       Step forward one frame
    A / ←       Step backward one frame
    Tab         Cycle between available sensor sources (DSLR ↔ RealSense ↔ ...)
    Q / Esc     Quit

SENSOR SOURCE DISCOVERY
    Annotation subfolders and their matching image folders are defined in
    VIEWER_SENSOR_CONFIGS below. A source is included only when both its
    annotation subfolder and image folder are present and non-empty.

ADDING A NEW SENSOR TYPE
    Add one entry to VIEWER_SENSOR_CONFIGS. No other changes needed.
'''

import os
import sys
import glob
import json
import argparse
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Sensor source configuration table
# ---------------------------------------------------------------------------
# Mirrors SENSOR_CONFIGS in replica_generate_annotations.py.
# Each entry maps a sensor name to:
#   annot_subdir  : subfolder under scene_replica/ containing annotation JSONs
#   images_subdir : subfolder under the pose folder containing source images
#   image_glob    : glob pattern for finding images in images_subdir
#   calib_file    : calibration filename within the calibration root folder
#
# A source is activated only when both annot_subdir and images_subdir exist
# and contain matching files — no flags needed.

VIEWER_SENSOR_CONFIGS = {
    "dslr": {
        "annot_subdir":  "pose_dslr",
        "images_subdir": "images_4",
        "image_glob":    "frame_*.jpg",
        "calib_file":    "cam_parameters.json",
    },
    "realsense": {
        "annot_subdir":  "pose_realsense",
        "images_subdir": "realsense",
        "image_glob":    "rs*_color.png",
        "calib_file":    "realsense_cam_parameters.json",
    },
}


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class SensorSource:
    """All data needed to display one sensor's annotations."""
    name:         str             # e.g. "dslr" or "realsense"
    image_paths:  list            # sorted list of image file paths
    annot_dir:    str             # annotation JSON folder
    cam_K:        np.ndarray      # (3,3) intrinsic matrix scaled to image res
    width:        int             # display width
    height:       int             # display height

    def __len__(self):
        return len(self.image_paths)

    def annot_path_for(self, img_path: str) -> str:
        """Return the expected annotation JSON path for a given image path."""
        stem = os.path.splitext(os.path.basename(img_path))[0]
        return os.path.join(self.annot_dir, f"{stem}.json")


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
    Verify the frame's aspect ratio against calibration intrinsics.
    Returns a scaled copy if the resolution differs, or the original if it
    matches exactly (scale == 1.0 is a no-op).
    Raises ValueError on aspect ratio mismatch.
    """
    calib_w, calib_h = intrinsics["width"], intrinsics["height"]
    frame_h, frame_w = frame.shape[:2]

    if abs((calib_w / calib_h) - (frame_w / frame_h)) > ASPECT_TOL:
        raise ValueError(
            f"Aspect ratio mismatch: calibration={calib_w}×{calib_h} "
            f"vs image={frame_w}×{frame_h}."
        )

    s = frame_w / calib_w
    if abs(s - 1.0) < 1e-6:
        return intrinsics

    return {
        **intrinsics,
        "fx": intrinsics["fx"] * s,  "fy": intrinsics["fy"] * s,
        "cx": intrinsics["cx"] * s,  "cy": intrinsics["cy"] * s,
        "width": frame_w, "height": frame_h,
    }


def bbox_corners(size: list[float]) -> np.ndarray:
    """
    Return the 8 corners of an axis-aligned box centred at the origin.
    size = [sx, sy, sz] full extents. Shape: (8, 3).

    Corner layout:
         6----7
        /|   /|
       4----5 |
       | 2--|-3
       |/   |/
       0----1
    """
    sx, sy, sz = np.array(size) / 2.0
    return np.array([
        [-sx, -sy, -sz], [ sx, -sy, -sz],
        [-sx,  sy, -sz], [ sx,  sy, -sz],
        [-sx, -sy,  sz], [ sx, -sy,  sz],
        [-sx,  sy,  sz], [ sx,  sy,  sz],
    ], dtype=np.float64)


BBOX_EDGES = [
    (0,1),(2,3),(4,5),(6,7),   # X edges
    (0,2),(1,3),(4,6),(5,7),   # Y edges
    (0,4),(1,5),(2,6),(3,7),   # Z edges
]


def project_points(points_3d: np.ndarray, cam_K: np.ndarray) -> np.ndarray:
    """Project (N,3) camera-space points to (N,2) pixel coordinates."""
    pts = (cam_K @ points_3d.T).T
    return pts[:, :2] / pts[:, 2:3]


def transform_points(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Apply rotation R (3×3) and translation t (3,) to (N,3) points."""
    return (R @ points.T).T + t


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

# Distinct BGR colours per object — cycles if more objects than colours
OBJECT_COLORS = [
    (  0, 200, 255),   # amber
    ( 50, 255,  50),   # green
    (255,  80,  80),   # blue
    (255,  50, 200),   # pink
    (  0, 255, 180),   # yellow-green
    (200, 100, 255),   # purple
]


def draw_bbox_3d(
    img:            np.ndarray,
    R:              np.ndarray,
    t:              np.ndarray,
    size:           list[float],
    cam_K:          np.ndarray,
    color:          tuple,
    label:          str = "",
    line_thickness: int = 2,
    label_opacity = 0.75
) -> np.ndarray:
    """
    Project and draw a 3D bounding box onto a copy of img.
    Edges where either endpoint is behind the camera are silently skipped.
    The label is anchored to the centroid of all visible projected corners.
    """
    img         = img.copy()
    corners_cam = transform_points(bbox_corners(size), R, t)

    if np.all(corners_cam[:, 2] <= 0):
        return img

    px     = project_points(corners_cam, cam_K)
    px_int = px.astype(int)

    for i, j in BBOX_EDGES:
        if corners_cam[i, 2] <= 0 or corners_cam[j, 2] <= 0:
            continue
        cv2.line(img, tuple(px_int[i]), tuple(px_int[j]),
                 color, line_thickness, cv2.LINE_AA)

    visible = corners_cam[:, 2] > 0
    if visible.any() and label:
        centroid = px[visible].mean(axis=0).astype(int)
        _draw_label(img, label, tuple(centroid), color,label_opacity)

    return img


def _draw_label(img, text, pos, color, font_scale=0.55,
                thickness=1, opacity=0.75):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = pos
    pad  = 3
    x1, y1 = x - pad,      y - th - pad
    x2, y2 = x + tw + pad, y + baseline + pad

    # Blend background rectangle at the given opacity
    overlay = img.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, opacity, img, 1.0 - opacity, 0, img)

    # Text is always drawn at full opacity on top
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_overlay(
    img:           np.ndarray,
    annotation:    dict | None,
    cam_K:         np.ndarray,
    sensor_name:   str,
    frame_idx:     int,
    total_frames:  int,
    paused:        bool,
    source_count:  int,
    label_opacity=0.75
) -> np.ndarray:
    """
    Composite all annotation overlays and the status bar onto a copy of img.

    Draws:
      - 3D bounding boxes for all objects in the annotation
      - Sensor / frame status bar at the bottom
      - Annotation type badge and keyboard hint at the top
    """
    img = img.copy()
    h, w = img.shape[:2]

    # ── Bounding boxes ────────────────────────────────────────────────────────
    if annotation is not None:
        for obj_idx, obj in enumerate(annotation.get("objects", [])):
            R     = np.array(obj["R"],   dtype=np.float64)
            t     = np.array(obj["t"],   dtype=np.float64)
            size  = obj.get("size", [0.1, 0.1, 0.1])
            label = obj.get("object_name", f"obj_{obj_idx}")
            color = OBJECT_COLORS[obj_idx % len(OBJECT_COLORS)]
            img   = draw_bbox_3d(img, R, t, size, cam_K, color, label, label_opacity=label_opacity)

    # ── Status bar (bottom) ───────────────────────────────────────────────────
    bar_h   = 36
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    font = cv2.FONT_HERSHEY_SIMPLEX

    if annotation is not None:
        frame_name    = annotation.get("frame",         "—")
        cam_key       = annotation.get("cam_key",       "?")
        turntable_deg = annotation.get("turntable_deg", 0.0)
        play_state    = "[PAUSED]" if paused else ""
        status = (f"  {play_state}  {sensor_name.upper()}  |  {frame_name}  |  "
                  f"cam: {cam_key}  |  turntable: {turntable_deg:.1f}deg  |  "
                  f"frame {frame_idx + 1}/{total_frames}")
    else:
        status = f"  {'[PAUSED]' if paused else ''}  {sensor_name.upper()}  |  "  \
                 f"frame {frame_idx + 1}/{total_frames}  |  NO ANNOTATION"

    cv2.putText(img, status, (8, h - bar_h // 2 + 5),
                font, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

    # ── Top hint bar ──────────────────────────────────────────────────────────
    cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), cv2.FILLED)

    # Annotation type badge (top-left)
    annot_type = "POSE"   # extend here when BBOX / MASK are implemented
    cv2.putText(img, f"[ {annot_type} ]", (8, 16),
                font, 0.45, (80, 200, 80), 1, cv2.LINE_AA)

    # Controls hint (right of badge)
    tab_hint = "  Tab: next sensor" if source_count > 1 else ""
    hint = f"Space: play/pause    A/D: step{tab_hint}    Q/Esc: quit"
    cv2.putText(img, hint, (120, 16),
                font, 0.40, (130, 130, 130), 1, cv2.LINE_AA)

    return img


# ---------------------------------------------------------------------------
# Source discovery
# ---------------------------------------------------------------------------

def discover_sources(
    pose_path:  str,
    calib_root: str,
) -> list[SensorSource]:
    """
    Scan for available annotation + image source pairs using VIEWER_SENSOR_CONFIGS.

    A sensor source is included only when:
      - Its annotation subfolder exists and contains at least one JSON file
      - Its image subfolder exists and contains at least one matching image
      - Its calibration file exists in calib_root

    Returns a list of SensorSource objects, one per available sensor, in the
    order they appear in VIEWER_SENSOR_CONFIGS.
    """
    sources = []

    for sensor_name, cfg in VIEWER_SENSOR_CONFIGS.items():

        annot_dir  = os.path.join(pose_path, "scene_replica", cfg["annot_subdir"])
        images_dir = os.path.join(pose_path, cfg["images_subdir"])
        calib_path = os.path.join(calib_root, cfg["calib_file"])

        # Check annotation folder
        ann_files = glob.glob(os.path.join(annot_dir, "*.json"))
        if not ann_files:
            print(f"  [SKIP] {sensor_name.upper()}: no annotations in {annot_dir}")
            continue

        # Check image folder
        image_paths = sorted(glob.glob(os.path.join(images_dir, cfg["image_glob"])))
        if not image_paths:
            print(f"  [SKIP] {sensor_name.upper()}: no images in {images_dir}")
            continue

        # Check calibration file
        if not os.path.isfile(calib_path):
            print(f"  [SKIP] {sensor_name.upper()}: calibration not found: {calib_path}")
            continue

        # Load calibration and scale intrinsics to actual image resolution
        with open(calib_path, "r") as f:
            cal = json.load(f)
        intrinsics = cal["intrinsics"]

        ref_bgr = cv2.imread(image_paths[0])
        if ref_bgr is None:
            print(f"  [SKIP] {sensor_name.upper()}: cannot read reference image: {image_paths[0]}")
            continue

        try:
            scaled_intr = check_and_scale_intrinsics(intrinsics, ref_bgr)
        except ValueError as e:
            print(f"  [SKIP] {sensor_name.upper()}: {e}")
            continue

        cam_K = build_cam_K(scaled_intr)

        sources.append(SensorSource(
            name        = sensor_name,
            image_paths = image_paths,
            annot_dir   = annot_dir,
            cam_K       = cam_K,
            width       = scaled_intr["width"],
            height      = scaled_intr["height"],
        ))

        print(f"  [OK] {sensor_name.upper()}: "
              f"{len(image_paths)} images, {len(ann_files)} annotations  "
              f"({scaled_intr['width']}×{scaled_intr['height']})")

    return sources


# ---------------------------------------------------------------------------
# Core viewer
# ---------------------------------------------------------------------------

def run_viewer(sources, play_fps=10.0, label_opacity=0.75, display_width=None):
    """
    Main viewer loop.

    Maintains a source index (which sensor) and a frame index (which frame
    within that sensor). Tab cycles the source index, clamping the frame index
    to the new source's length so out-of-bounds switches are handled gracefully.

    Args:
        sources  : list of SensorSource objects (at least one)
        play_fps : playback speed in frames per second when not paused
    """
    source_idx     = 0
    frame_idx      = 0
    paused         = False
    frame_delay_ms = max(1, int(1000.0 / play_fps))

    src = sources[source_idx]
    cv2.namedWindow("Annotation Viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Annotation Viewer", src.width, src.height)

    print(f"\n  {len(sources)} sensor source(s) available: "
          f"{[s.name for s in sources]}")
    print(f"  Space: play/pause  |  A/D: step  |  Tab: next sensor  |  Q/Esc: quit\n")

    while True:
        src       = sources[source_idx]
        img_path  = src.image_paths[frame_idx]
        ann_path  = src.annot_path_for(img_path)

        # ── Load image ────────────────────────────────────────────────────────
        bgr = cv2.imread(img_path)
        if bgr is None:
            print(f"  [WARN] Cannot read {img_path}")
            frame_idx = (frame_idx + 1) % len(src)
            continue

        # Resize to display resolution if needed
        if bgr.shape[1] != src.width or bgr.shape[0] != src.height:
            bgr = cv2.resize(bgr, (src.width, src.height),
                             interpolation=cv2.INTER_LINEAR)

        # ── Load annotation ───────────────────────────────────────────────────
        annotation = None
        if os.path.isfile(ann_path):
            with open(ann_path, "r") as f:
                annotation = json.load(f)
        else:
            # Draw a visible warning directly on the frame
            cv2.putText(bgr,
                        f"No annotation: {os.path.basename(ann_path)}",
                        (10, src.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2, cv2.LINE_AA)

        # ── Compose and display ───────────────────────────────────────────────
        display = draw_overlay(
            img          = bgr,
            annotation   = annotation,
            cam_K        = src.cam_K,
            sensor_name  = src.name,
            frame_idx    = frame_idx,
            total_frames = len(src),
            paused       = paused,
            source_count = len(sources),
            label_opacity = label_opacity
        )

        if display_width is not None:
            scale   = display_width / display.shape[1]
            disp_h  = int(display.shape[0] * scale)
            display = cv2.resize(display, (display_width, disp_h),
                                interpolation=cv2.INTER_LINEAR)
            cv2.resizeWindow("Annotation Viewer", display_width, disp_h)
        else:
            # Resize window if we switched to a source with different resolution
            cv2.resizeWindow("Annotation Viewer", src.width, src.height)
        cv2.imshow("Annotation Viewer", display)

        # ── Keyboard handling ─────────────────────────────────────────────────
        wait_ms = frame_delay_ms if not paused else 30
        key     = cv2.waitKey(wait_ms) & 0xFF

        if key in (ord('q'), 27):               # Q / Esc — quit
            break

        elif key == ord(' '):                   # Space — toggle play/pause
            paused = not paused

        elif key in (83, ord('d')):             # → / D — next frame
            frame_idx = (frame_idx + 1) % len(src)
            paused = True

        elif key in (81, ord('a')):             # ← / A — prev frame
            frame_idx = (frame_idx - 1) % len(src)
            paused = True

        elif key == ord('\t'):                  # Tab — next sensor source
            source_idx = (source_idx + 1) % len(sources)
            new_src    = sources[source_idx]
            # Clamp frame index in case the new source has fewer frames
            frame_idx  = min(frame_idx, len(new_src) - 1)
            print(f"  Switched to: {new_src.name.upper()} "
                  f"(frame {frame_idx + 1}/{len(new_src)})")
            cv2.resizeWindow("Annotation Viewer", new_src.width, new_src.height)

        elif not paused:                        # Auto-advance during playback
            frame_idx = (frame_idx + 1) % len(src)

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

    print(f"\n  Pose folder : {pose_path}")
    print(f"  Calib root  : {args.calib_root}\n")

    # ── Discover available sensor sources ─────────────────────────────────────
    sources = discover_sources(pose_path, args.calib_root)

    if not sources:
        print("\n[ERROR] No usable annotation sources found.")
        print("        Check that replica_generate_annotations.py has been run")
        print("        and that VIEWER_SENSOR_CONFIGS matches the generated output.")
        sys.exit(1)

    print(f"\n  {len(sources)} source(s) ready: {[s.name for s in sources]}\n")

    # ── Launch viewer ─────────────────────────────────────────────────────────
    run_viewer(sources, play_fps=args.fps,
           label_opacity=args.label_opacity,
           display_width=args.display_width)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="View ground truth 3D bounding box annotations overlaid on "
                    "scan frames. Automatically discovers available sensor sources "
                    "— no sensor flag needed. Press Tab during playback to switch "
                    "between sensors at the same frame index."
    )

    parser.add_argument("--data-root",  default="/home/csrobot/MOAD_DATA",
                        help="Root data directory")
    parser.add_argument("--object",     default="batch1_007",
                        help="Object subfolder name")
    parser.add_argument("--pose",       default="pose-b",
                        help="Pose subfolder name")
    parser.add_argument("--calib-root", default="/home/csrobot/moad_control/moad_cui/calibration/55mm_joint",
                        help="Calibration folder containing cam_parameters files "
                             "(must match entries in VIEWER_SENSOR_CONFIGS)")
    parser.add_argument("--fps",        type=float, default=20.0,
                        help="Playback speed in frames per second (default: 10)")
    
    parser.add_argument("--label-opacity", type=float, default=0.75,
                    help="Opacity of object label backgrounds (0.0–1.0, default: 0.75)")
    parser.add_argument("--display-width", type=int, default=1500,
                        help="Optional display width in pixels. If set, the window is "
                            "scaled to this width while preserving aspect ratio. "
                            "Does not affect projection math. Default: no scaling.")

    args = parser.parse_args()
    main(args)