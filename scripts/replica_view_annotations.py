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

# 16 visually distinct BGR colours — chosen for mutual contrast on both dark
# and light backgrounds. Ordered so adjacent entries are maximally different,
# avoiding repeats for scenes with up to 16 objects.
OBJECT_COLORS = [
    (  0, 200, 255),   #  0  amber
    ( 50, 220,  50),   #  1  green
    (255,  60,  60),   #  2  blue (vivid)
    (200,  50, 255),   #  3  violet
    (  0, 255, 160),   #  4  spring green
    (255, 160,   0),   #  5  sky blue
    (  0, 100, 255),   #  6  orange
    (180, 255,   0),   #  7  lime
    (255,   0, 160),   #  8  hot pink / magenta
    ( 40, 200, 200),   #  9  olive/teal
    (128,   0, 255),   # 10  deep orange
    (  0, 255, 255),   # 11  yellow
    (255,   0,   0),   # 12  pure blue
    (  0, 180, 120),   # 13  dark green
    (200, 200,   0),   # 14  cyan-green
    (100,  60, 200),   # 15  brown-red
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


# Cyclic annotation visualisation modes toggled by V key
VIZ_MODES = ["pose", "bbox_obj", "bbox_visib"]
VIZ_LABELS = {
    "pose":       "POSE  (6D)",
    "bbox_obj":   "BBOX  (full)",
    "bbox_visib": "BBOX  (visible)",
}


def draw_bbox_2d(
    img:   np.ndarray,
    bbox:  list[int],
    color: tuple,
    label: str = "",
    label_opacity: float = 0.75,
) -> np.ndarray:
    """
    Draw a 2D axis-aligned bounding box [x, y, w, h] (BOP convention,
    top-left origin) onto a copy of img.
    """
    img = img.copy()
    x, y, w, h = bbox
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2, cv2.LINE_AA)
    if label:
        _draw_label(img, label, (x, y - 4 if y > 12 else y + h + 12),
                    color, label_opacity)
    return img


def draw_occlusion_panel(
    img:        np.ndarray,
    objects:    list[dict],
    colors:     list[tuple],
    bar_width:  int = 18,
    bar_height: int = 80,
    margin:     int = 8,
) -> np.ndarray:
    """
    Draw a compact occlusion panel in the top-right corner of img.

    Each object gets a vertical bar whose filled height represents its
    visible fraction (0–100 %). The bar is split vertically:
      - filled portion (bright colour)  = visible fraction
      - empty portion (dark colour)     = occluded fraction

    A small percentage label is drawn below each bar.

    Args:
        img        : BGR image (modified in-place copy returned)
        objects    : list of annotation object dicts (must have visib_fract)
        colors     : parallel list of BGR colours, one per object
        bar_width  : pixel width of each bar
        bar_height : total pixel height of each bar (100 % = full height)
        margin     : gap from the right/top image edge and between bars
    """
    img = img.copy()
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    n = len(objects)
    if n == 0:
        return img

    panel_w = n * bar_width + (n + 1) * margin
    panel_h = bar_height + margin * 3 + 14   # bar + top/bottom margin + text

    # Panel origin (top-right, with margin from edge)
    px0 = w - panel_w - margin
    py0 = 30   # sit just below the top hint bar

    # Semi-transparent panel background
    overlay = img.copy()
    cv2.rectangle(overlay, (px0, py0), (px0 + panel_w, py0 + panel_h),
                  (20, 20, 20), cv2.FILLED)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

    for i, (obj, color) in enumerate(zip(objects, colors)):
        frac = float(obj.get("visib_fract", 0.0))
        frac = max(0.0, min(1.0, frac))

        bx = px0 + margin + i * (bar_width + margin)
        by = py0 + margin

        # Dark background bar (full height = 0 % visible)
        dark = tuple(max(0, int(c * 0.25)) for c in color)
        cv2.rectangle(img,
                      (bx, by),
                      (bx + bar_width, by + bar_height),
                      dark, cv2.FILLED)

        # Filled bar from the bottom up (fill height proportional to frac)
        filled_h = int(round(frac * bar_height))
        if filled_h > 0:
            cv2.rectangle(img,
                          (bx, by + bar_height - filled_h),
                          (bx + bar_width, by + bar_height),
                          color, cv2.FILLED)

        # Percentage text below the bar
        pct_str = f"{int(round(frac * 100))}%"
        (tw, _), _ = cv2.getTextSize(pct_str, font, 0.32, 1)
        tx = bx + (bar_width - tw) // 2
        cv2.putText(img, pct_str,
                    (tx, by + bar_height + margin + 6),
                    font, 0.32, color, 1, cv2.LINE_AA)

    return img


def draw_overlay(
    img:           np.ndarray,
    annotation:    dict | None,
    cam_K:         np.ndarray,
    sensor_name:   str,
    frame_idx:     int,
    total_frames:  int,
    paused:        bool,
    source_count:  int,
    viz_mode:      str   = "pose",
    label_opacity: float = 0.75,
) -> np.ndarray:
    """
    Composite all annotation overlays and UI chrome onto a copy of img.

    viz_mode controls what geometry is drawn per object:
        "pose"       — 3D projected bounding box (6D pose visualisation)
        "bbox_obj"   — 2D axis-aligned box of full object silhouette
        "bbox_visib" — 2D axis-aligned box of visible object region

    An occlusion panel is drawn in the top-right corner whenever
    visib_fract data is present in the annotation.
    """
    img = img.copy()
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    objects = annotation.get("objects", []) if annotation else []

    # Assign colours once so the occlusion panel matches the drawn boxes
    obj_colors = [
        OBJECT_COLORS[i % len(OBJECT_COLORS)] for i in range(len(objects))
    ]

    # ── Per-object geometry ───────────────────────────────────────────────────
    for obj, color in zip(objects, obj_colors):
        label = obj.get("object_name", "?")

        if viz_mode == "pose":
            R    = np.array(obj["R"], dtype=np.float64)
            t    = np.array(obj["t"], dtype=np.float64)
            size = obj.get("size", [0.1, 0.1, 0.1])
            img  = draw_bbox_3d(img, R, t, size, cam_K, color, label,
                                label_opacity=label_opacity)

        elif viz_mode == "bbox_obj":
            bbox = obj.get("bbox_obj")
            if bbox:
                img = draw_bbox_2d(img, bbox, color, label, label_opacity)

        elif viz_mode == "bbox_visib":
            bbox = obj.get("bbox_visib")
            if bbox:
                img = draw_bbox_2d(img, bbox, color, label, label_opacity)

    # ── Occlusion panel (top-right) ───────────────────────────────────────────
    # Only shown when at least one object has visib_fract in its annotation
    if objects and any("visib_fract" in o for o in objects):
        img = draw_occlusion_panel(img, objects, obj_colors)

    # ── Status bar (bottom) ───────────────────────────────────────────────────
    bar_h   = 36
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    if annotation:
        frame_name    = annotation.get("frame",         "—")
        cam_key       = annotation.get("cam_key",       "?")
        turntable_deg = annotation.get("turntable_deg", 0.0)
        play_state    = "[PAUSED]" if paused else ""
        status = (f"  {play_state}  {sensor_name.upper()}  |  {frame_name}  |  "
                  f"cam: {cam_key}  |  turntable: {turntable_deg:.1f}deg  |  "
                  f"frame {frame_idx + 1}/{total_frames}")
    else:
        status = (f"  {'[PAUSED]' if paused else ''}  {sensor_name.upper()}  |  "
                  f"frame {frame_idx + 1}/{total_frames}  |  NO ANNOTATION")

    cv2.putText(img, status, (8, h - bar_h // 2 + 5),
                font, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

    # ── Top hint bar ──────────────────────────────────────────────────────────
    cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), cv2.FILLED)

    # Current viz mode badge (top-left, green)
    mode_label = VIZ_LABELS.get(viz_mode, viz_mode.upper())
    cv2.putText(img, f"[ {mode_label} ]", (8, 16),
                font, 0.45, (80, 200, 80), 1, cv2.LINE_AA)

    # Controls hint
    tab_hint = "  Tab: sensor" if source_count > 1 else ""
    hint = f"Space: play/pause    A/D: step    V: viz mode{tab_hint}    Q: quit"
    cv2.putText(img, hint, (200, 16),
                font, 0.38, (130, 130, 130), 1, cv2.LINE_AA)

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

    Maintains a source index (which sensor), a frame index (which frame
    within that sensor), and a viz mode index (which annotation type to draw).
    Tab cycles sensors, V cycles viz modes, both preserve the frame position.
    """
    source_idx   = 0
    frame_idx    = 0
    viz_mode_idx = 0
    paused       = False
    frame_delay_ms = max(1, int(1000.0 / play_fps))

    src = sources[source_idx]
    cv2.namedWindow("Annotation Viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Annotation Viewer", src.width, src.height)

    print(f"\n  {len(sources)} sensor source(s): {[s.name for s in sources]}")
    print(f"  Space: play/pause  |  A/D: step  |  V: viz mode  "
          f"|  Tab: next sensor  |  Q/Esc: quit\n")

    while True:
        src      = sources[source_idx]
        viz_mode = VIZ_MODES[viz_mode_idx]
        img_path = src.image_paths[frame_idx]
        ann_path = src.annot_path_for(img_path)

        # ── Load image ────────────────────────────────────────────────────────
        bgr = cv2.imread(img_path)
        if bgr is None:
            print(f"  [WARN] Cannot read {img_path}")
            frame_idx = (frame_idx + 1) % len(src)
            continue

        if bgr.shape[1] != src.width or bgr.shape[0] != src.height:
            bgr = cv2.resize(bgr, (src.width, src.height),
                             interpolation=cv2.INTER_LINEAR)

        # ── Load annotation ───────────────────────────────────────────────────
        annotation = None
        if os.path.isfile(ann_path):
            with open(ann_path, "r") as f:
                annotation = json.load(f)
        else:
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
            viz_mode     = viz_mode,
            label_opacity = label_opacity,
        )

        if display_width is not None:
            scale   = display_width / display.shape[1]
            disp_h  = int(display.shape[0] * scale)
            display = cv2.resize(display, (display_width, disp_h),
                                 interpolation=cv2.INTER_LINEAR)
            cv2.resizeWindow("Annotation Viewer", display_width, disp_h)
        else:
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

        elif key == ord('v'):                   # V — cycle viz mode
            viz_mode_idx = (viz_mode_idx + 1) % len(VIZ_MODES)
            new_mode = VIZ_MODES[viz_mode_idx]
            print(f"  Viz mode: {VIZ_LABELS[new_mode]}")

        elif key == ord('\t'):                  # Tab — next sensor
            source_idx = (source_idx + 1) % len(sources)
            new_src    = sources[source_idx]
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
    
    parser.add_argument("--label-opacity", type=float, default=0.5,
                    help="Opacity of object label backgrounds (0.0–1.0, default: 0.75)")
    parser.add_argument("--display-width", type=int, default=1500,
                        help="Optional display width in pixels. If set, the window is "
                            "scaled to this width while preserving aspect ratio. "
                            "Does not affect projection math. Default: no scaling.")

    args = parser.parse_args()
    main(args)