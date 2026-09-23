#!/usr/bin/env python3
"""
colmap_depth_video.py
---------------------
Render COLMAP patch-match depth maps (*.photometric.bin / *.geometric.bin)
into a single video with a shared color scale, per-frame annotations,
and an optional side-by-side view with the undistorted RGB frame.

Frames are sorted by filename. For MOAD's camera-major naming this plays
each camera's full turntable orbit in turn (cam1 x72, then cam2 x72, ...).

USAGE
    # all defaults (MOAD ex2_006/pose-a paths, geometric, model units)
    python3 colmap_depth_video.py

    # metric depth (meters) using the calibration scale factor
    python3 colmap_depth_video.py --scale 0.11923018414896373

    # RealSense-reprojected depth (DSLR_depth/*.png, uint16 mm -> meters)
    python3 colmap_depth_video.py --mode png \
        --depth-dir  ~/MOAD_DATA/ex2_006/pose-b/DSLR_depth \
        --images-dir ~/MOAD_DATA/ex2_006/pose-b/images_4 \
        --output-dir ~/MOAD_DATA/ex2_006/pose-b/output

    # disable the RGB side-by-side
    python3 colmap_depth_video.py --images-dir none

    # auto color scale (2nd..98th percentile of valid depths, all frames)
    python3 colmap_depth_video.py \\
        --depth-dir  ~/MOAD_DATA/ex2_006/pose-a/colmap/dense/stereo/depth_maps \\
        --mode geometric \\
        --output-dir ~/MOAD_DATA/ex2_006/pose-a/colmap/videos

    # fixed range (model units) + RGB side-by-side
    python3 colmap_depth_video.py --depth-dir .../depth_maps --mode photometric \\
        --output-dir .../videos --vmin 5.5 --vmax 8.5 \\
        --images-dir ~/MOAD_DATA/ex2_006/pose-a/colmap/dense/images
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np


DEFAULT_DIR   = "/home/csrobot/MOAD_DATA/ex2_006/pose-a/colmap/dense/stereo/depth_maps"
DEFAULT_IMAGE = "/home/csrobot/MOAD_DATA/ex2_006/pose-a/colmap/dense/images"  # "none" disables
DEFAULT_OUT   = "/home/csrobot/MOAD_DATA/ex2_006/pose-a/colmap/output"
DEFAULT_SCALE = 1.0   # 1.0 = model units; calibration metric scale = 0.11923018414896373

INVALID_COLOR = (40, 40, 40)   # BGR, drawn where depth <= 0
COLORBAR_W    = 70             # px, right-hand colorbar panel


# ---------------------------------------------------------------------------
# COLMAP dense array I/O
# ---------------------------------------------------------------------------

def read_colmap_array(path: str) -> np.ndarray:
    """Read a COLMAP dense .bin array (header 'w&h&c&' + column-major float32)."""
    with open(path, "rb") as f:
        header = b""
        while header.count(b"&") < 3:
            ch = f.read(1)
            if not ch:
                raise ValueError(f"truncated header in {path}")
            header += ch
        w, h, c = map(int, header.decode().split("&")[:3])
        data = np.fromfile(f, np.float32)
    if data.size != w * h * c:
        raise ValueError(f"{path}: expected {w*h*c} values, got {data.size}")
    arr = data.reshape((w, h, c), order="F")
    return np.transpose(arr, (1, 0, 2)).squeeze()


PNG_UNIT = 0.001   # DSLR_depth PNGs are uint16 millimeters


def read_depth(path: str, scale: float) -> np.ndarray:
    """
    Read a depth map and apply a uniform scale (invalid zeros stay zero).
    .bin  -> COLMAP float32 array (model units) * scale
    .png  -> uint16 millimeters -> meters (scale ignored; already metric)
    """
    if path.endswith(".png"):
        d = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if d is None:
            raise ValueError(f"cannot read {path}")
        return d.astype(np.float32) * np.float32(PNG_UNIT)
    return read_colmap_array(path) * np.float32(scale)


# ---------------------------------------------------------------------------
# Color scale
# ---------------------------------------------------------------------------

def compute_global_range(
    paths: list[Path],
    lo_pct: float,
    hi_pct: float,
    scale: float = 1.0,
    samples_per_frame: int = 20000,
    seed: int = 0,
) -> tuple[float, float]:
    """
    Robust shared color range: pool a random sample of valid depths from
    every frame, then take percentiles. Avoids loading all maps at once
    and ignores the huge photometric outliers. Depths are scaled first,
    so the returned range is in output units.
    """
    rng = np.random.default_rng(seed)
    pooled = []
    for p in paths:
        d = read_depth(str(p), scale)
        v = d[d > 0]
        if v.size == 0:
            continue
        if v.size > samples_per_frame:
            v = rng.choice(v, samples_per_frame, replace=False)
        pooled.append(v)
    if not pooled:
        raise RuntimeError("no valid depth pixels in ANY frame — nothing to scale")
    pooled = np.concatenate(pooled)
    vmin, vmax = np.percentile(pooled, [lo_pct, hi_pct])
    return float(vmin), float(vmax)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def colorize(depth: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
    """Depth -> BGR via TURBO; invalid (<=0) pixels painted INVALID_COLOR."""
    valid = depth > 0
    norm = np.clip((depth - vmin) / max(vmax - vmin, 1e-12), 0.0, 1.0)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = INVALID_COLOR
    return img


def make_colorbar(height: int, vmin: float, vmax: float, units: str) -> np.ndarray:
    """Vertical colorbar panel with max at top, min at bottom."""
    panel = np.full((height, COLORBAR_W, 3), 20, np.uint8)
    top, bot = 30, height - 30
    grad = np.linspace(255, 0, bot - top).astype(np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(np.repeat(grad, 18, axis=1), cv2.COLORMAP_TURBO)
    panel[top:bot, 8:26] = bar
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(panel, units, (8, 18), font, 0.4, (220, 220, 220), 1, cv2.LINE_AA)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = int(top + frac * (bot - top - 1))
        val = vmax - frac * (vmax - vmin)
        cv2.line(panel, (26, y), (31, y), (220, 220, 220), 1)
        cv2.putText(panel, f"{val:.3g}", (33, y + 4), font, 0.35,
                    (220, 220, 220), 1, cv2.LINE_AA)
    return panel


def annotate(img: np.ndarray, lines: list[str]) -> None:
    """Draw text lines top-left with a dark backing box for legibility."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
    y = 10
    for text in lines:
        (tw, th), base = cv2.getTextSize(text, font, scale, thick)
        cv2.rectangle(img, (6, y), (14 + tw, y + th + base + 6), (0, 0, 0), -1)
        cv2.putText(img, text, (10, y + th + 3), font, scale,
                    (255, 255, 255), thick, cv2.LINE_AA)
        y += th + base + 10


def find_rgb(images_dir: Path | None, image_name: str) -> np.ndarray | None:
    if images_dir is None:
        return None
    p = images_dir / image_name
    return cv2.imread(str(p)) if p.is_file() else None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Render COLMAP depth maps to video.")
    ap.add_argument("--depth-dir", default=DEFAULT_DIR,
                    help="dense/stereo/depth_maps directory")
    ap.add_argument("--mode", default="geometric",
                    choices=["photometric", "geometric", "png"],
                    help="COLMAP maps (photometric/geometric .bin) or "
                         "DSLR_depth uint16-mm PNGs (png)")
    ap.add_argument("--output-dir", default=DEFAULT_OUT)
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help="multiply all depths by this factor before scaling "
                         "and rendering (e.g. calibration scale for meters; "
                         "default 1.0 = model units)")
    ap.add_argument("--vmin", type=float, default=None,
                    help="color range min, in OUTPUT units (after --scale); "
                         "auto if omitted")
    ap.add_argument("--vmax", type=float, default=None,
                    help="color range max, in OUTPUT units (after --scale); "
                         "auto if omitted")
    ap.add_argument("--pct", type=float, nargs=2, default=(2.0, 98.0),
                    metavar=("LO", "HI"),
                    help="percentiles for auto range (default 2 98)")
    ap.add_argument("--images-dir", default=DEFAULT_IMAGE,
                    help="undistorted RGB dir (dense/images) for side-by-side "
                         "output; pass 'none' to disable")
    ap.add_argument("--fps", type=float, default=10.0)
    args = ap.parse_args()

    depth_dir = Path(args.depth_dir).expanduser()
    out_dir   = Path(args.output_dir).expanduser()
    suffix = ".png" if args.mode == "png" else f".{args.mode}.bin"

    images_dir = None
    if args.images_dir and args.images_dir.lower() != "none":
        images_dir = Path(args.images_dir).expanduser()
        if not images_dir.is_dir():
            print(f"  [WARN] images dir not found ({images_dir}) — "
                  f"RGB side-by-side disabled")
            images_dir = None

    if args.mode == "png" and args.scale != 1.0:
        print(f"  [WARN] --scale ignored in png mode (PNG depth is already metric)")
        args.scale = 1.0
    if args.scale <= 0:
        print(f"[ERROR] --scale must be positive, got {args.scale}")
        sys.exit(1)
    units = ("meters (PNG mm)" if args.mode == "png" else
             "model units" if args.scale == 1.0 else f"scaled x{args.scale:.6g}")
    print(f"  Depth scale         : {args.scale:.10g}  ({units})")

    paths = sorted(p for p in depth_dir.glob(f"*{suffix}"))
    if not paths:
        print(f"[ERROR] no *{suffix} files in {depth_dir}")
        sys.exit(1)
    print(f"  Found {len(paths)} {args.mode} depth maps in {depth_dir}")
    print(f"  First: {paths[0].name}   Last: {paths[-1].name}")

    # ── Color range ──────────────────────────────────────────────────────
    if args.vmin is not None and args.vmax is not None:
        vmin, vmax = args.vmin, args.vmax
        print(f"  Color range (user)  : {vmin:.3f} .. {vmax:.3f}")
    else:
        print(f"  Computing shared color range (p{args.pct[0]:g}..p{args.pct[1]:g})...")
        auto_min, auto_max = compute_global_range(paths, *args.pct,
                                                  scale=args.scale)
        vmin = args.vmin if args.vmin is not None else auto_min
        vmax = args.vmax if args.vmax is not None else auto_max
        print(f"  Color range (auto)  : {vmin:.3f} .. {vmax:.3f}")
    if vmax <= vmin:
        print(f"[ERROR] invalid color range {vmin} .. {vmax}")
        sys.exit(1)

    # ── Video writer (size fixed from first frame) ───────────────────────
    first = read_depth(str(paths[0]), 1.0)     # shape only, scale irrelevant
    h, w = first.shape[:2]
    frame_w = w * (2 if images_dir else 1) + COLORBAR_W
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = "" if (args.scale == 1.0 or args.mode == "png") else "_scaled"
    out_path = out_dir / f"depth_{args.mode}{tag}.mp4"
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (frame_w, h))
    if not writer.isOpened():
        print(f"[ERROR] could not open video writer for {out_path}")
        sys.exit(1)
    bar_units = ("m" if args.mode == "png" else
                 "model" if args.scale == 1.0 else "scaled")
    colorbar = make_colorbar(h, vmin, vmax, bar_units)
    print(f"  Output: {out_path}  ({frame_w}×{h} @ {args.fps:g} fps)")

    # ── Render ───────────────────────────────────────────────────────────
    valid_fracs, missing_rgb = [], 0
    for i, p in enumerate(paths):
        depth = read_depth(str(p), args.scale)
        if depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        if args.mode == "png":                       # frame_00001.png -> .jpg
            image_name = p.stem + ".jpg"
        else:
            image_name = p.name[: -len(suffix)]      # e.g. frame_00001.jpg
        valid_frac = float((depth > 0).mean())
        valid_fracs.append(valid_frac)

        vis = colorize(depth, vmin, vmax)
        valid_d = depth[depth > 0]
        med = f"  median {np.median(valid_d):.3g}" if valid_d.size else ""
        annotate(vis, [image_name,
                       f"{args.mode}  valid {valid_frac*100:.1f}%{med}"])

        panels = [vis]
        if images_dir is not None:
            rgb = find_rgb(images_dir, image_name)
            if rgb is None:
                missing_rgb += 1
                rgb = np.zeros_like(vis)
            else:
                rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
            panels.insert(0, rgb)
        panels.append(colorbar)
        writer.write(np.hstack(panels))

        if (i + 1) % 50 == 0 or i + 1 == len(paths):
            print(f"  rendered {i+1}/{len(paths)}")

    writer.release()

    # ── Summary ──────────────────────────────────────────────────────────
    vf = np.array(valid_fracs)
    print(f"\n  Valid coverage: mean {vf.mean()*100:.1f}%  "
          f"min {vf.min()*100:.1f}% ({paths[int(vf.argmin())].name})  "
          f"max {vf.max()*100:.1f}%")
    near_empty = [paths[i].name for i in np.flatnonzero(vf < 0.01)]
    if near_empty:
        print(f"  [WARN] {len(near_empty)} frames under 1% valid: "
              f"{', '.join(near_empty[:10])}"
              f"{' ...' if len(near_empty) > 10 else ''}")
    if missing_rgb:
        print(f"  [WARN] {missing_rgb} frames had no matching RGB in {images_dir}")
    print(f"  ✓ wrote {out_path}")


if __name__ == "__main__":
    main()
