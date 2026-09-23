#!/usr/bin/env python3
"""
da3_io.py
---------
Output contract for MOAD depth products. Deliberately backend-agnostic:
the RealSense reprojection and COLMAP stereo pipelines can write the same
artifacts so downstream tools (videos, per-pixel comparisons, training
loaders) need no per-backend special cases.

DEPTH PNG CONTRACT
    uint16, one channel, MILLIMETERS, 0 = invalid/no data.
    Values are z-depth along the camera's optical axis (not ray length).
    Geometry is either "raw" (distorted, pixel-aligned with images_N) or
    "undistorted" (pinhole), recorded in meta.json — the pixels alone do
    not tell you which, so the meta is not optional.

META SCHEMA ("moad.depth/1")
    Written once per output directory as meta.json:

        schema, backend, created_utc, scan, calibration, model,
        settings{...}, config_hash, geometry, units, depth_scale_mm,
        pose_source, metric, output_size, frames{ name -> stats }

    `pose_source` and `metric` are the load-bearing fields: depth from an
    unconditioned run is scale-free and in the model's own frame, so it
    must never be mixed into a directory of metric, calibrated depth.
    check_compatible() enforces exactly that on resume.
"""

from __future__ import annotations

import datetime as _dt
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

SCHEMA = "moad.depth/1"
DEPTH_SCALE_MM = 1000.0          # meters -> stored uint16 counts
META_NAME = "meta.json"

# Fields that must match when appending to an existing output directory.
_COMPAT_KEYS = ("schema", "backend", "geometry", "units", "depth_scale_mm",
                "pose_source", "metric", "output_size")


# ---------------------------------------------------------------------------
# Depth maps
# ---------------------------------------------------------------------------

def write_depth_png(path: str | Path, depth_m: np.ndarray) -> None:
    """Write metric depth (meters, <=0 invalid) as uint16 millimeters."""
    d = np.asarray(depth_m, np.float64)
    valid = np.isfinite(d) & (d > 0)
    png = np.where(valid, np.clip(np.round(d * DEPTH_SCALE_MM), 0, 65535), 0)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), png.astype(np.uint16)):
        raise IOError(f"failed to write {path}")


def read_depth_png(path: str | Path) -> np.ndarray:
    """Read a depth PNG back into meters as float32 (0 stays 0 = invalid)."""
    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise IOError(f"cannot read {path}")
    if raw.dtype != np.uint16:
        raise ValueError(f"{path}: expected uint16, got {raw.dtype}")
    return raw.astype(np.float32) / np.float32(DEPTH_SCALE_MM)


def depth_stats(depth_m: np.ndarray) -> dict[str, float]:
    """Per-frame summary stored in meta.json (cheap, but enough to triage)."""
    d = np.asarray(depth_m, np.float64)
    valid = np.isfinite(d) & (d > 0)
    if not valid.any():
        return {"valid": 0.0, "median_m": 0.0, "p5_m": 0.0, "p95_m": 0.0}
    v = d[valid]
    return {"valid": float(valid.mean()),
            "median_m": float(np.median(v)),
            "p5_m": float(np.percentile(v, 5)),
            "p95_m": float(np.percentile(v, 95))}


# ---------------------------------------------------------------------------
# Point clouds
# ---------------------------------------------------------------------------

def write_ply(path: str | Path, xyz: np.ndarray, rgb: np.ndarray | None = None) -> None:
    """Binary little-endian PLY; rgb is Nx3 uint8 (RGB order) or None."""
    xyz = np.asarray(xyz, np.float32)
    if rgb is None:
        rgb = np.full((len(xyz), 3), 200, np.uint8)
    rgb = np.asarray(rgb, np.uint8)
    if len(rgb) != len(xyz):
        raise ValueError(f"{len(xyz)} points but {len(rgb)} colors")
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec["red"], rec["green"], rec["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 f"element vertex {len(xyz)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                 "end_header\n").encode())
        rec.tofile(f)


def frustum_points(c2w: np.ndarray, size: float, color, samples: int = 60
                   ) -> tuple[np.ndarray, np.ndarray]:
    """
    Camera frustum drawn as sampled points along its edges.

    Points rather than PLY edge elements because point support is
    universal across viewers (CloudCompare, MeshLab, Open3D).
    """
    R, C = c2w[:3, :3], c2w[:3, 3]
    corners = np.array([[0.6, 0.4, 1.0], [-0.6, 0.4, 1.0],
                        [-0.6, -0.4, 1.0], [0.6, -0.4, 1.0]]) * size @ R.T + C
    segments = ([(C, c) for c in corners]
                + [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
                + [(C, C + R[:, 2] * size * 1.6)])          # optical axis
    t = np.linspace(0, 1, samples)[:, None]
    pts = np.concatenate([a + (b - a) * t for a, b in segments])
    return pts, np.tile(np.asarray(color, np.uint8), (len(pts), 1))


# ---------------------------------------------------------------------------
# Previews
# ---------------------------------------------------------------------------

def colorize_depth(depth_m: np.ndarray, vmin: float | None = None,
                   vmax: float | None = None) -> np.ndarray:
    """Turbo colormap with invalid pixels in dark gray (BGR image)."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if vmin is None or vmax is None:
        lo, hi = (np.percentile(depth_m[valid], [2, 98]) if valid.any() else (0.0, 1.0))
        vmin = lo if vmin is None else vmin
        vmax = hi if vmax is None else vmax
    norm = np.clip((depth_m - vmin) / max(vmax - vmin, 1e-9), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = (40, 40, 40)
    return img


def write_preview(path: str | Path, rgb_bgr: np.ndarray, depth_m: np.ndarray,
                  label: str, conf: np.ndarray | None = None) -> None:
    """RGB | depth | blend (| confidence) strip for eyeballing alignment."""
    h, w = depth_m.shape[:2]
    rgb = cv2.resize(rgb_bgr, (w, h), interpolation=cv2.INTER_AREA)
    dv = colorize_depth(depth_m)
    valid = np.isfinite(depth_m) & (depth_m > 0)
    blend = rgb.copy()
    blend[valid] = (0.45 * rgb[valid] + 0.55 * dv[valid]).astype(np.uint8)
    panels = [rgb, dv, blend]
    if conf is not None:
        c = np.asarray(conf, np.float64)
        span = max(float(np.ptp(c)), 1e-9)
        cv_img = cv2.applyColorMap(
            (np.clip((c - c.min()) / span, 0, 1) * 255).astype(np.uint8),
            cv2.COLORMAP_VIRIDIS)
        panels.append(cv2.resize(cv_img, (w, h), interpolation=cv2.INTER_NEAREST))
    panel = np.hstack(panels)
    stats = depth_stats(depth_m)
    txt = f"{label}  valid {stats['valid']*100:.1f}%  median {stats['median_m']:.3f} m"
    cv2.rectangle(panel, (0, 0), (10 + 9 * len(txt), 28), (0, 0, 0), -1)
    cv2.putText(panel, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), panel)


# ---------------------------------------------------------------------------
# meta.json
# ---------------------------------------------------------------------------

def build_meta(*, backend: str, scan: str | Path, calibration: str | Path,
               geometry: str, metric: bool, pose_source: str,
               output_size: tuple[int, int], settings: dict[str, Any],
               config_hash: str, model: str | None = None,
               units: str = "meters") -> dict:
    """Assemble a schema-compliant meta dict (frames filled in by the caller)."""
    if geometry not in ("raw", "undistorted"):
        raise ValueError(f"geometry must be 'raw' or 'undistorted', got {geometry!r}")
    return {
        "schema": SCHEMA,
        "backend": backend,
        "created_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "scan": str(scan),
        "calibration": str(calibration),
        "model": model,
        "geometry": geometry,
        "units": units,
        "depth_scale_mm": DEPTH_SCALE_MM,
        "depth_format": "uint16 PNG, z-depth, 0 = invalid",
        "pose_source": pose_source,
        "metric": bool(metric),
        "output_size": [int(output_size[0]), int(output_size[1])],
        "settings": settings,
        "config_hash": config_hash,
        "frames": {},
    }


def read_meta(out_dir: str | Path) -> dict | None:
    p = Path(out_dir) / META_NAME
    if not p.is_file():
        return None
    with open(p) as f:
        return json.load(f)


def write_meta(out_dir: str | Path, meta: dict) -> Path:
    p = Path(out_dir)
    p.mkdir(parents=True, exist_ok=True)
    path = p / META_NAME
    with open(path, "w") as f:
        json.dump(meta, f, indent=2, sort_keys=False)
    return path


def check_compatible(existing: dict | None, new: dict) -> list[str]:
    """
    Conflicts between an existing output dir and a new run.

    Empty list means it is safe to append. A config_hash difference is
    reported too: same geometry and units, but different model or
    resolution, which makes for a silently inhomogeneous dataset.
    """
    if existing is None:
        return []
    conflicts = []
    for key in _COMPAT_KEYS:
        old, cur = existing.get(key), new.get(key)
        if old != cur:
            conflicts.append(f"{key}: existing {old!r} != new {cur!r}")
    if existing.get("config_hash") != new.get("config_hash"):
        conflicts.append(
            f"config_hash: existing {existing.get('config_hash')} != "
            f"new {new.get('config_hash')} (different model/resolution/settings)")
    return conflicts


def merge_frames(existing: dict | None, meta: dict) -> dict:
    """Carry forward frame stats from a previous run (for resumed scans)."""
    if existing and isinstance(existing.get("frames"), dict):
        merged = dict(existing["frames"])
        merged.update(meta.get("frames", {}))
        meta["frames"] = merged
    return meta
