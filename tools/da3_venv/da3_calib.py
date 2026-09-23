#!/usr/bin/env python3
"""
da3_calib.py
------------
MOAD joint-calibration handling: intrinsics, extrinsics, distortion and
the model-unit <-> meter conversion.

CONVENTIONS (verified against the calibration files)
    * Poses are OpenCV: camera +x right, +y down, +z forward (toward the
      turntable). c2w and w2c are both stored; w2c = inv(c2w).
    * Translations are in MODEL UNITS. One model unit = `scale` meters
      (from the calibration's reference measurement), so a camera 7.0
      units from the origin stands 0.83 m away.
    * The DSLR camera model is COLMAP SIMPLE_RADIAL:
          u = f * x * (1 + k1 r^2) + cx
      with a single k1. Note DA3 (and any pinhole K) has nowhere to put
      k1, which is why images are undistorted before inference and depth
      is optionally redistorted afterwards.

DEPTH AND DISTORTION
    Depth here is always the z-component along the optical axis, not ray
    length. Distortion changes WHICH PIXEL a ray lands on, not the ray's
    z, so (re)distorting a depth map is a pure resampling: the stored
    values are unchanged and only the sampling grid moves.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    """Pinhole + single radial term at a specific image resolution."""
    f: float
    cx: float
    cy: float
    k1: float
    width: int
    height: int

    def K(self) -> np.ndarray:
        return np.array([[self.f, 0.0, self.cx],
                         [0.0, self.f, self.cy],
                         [0.0, 0.0, 1.0]], np.float64)

    def dist_coeffs(self) -> np.ndarray:
        """OpenCV [k1, k2, p1, p2]; SIMPLE_RADIAL uses k1 only."""
        return np.array([self.k1, 0.0, 0.0, 0.0], np.float64)


class Calibration:
    """Loaded cam_parameters.json with unit-aware accessors."""

    def __init__(self, path: str | Path):
        path = Path(path)
        if path.is_dir():
            path = path / "cam_parameters.json"
        if not path.is_file():
            raise FileNotFoundError(f"calibration not found: {path}")
        with open(path) as f:
            self.raw = json.load(f)
        self.path = path

        intr = self.raw["intrinsics"]
        if intr.get("camera_model") != "SIMPLE_RADIAL":
            raise ValueError(f"{path}: unsupported camera model "
                             f"{intr.get('camera_model')!r} (expected SIMPLE_RADIAL)")
        self.intrinsics = intr
        self.k1 = float(intr["distortion"]["k1"])
        self.scale_m = float(self.raw["_info"]["scaling"]["scale"])
        self.cameras = self.raw["cameras"]

    # -- basics ---------------------------------------------------------
    def camera_keys(self) -> list[str]:
        return sorted(self.cameras, key=lambda k: int("".join(c for c in k if c.isdigit())))

    def c2w(self, cam: str) -> np.ndarray:
        """4x4 camera-to-world in MODEL units."""
        self._check(cam)
        return np.array(self.cameras[cam]["extrinsics"]["c2w"], np.float64)

    def w2c(self, cam: str) -> np.ndarray:
        """4x4 world-to-camera in MODEL units."""
        self._check(cam)
        return np.array(self.cameras[cam]["extrinsics"]["w2c"], np.float64)

    def w2c_metric(self, cam: str) -> np.ndarray:
        """
        4x4 world-to-camera with the translation converted to METERS.

        Feed these to a pose-conditioned model together with
        align_to_input_ext_scale=True and the returned depth is metric.
        """
        m = self.w2c(cam).copy()
        m[:3, 3] *= self.scale_m
        return m

    def center_m(self, cam: str) -> np.ndarray:
        """Camera center in meters."""
        return self.c2w(cam)[:3, 3] * self.scale_m

    def _check(self, cam: str) -> None:
        if cam not in self.cameras:
            raise KeyError(f"{cam} not in {self.path} (have: {', '.join(self.camera_keys())})")

    # -- resolution-dependent model -------------------------------------
    def model_at(self, width: int, height: int) -> CameraModel:
        """
        Camera model for an image resized to width x height.

        Pixel-center aware: the center of pixel 0 sits at 0, so the
        principal point scales as (c + 0.5) * s - 0.5.
        """
        intr = self.intrinsics
        sx = width / float(intr["width"])
        sy = height / float(intr["height"])
        return CameraModel(
            f=float(intr["fx"]) * sx,
            cx=(float(intr["cx"]) + 0.5) * sx - 0.5,
            cy=(float(intr["cy"]) + 0.5) * sy - 0.5,
            k1=self.k1, width=width, height=height,
        )

    def describe(self) -> str:
        cams = self.camera_keys()
        d = [np.linalg.norm(self.center_m(c)) for c in cams]
        return (f"{self.path.name}: {len(cams)} cameras ({', '.join(cams)}), "
                f"1 unit = {self.scale_m:.6f} m, k1 = {self.k1:+.4f}, "
                f"standoff {min(d):.3f}-{max(d):.3f} m")


# ---------------------------------------------------------------------------
# Distortion
# ---------------------------------------------------------------------------

def undistort_image(img: np.ndarray, cam: CameraModel) -> np.ndarray:
    """Remove SIMPLE_RADIAL distortion, keeping the same K (no cropping)."""
    K = cam.K()
    return cv2.undistort(img, K, cam.dist_coeffs(), None, K)


def undistort_normalized(xd: np.ndarray, yd: np.ndarray, k1: float,
                         iters: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """
    Invert x_d = x * (1 + k1 r^2) by fixed-point iteration.

    Converges quickly for the magnitudes seen here (k1 ~ 0.36); 10 passes
    leave far less error than the pixel grid itself.
    """
    x, y = np.asarray(xd, np.float64).copy(), np.asarray(yd, np.float64).copy()
    for _ in range(iters):
        d = 1.0 + k1 * (x * x + y * y)
        x, y = xd / d, yd / d
    return x, y


def redistort_maps(K: np.ndarray, k1: float, width: int, height: int
                   ) -> tuple[np.ndarray, np.ndarray]:
    """
    cv2.remap maps that resample an UNDISTORTED image back into RAW
    (distorted) geometry: for each distorted pixel, where does its ray
    land in the undistorted image?

    Use INTER_NEAREST for depth — bilinear would blend across depth
    discontinuities and invent surfaces between foreground and background.
    """
    v, u = np.mgrid[0:height, 0:width].astype(np.float64)
    xd = (u - K[0, 2]) / K[0, 0]
    yd = (v - K[1, 2]) / K[1, 1]
    x, y = undistort_normalized(xd, yd, k1)
    return ((K[0, 0] * x + K[0, 2]).astype(np.float32),
            (K[1, 1] * y + K[1, 2]).astype(np.float32))


def redistort_depth(depth: np.ndarray, K: np.ndarray, k1: float) -> np.ndarray:
    """Resample a depth map from undistorted into raw geometry."""
    h, w = depth.shape[:2]
    mx, my = redistort_maps(K, k1, w, h)
    return cv2.remap(depth, mx, my, cv2.INTER_NEAREST,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)


# ---------------------------------------------------------------------------
# Projection helpers
# ---------------------------------------------------------------------------

def project(points_cam: np.ndarray, cam: CameraModel) -> tuple[np.ndarray, np.ndarray]:
    """Camera-frame points (N,3) -> distorted pixel coordinates."""
    x = points_cam[:, 0] / points_cam[:, 2]
    y = points_cam[:, 1] / points_cam[:, 2]
    d = 1.0 + cam.k1 * (x * x + y * y)
    return cam.f * x * d + cam.cx, cam.f * y * d + cam.cy


def unproject(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Depth map + pinhole K -> camera-frame points, shape (H, W, 3).

    Assumes the depth map is in UNDISTORTED geometry (as produced by a
    pinhole model); redistort afterwards if raw geometry is wanted.
    """
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    x = (u - K[0, 2]) / K[0, 0]
    y = (v - K[1, 2]) / K[1, 1]
    return np.stack([x * depth, y * depth, depth], axis=-1)


def camera_center(w2c: np.ndarray) -> np.ndarray:
    """Camera center from a 3x4 or 4x4 world-to-camera matrix."""
    R, t = w2c[:3, :3], w2c[:3, 3]
    return -R.T @ t


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Inspect a MOAD calibration.")
    ap.add_argument("calib", help="calibration dir or cam_parameters.json")
    ap.add_argument("--width", type=int, default=1500)
    ap.add_argument("--height", type=int, default=1000)
    a = ap.parse_args()
    c = Calibration(a.calib)
    print(c.describe())
    m = c.model_at(a.width, a.height)
    print(f"  at {a.width}x{a.height}: f={m.f:.1f} cx={m.cx:.1f} cy={m.cy:.1f} k1={m.k1:+.4f}")
    for cam in c.camera_keys():
        print(f"    {cam}: center {np.round(c.center_m(cam), 4)} m")
