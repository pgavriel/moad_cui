#!/usr/bin/env python3
"""
da3_inference.py
----------------
Model wrapper and the per-group pipeline shared by the run_* scripts.

A "group" is one turntable position: the N DSLR views captured
simultaneously, which see a mutually rigid scene and so can be predicted
together.

Load the model ONCE and reuse it:

    runner = DA3Runner(model_name, device="auto")
    for group in groups:
        result = runner.run_group(inputs)

KEY CONTRACTS (learned the hard way, do not "simplify")
    * inference() ACCEPTS 4x4 extrinsics but RETURNS 3x4. Passing 3x4 in
      fails inside _normalize_extrinsics with a batched-matmul error.
    * Extrinsics passed in must be METRIC for the returned depth to be
      metric, together with align_to_input_ext_scale=True.
    * intrinsics is a pinhole K with no distortion term, so images must
      be undistorted first and the same K passed alongside.
    * The model works at process_res (long side); input image size does
      not raise output resolution.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from da3_calib import Calibration, camera_center, undistort_image
from da3_frames import Frame


# ---------------------------------------------------------------------------
# Group inputs / results
# ---------------------------------------------------------------------------

@dataclass
class GroupInput:
    """Everything one inference call needs for a single turntable position."""
    position_deg: int
    frames: list[Frame]
    images_rgb: list[np.ndarray]                 # undistorted, RGB
    extrinsics: np.ndarray | None = None         # (N,4,4) metric w2c, or None
    intrinsics: np.ndarray | None = None         # (N,3,3) pinhole K, or None
    image_size: tuple[int, int] = (0, 0)         # (width, height) of images_rgb

    @property
    def pose_conditioned(self) -> bool:
        return self.extrinsics is not None


@dataclass
class GroupResult:
    """Raw model output for one group, plus a pose sanity measurement."""
    depth: np.ndarray                            # (N,H,W) float32
    conf: np.ndarray                             # (N,H,W) float32
    extrinsics: np.ndarray                       # (N,3,4) returned w2c
    intrinsics: np.ndarray                       # (N,3,3) returned K
    pose_deviation_mm: float = 0.0               # max |returned - input| center
    metric: bool = False
    seconds: float = 0.0
    extras: dict = field(default_factory=dict)

    @property
    def size(self) -> tuple[int, int]:
        """(width, height) of the predicted maps."""
        return int(self.depth.shape[2]), int(self.depth.shape[1])


# ---------------------------------------------------------------------------
# Input preparation
# ---------------------------------------------------------------------------

def prepare_group(frames: list[Frame], scan_dir: Path, images_subdir: str,
                  calib: Calibration | None, *, undistort: bool = True,
                  pose_source: str = "calibrated") -> GroupInput:
    """
    Load and prepare one turntable position's views.

    pose_source:
        "calibrated" -> metric 4x4 w2c + pinhole K per view (conditioned)
        "none"       -> images only; the model estimates poses and the
                        result is scale-free in its own frame
    """
    if not frames:
        raise ValueError("prepare_group called with no frames")
    if pose_source == "calibrated" and calib is None:
        raise ValueError("pose_source='calibrated' requires a calibration")

    images, exts, ixts = [], [], []
    size = None
    for fr in frames:
        path = fr.image_path(scan_dir, images_subdir)
        bgr = cv2.imread(str(path))
        if bgr is None:
            raise FileNotFoundError(f"cannot read image {path}")
        h, w = bgr.shape[:2]
        if size is None:
            size = (w, h)
        elif size != (w, h):
            raise ValueError(f"{path} is {w}x{h}, expected {size[0]}x{size[1]} "
                             f"— all views in a group must match")

        cam_model = calib.model_at(w, h) if calib is not None else None
        if undistort:
            if cam_model is None:
                raise ValueError("undistort=True requires a calibration")
            bgr = undistort_image(bgr, cam_model)
        images.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

        if pose_source == "calibrated":
            exts.append(calib.w2c_metric(fr.camera_key))     # 4x4, meters
            ixts.append(cam_model.K())

    return GroupInput(
        position_deg=frames[0].position_deg,
        frames=list(frames),
        images_rgb=images,
        extrinsics=np.array(exts, np.float32) if exts else None,
        intrinsics=np.array(ixts, np.float32) if ixts else None,
        image_size=size or (0, 0),
    )


# ---------------------------------------------------------------------------
# Model wrapper
# ---------------------------------------------------------------------------

class DA3Runner:
    """Holds a loaded Depth Anything 3 model across many groups."""

    def __init__(self, model_name: str, device: str = "auto", verbose: bool = True):
        import time
        from depth_anything_3.api import DepthAnything3

        self.model_name = model_name
        self._torch = None
        try:
            import torch
            self._torch = torch
            if device == "auto":
                device = "cuda" if torch.cuda.is_available() else "cpu"
            self.device = torch.device(device)
        except ImportError:                       # allows stub-based testing
            self.device = None

        t0 = time.time()
        model = DepthAnything3.from_pretrained(model_name)
        self.model = model.to(device=self.device) if self.device is not None else model
        self.load_seconds = time.time() - t0
        if verbose:
            print(f"  model {model_name} loaded in {self.load_seconds:.1f}s "
                  f"on {self.device}")

    # -- info -----------------------------------------------------------
    def vram_peak_gib(self) -> float | None:
        t = self._torch
        if t is None or not t.cuda.is_available():
            return None
        return t.cuda.max_memory_allocated() / 2 ** 30

    # -- inference ------------------------------------------------------
    def run_group(self, group: GroupInput, *, process_res: int = 504,
                  align_to_input_ext_scale: bool = True,
                  export_format: str = "mini_npz") -> GroupResult:
        """
        Predict depth for one turntable position.

        When the group carries extrinsics, this runs pose-conditioned and
        (with align_to_input_ext_scale) returns METRIC depth in the
        calibration frame. Otherwise the depth is scale-free.
        """
        import time
        kwargs = dict(process_res=process_res, export_format=export_format)
        if group.pose_conditioned:
            kwargs.update(extrinsics=group.extrinsics,
                          intrinsics=group.intrinsics,
                          align_to_input_ext_scale=align_to_input_ext_scale)
            if group.extrinsics.shape[1:] != (4, 4):
                raise ValueError(f"extrinsics must be (N,4,4), got "
                                 f"{group.extrinsics.shape} — the API rejects 3x4")

        t0 = time.time()
        pred = self.model.inference(group.images_rgb, **kwargs)
        seconds = time.time() - t0

        depth = np.asarray(pred.depth, np.float32)
        conf = np.asarray(pred.conf, np.float32)
        extr = np.asarray(pred.extrinsics, np.float64)
        intr = np.asarray(pred.intrinsics, np.float64)

        # Sanity: with pose conditioning the returned centers should equal
        # the ones we passed. Drift means the depth scale is NOT metric.
        dev_mm = 0.0
        metric = False
        if group.pose_conditioned:
            c_in = np.array([camera_center(E) for E in group.extrinsics])
            c_out = np.array([camera_center(E) for E in extr])
            dev_mm = float(np.linalg.norm(c_out - c_in, axis=1).max() * 1000.0)
            metric = align_to_input_ext_scale and dev_mm < 1.0

        return GroupResult(depth=depth, conf=conf, extrinsics=extr, intrinsics=intr,
                           pose_deviation_mm=dev_mm, metric=metric, seconds=seconds)


def points_from_result(result: GroupResult, index: int, *, calib: Calibration,
                       pose_source: str = "calibrated",
                       frames: list[Frame] | None = None,
                       conf_pct: float = 20.0,
                       colors_bgr: np.ndarray | None = None
                       ) -> tuple[np.ndarray, np.ndarray]:
    """
    One view's depth -> world-frame points (meters) with optional colors.

    With pose_source="calibrated" the calibrated pose places the points,
    which is the trustworthy option; with "none" the model's own pose is
    used and the result lives in its arbitrary frame.
    """
    from da3_calib import unproject

    depth = result.depth[index].astype(np.float64)
    valid = np.isfinite(depth) & (depth > 0)
    if conf_pct > 0 and valid.any():
        thresh = np.percentile(result.conf[index][valid], conf_pct)
        valid &= result.conf[index] >= thresh

    pts_cam = unproject(depth, result.intrinsics[index])
    if pose_source == "calibrated" and frames is not None:
        c2w = calib.c2w(frames[index].camera_key).copy()
        c2w[:3, 3] *= calib.scale_m                       # meters
    else:
        E = result.extrinsics[index]
        R, t = E[:3, :3], E[:3, 3]
        c2w = np.eye(4)
        c2w[:3, :3], c2w[:3, 3] = R.T, -R.T @ t
    world = pts_cam @ c2w[:3, :3].T + c2w[:3, 3]

    xyz = world[valid]
    if colors_bgr is not None:
        h, w = depth.shape
        rgb = cv2.resize(colors_bgr, (w, h), interpolation=cv2.INTER_AREA)[valid][:, ::-1]
    else:
        rgb = np.full((len(xyz), 3), 200, np.uint8)
    return xyz, rgb
