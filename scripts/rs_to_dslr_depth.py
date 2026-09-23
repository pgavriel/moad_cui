#!/usr/bin/env python3
"""
rs_to_dslr_depth.py
-------------------
Reproject RealSense depth into every DSLR frame of a MOAD scan and write
per-frame depth maps aligned to images_N/ (raw, distorted DSLR geometry).

FRAME MAPPING
    Driven by <scan>/transforms.json: each frame's file_path
    (images/frame_00001.jpg), camera_id (1..5) and position_deg (0..355)
    select DSLR camN and the RealSense captures rsK_{pos:03d}_depth.png.
    Cameras are physically fixed, so the PHYSICAL poses in the joint
    calibration are used directly — the turntable rotation is already
    "baked into" the RS data captured at that same position.

GEOMETRY (joint calibration, OpenCV convention, poses in model units)
    RS depth (uint16 * depth_unit m) -> undistort RS px -> ray * z
    -> m / scale (model units) -> RS c2w -> DSLR w2c
    -> SIMPLE_RADIAL projection at images_N resolution
    -> robust z-buffer merge -> meters

ROBUST MERGE
    Per output pixel: take the nearest sample z0, then the MEDIAN of all
    samples within z0 + merge_tol. Keeps occlusion handling (background
    is rejected) but removes the near-bias of a plain "nearest wins"
    minimum over several noisy RS views.

OUTPUTS
    <scan>/DSLR_depth/frame_00001.png ...  uint16 millimeters, z-depth, 0 = invalid
    <scan>/DSLR_depth/meta.json
    optional overlays (--overlay-every N) in <scan>/DSLR_depth/overlays/

USAGE
    python3 rs_to_dslr_depth.py                         # full scan, rs2+rs3+rs4
    python3 rs_to_dslr_depth.py --positions 0 90 180    # subset (degrees)
    python3 rs_to_dslr_depth.py --rs rs1 rs2 rs3 rs4 rs5 --out-dir .../DSLR_depth_all5
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DEFAULT_SCAN  = "/home/csrobot/MOAD_DATA/ex2_006/pose-b"
DEFAULT_CALIB = "/home/csrobot/moad_control/moad_cui/calibration/55mm_joint"


# ---------------------------------------------------------------------------
# Calibration / camera model
# ---------------------------------------------------------------------------

def load_calib(path: Path) -> dict:
    with open(path) as f:
        c = json.load(f)
    if c["intrinsics"]["camera_model"] != "SIMPLE_RADIAL":
        raise ValueError(f"{path}: only SIMPLE_RADIAL supported")
    return c


def scaled_intrinsics(intr: dict, out_w: int, out_h: int) -> tuple:
    """DSLR intrinsics for an image resized to out_w x out_h (pixel-center aware)."""
    sx, sy = out_w / intr["width"], out_h / intr["height"]
    f  = intr["fx"] * sx
    cx = (intr["cx"] + 0.5) * sx - 0.5
    cy = (intr["cy"] + 0.5) * sy - 0.5
    return f, cx, cy, intr["distortion"]["k1"], out_w, out_h


def undistort_normalized(xd, yd, k1, iters=10):
    """Invert x_d = x * (1 + k1 r^2) (COLMAP SIMPLE_RADIAL) by fixed-point iteration."""
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        d = 1.0 + k1 * (x * x + y * y)
        x, y = xd / d, yd / d
    return x, y


def project(Xc, f, cx, cy, k1):
    x = Xc[:, 0] / Xc[:, 2]
    y = Xc[:, 1] / Xc[:, 2]
    d = 1.0 + k1 * (x * x + y * y)
    return f * x * d + cx, f * y * d + cy


# ---------------------------------------------------------------------------
# RealSense depth -> world points (model units)
# ---------------------------------------------------------------------------

class RSUnprojector:
    """Precomputes undistorted RS rays once (shared intrinsics for all RS)."""

    def __init__(self, rs_intr: dict):
        self.w, self.h = rs_intr["width"], rs_intr["height"]
        v, u = np.mgrid[0:self.h, 0:self.w].astype(np.float64)
        xn, yn = undistort_normalized((u - rs_intr["cx"]) / rs_intr["fx"],
                                      (v - rs_intr["cy"]) / rs_intr["fy"],
                                      rs_intr["distortion"]["k1"])
        self.rays = np.stack([xn, yn, np.ones_like(xn)], -1)   # (h, w, 3), z = 1

    def __call__(self, depth_raw, c2w, depth_unit, scale, edge_thresh, zmin, zmax):
        if depth_raw.shape != (self.h, self.w):
            raise ValueError(f"RS depth {depth_raw.shape[::-1]} != calib {self.w}x{self.h}")
        z_m = depth_raw.astype(np.float32) * depth_unit
        valid = (z_m > zmin) & (z_m < zmax)
        if edge_thresh > 0:                       # flying-pixel rejection
            k = np.ones((3, 3), np.uint8)
            zhi = cv2.dilate(np.where(valid, z_m, 0).astype(np.float32), k)
            zlo = cv2.erode(np.where(valid, z_m, 1e6).astype(np.float32), k)
            valid &= (zhi - zlo) < edge_thresh * z_m
        z = z_m[valid].astype(np.float64) / scale            # -> model units
        Xc = self.rays[valid] * z[:, None]
        return Xc @ c2w[:3, :3].T + c2w[:3, 3]


# ---------------------------------------------------------------------------
# World points -> DSLR depth with robust merge
# ---------------------------------------------------------------------------

def splat_offsets(radius: int) -> list[tuple[int, int]]:
    return [(du, dv) for dv in range(-radius, radius + 1)
            for du in range(-radius, radius + 1)
            if du * du + dv * dv <= radius * radius + radius]


def render_depth(Xw, w2c, cam, scale, offsets, merge_tol):
    f, cx, cy, k1, W, H = cam
    Xc = Xw @ w2c[:3, :3].T + w2c[:3, 3]
    Xc = Xc[Xc[:, 2] > 1e-6]
    u, v = project(Xc, f, cx, cy, k1)
    z = (Xc[:, 2] * scale).astype(np.float32)             # -> meters
    ui = np.round(u).astype(np.int64)
    vi = np.round(v).astype(np.int64)
    r = max(max(abs(a), abs(b)) for a, b in offsets)
    keep = (ui >= -r) & (ui < W + r) & (vi >= -r) & (vi < H + r)
    ui, vi, z = ui[keep], vi[keep], z[keep]

    idx_l, z_l = [], []
    for du, dv in offsets:
        uu, vv = ui + du, vi + dv
        ok = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
        idx_l.append(vv[ok] * W + uu[ok])
        z_l.append(z[ok])
    idx = np.concatenate(idx_l)
    zz = np.concatenate(z_l)
    depth = np.zeros(H * W, np.float32)
    if idx.size == 0:
        return depth.reshape(H, W)

    # Sort by pixel, then depth ascending
    order = np.lexsort((zz, idx))
    idx_s, z_s = idx[order], zz[order]
    is_start = np.empty(idx_s.size, bool)
    is_start[0] = True
    is_start[1:] = idx_s[1:] != idx_s[:-1]
    starts = np.flatnonzero(is_start)
    group_id = np.cumsum(is_start) - 1
    z0 = z_s[starts][group_id]                             # nearest sample per pixel

    # Samples within tolerance of the nearest are a contiguous prefix of each
    # (ascending) group, so the median is directly indexable.
    near = z_s <= z0 + merge_tol
    k = np.bincount(group_id, weights=near).astype(np.int64)
    lo = starts + (k - 1) // 2
    hi = starts + k // 2
    depth[idx_s[starts]] = 0.5 * (z_s[lo] + z_s[hi])
    return depth.reshape(H, W)


# ---------------------------------------------------------------------------
# Overlay (debug)
# ---------------------------------------------------------------------------

def make_overlay(rgb, depth_m, label, lo, hi):
    valid = depth_m > 0
    norm = np.clip((depth_m - lo) / max(hi - lo, 1e-9), 0, 1)
    cmap = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    cmap[~valid] = (40, 40, 40)
    blend = rgb.copy()
    blend[valid] = (0.45 * rgb[valid] + 0.55 * cmap[valid]).astype(np.uint8)
    out = np.hstack([rgb, cmap, blend])
    cv2.rectangle(out, (0, 0), (18 + 11 * len(label), 34), (0, 0, 0), -1)
    cv2.putText(out, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="RealSense -> DSLR depth for a full scan.")
    ap.add_argument("--scan-dir", default=DEFAULT_SCAN)
    ap.add_argument("--calib-dir", default=DEFAULT_CALIB)
    ap.add_argument("--images-subdir", default="images_4",
                    help="DSLR image folder the depth is aligned to (sets resolution)")
    ap.add_argument("--rs", nargs="+", default=["rs2", "rs3", "rs4"],
                    help="RS cameras merged into EVERY DSLR view")
    ap.add_argument("--positions", nargs="+", type=int, default=None,
                    help="only these turntable positions (deg); default all")
    ap.add_argument("--splat-radius", type=int, default=None,
                    help="default: ~half the RS pixel footprint at output res")
    ap.add_argument("--merge-tol", type=float, default=0.01,
                    help="median-merge window above nearest sample, m (default 0.01)")
    ap.add_argument("--depth-unit", type=float, default=0.001,
                    help="meters per RS depth count (default 0.001 = mm)")
    ap.add_argument("--edge-thresh", type=float, default=0.02,
                    help="relative 3x3 depth range for flying-pixel rejection (0=off)")
    ap.add_argument("--zmin", type=float, default=0.15, help="RS min depth, m")
    ap.add_argument("--zmax", type=float, default=2.0, help="RS max depth, m")
    ap.add_argument("--out-dir", default=None, help="default: <scan>/DSLR_depth")
    ap.add_argument("--overlay-every", type=int, default=45,
                    help="write a debug overlay every N frames (0 = none)")
    args = ap.parse_args()

    scan  = Path(args.scan_dir).expanduser()
    calib = Path(args.calib_dir).expanduser()
    out   = Path(args.out_dir).expanduser() if args.out_dir else scan / "DSLR_depth"
    img_dir = scan / args.images_subdir
    out.mkdir(parents=True, exist_ok=True)

    dcal = load_calib(calib / "cam_parameters.json")
    rcal = load_calib(calib / "realsense_cam_parameters.json")
    scale = dcal["_info"]["scaling"]["scale"]
    if abs(rcal["_info"]["scaling"]["scale"] - scale) > 1e-12:
        print("[ERROR] DSLR and RS calibration scale factors differ"); sys.exit(1)
    missing_rs = [r for r in args.rs if r not in rcal["cameras"]]
    if missing_rs:
        print(f"[ERROR] RS cameras not in calibration: {missing_rs}"); sys.exit(1)

    # ── Frame list from transforms.json ──────────────────────────────────
    with open(scan / "transforms.json") as f:
        frames = json.load(f)["frames"]
    n_cams = len({fr["camera_id"] for fr in frames})
    per_cam = len(frames) // n_cams
    mism = sum(1 for i, fr in enumerate(frames)
               if (fr["camera_id"], fr["position_deg"])
               != (i // per_cam + 1, (i % per_cam) * (360 // per_cam)))
    print(f"  transforms.json: {len(frames)} frames, {n_cams} cameras, "
          f"{per_cam}/camera  (camera-major formula mismatches: {mism})")

    jobs = {}                                   # position -> [(frame_name, cam_key)]
    for fr in frames:
        pos = int(fr["position_deg"])
        if args.positions is not None and pos not in args.positions:
            continue
        jobs.setdefault(pos, []).append(
            (Path(fr["file_path"]).name, f"cam{fr['camera_id']}"))
    if not jobs:
        print("[ERROR] no frames selected"); sys.exit(1)

    # ── Output resolution from images_N ──────────────────────────────────
    probe_name = next(iter(jobs.values()))[0][0]
    probe = cv2.imread(str(img_dir / probe_name))
    if probe is None:
        print(f"[ERROR] cannot read {img_dir / probe_name}"); sys.exit(1)
    H, W = probe.shape[:2]
    cam_model = scaled_intrinsics(dcal["intrinsics"], W, H)
    rs_footprint = cam_model[0] / rcal["intrinsics"]["fx"]
    radius = args.splat_radius if args.splat_radius is not None \
        else max(1, int(np.ceil(rs_footprint / 2)))
    offsets = splat_offsets(radius)
    print(f"  output {W}x{H} ({args.images_subdir})  RS px ≈ {rs_footprint:.2f} out px  "
          f"splat r={radius}  merge_tol={args.merge_tol*1000:.0f} mm  RS: {'+'.join(args.rs)}")

    unproj = RSUnprojector(rcal["intrinsics"])
    rs_c2w = {r: np.array(rcal["cameras"][r]["extrinsics"]["c2w"]) for r in args.rs}
    dslr_w2c = {k: np.array(v["extrinsics"]["w2c"]) for k, v in dcal["cameras"].items()}

    stats, t0 = {}, time.time()
    positions = sorted(jobs)
    for pi, pos in enumerate(positions):
        pts = []
        for r in args.rs:
            p = scan / "realsense" / f"{r}_{pos:03d}_depth.png"
            d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
            if d is None:
                print(f"  [WARN] missing {p.name}")
                continue
            pts.append(unproj(d, rs_c2w[r], args.depth_unit, scale,
                              args.edge_thresh, args.zmin, args.zmax))
        if not pts:
            print(f"  [WARN] position {pos:03d}: no RS data — frames skipped")
            continue
        Xw = np.concatenate(pts)

        for name, cam in jobs[pos]:
            depth = render_depth(Xw, dslr_w2c[cam], cam_model, scale,
                                 offsets, args.merge_tol)
            png = np.clip(np.round(depth * 1000), 0, 65535).astype(np.uint16)
            cv2.imwrite(str(out / (Path(name).stem + ".png")), png)
            v = depth[depth > 0]
            stats[name] = {"cam": cam, "pos": pos,
                           "valid": float((depth > 0).mean()),
                           "median_m": float(np.median(v)) if v.size else 0.0}

            if args.overlay_every and (len(stats) - 1) % args.overlay_every == 0:
                rgb = cv2.imread(str(img_dir / name))
                if rgb is not None and v.size:
                    lo, hi = np.percentile(v, [2, 98])
                    (out / "overlays").mkdir(exist_ok=True)
                    cv2.imwrite(str(out / "overlays" / f"{Path(name).stem}.jpg"),
                                make_overlay(rgb, depth,
                                             f"{name} {cam}_{pos:03d} <- {'+'.join(args.rs)}",
                                             lo, hi))

        el = time.time() - t0
        eta = el / (pi + 1) * (len(positions) - pi - 1)
        print(f"  pos {pos:03d}  [{pi+1}/{len(positions)}]  "
              f"{len(Xw)/1e6:.2f}M pts  {el:.0f}s elapsed, ~{eta:.0f}s left")

    # ── Summary + metadata ───────────────────────────────────────────────
    if stats:
        vf = np.array([s["valid"] for s in stats.values()])
        worst = min(stats, key=lambda k: stats[k]["valid"])
        print(f"\n  {len(stats)} depth maps written to {out}")
        print(f"  valid coverage: mean {vf.mean()*100:.1f}%  "
              f"min {vf.min()*100:.1f}% ({worst})")
    meta = {
        "scan": str(scan), "calib": str(calib), "images_subdir": args.images_subdir,
        "format": "uint16 PNG, millimeters, z-depth along DSLR optical axis, 0 = invalid",
        "geometry": "raw (distorted) DSLR, pixel-aligned with images_subdir",
        "output_size": [W, H],
        "intrinsics_out": {"model": "SIMPLE_RADIAL", "f": cam_model[0],
                           "cx": cam_model[1], "cy": cam_model[2], "k1": cam_model[3]},
        "rs_sources": args.rs, "splat_radius": radius, "merge_tol_m": args.merge_tol,
        "edge_thresh": args.edge_thresh, "depth_unit_in": args.depth_unit,
        "frames": stats,
    }
    with open(out / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(f"  ✓ meta.json written")


if __name__ == "__main__":
    main()
