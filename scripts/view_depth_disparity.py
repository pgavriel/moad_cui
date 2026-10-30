#!/usr/bin/env python3
"""
compare_depth_da3_rs.py
-----------------------
Compare Depth Anything 3 depth against RealSense-derived depth, frame by frame.

Both folders hold uint16 PNGs in millimetres (0 = invalid), named
frame_XXXX.png. For each matching pair:

    diff = DA3 - RS   (mm)
    red   -> DA3 farther than RS
    blue  -> DA3 nearer than RS
    white -> same depth
    grey  -> invalid in either map

The colour scale (--vmin/--vmax, mm) is fixed for the whole scan. All frames
are written to one video with a colour bar, and scan-wide statistics are
printed at the end.

USAGE
    python3 compare_depth_da3_rs.py
    python3 compare_depth_da3_rs.py --scan /path/to/scan --vmin -30 --vmax 30
"""

import argparse
import re
import sys
from pathlib import Path

import cv2
import numpy as np

NAME_RE = re.compile(r"^frame_\d{4,}\.png$")
BAR_W = 110
HEADER_H = 40
INVALID_BGR = (50, 50, 50)


def list_frames(folder: Path) -> list[str]:
    return sorted(p.name for p in folder.iterdir() if NAME_RE.match(p.name))


def read_depth(path: Path) -> np.ndarray:
    d = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if d is None:
        sys.exit(f"ERROR: could not read {path}")
    if d.dtype != np.uint16:
        sys.exit(f"ERROR: {path} is {d.dtype}, expected uint16 mm")
    return d


def bwr(t: np.ndarray) -> np.ndarray:
    """t in [0,1] (0.5 = white) -> BGR uint8. Blue -> white -> red."""
    t = np.clip(t, 0.0, 1.0)
    lo = np.clip(t * 2.0, 0, 1)            # 0..1 over the blue half
    hi = np.clip(t * 2.0 - 1.0, 0, 1)      # 0..1 over the red half
    r = np.where(t < 0.5, lo, 1.0)
    g = np.where(t < 0.5, lo, 1.0 - hi)
    b = np.where(t < 0.5, 1.0, 1.0 - hi)
    return (np.stack([b, g, r], axis=-1) * 255).astype(np.uint8)


def diff_to_t(diff: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
    """Map mm to [0,1] with 0 mm pinned to 0.5 (white), even if |vmin| != vmax."""
    t = np.empty_like(diff, dtype=np.float32)
    neg = diff < 0
    t[neg] = 0.5 - 0.5 * (diff[neg] / vmin)
    t[~neg] = 0.5 + 0.5 * (diff[~neg] / vmax)
    return t


def colour_bar(height: int, vmin: float, vmax: float) -> np.ndarray:
    bar = np.full((height, BAR_W, 3), 255, np.uint8)
    top, bot = HEADER_H + 10, height - 10

    # Asymmetric color bar
    vals = np.linspace(vmax, vmin, bot - top, dtype=np.float32)
    strip = bwr(diff_to_t(vals, vmin, vmax))[:, None, :]
    # Force color bar symmetry 
    # t = np.linspace(1.0, 0.0, bot - top, dtype=np.float32)   # top = vmax, bottom = vmin
    # strip = bwr(t)[:, None, :]

    bar[top:bot, 10:35] = np.repeat(strip, 25, axis=1)
    cv2.rectangle(bar, (10, top), (34, bot - 1), (0, 0, 0), 1)
    for v in (vmax, vmax / 2, 0.0, vmin / 2, vmin):
        # place ticks by the same piecewise mapping used for colours
        tt = float(diff_to_t(np.array([v], np.float32), vmin, vmax)[0])

        # Asymmetric color bar
        y = int(bot - 1 - (v - vmin) / (vmax - vmin) * (bot - top - 1))
        # Force color bar symmetry 
        # y = int(round(bot - 1 - tt * (bot - top - 1)))

        cv2.line(bar, (35, y), (42, y), (0, 0, 0), 1)
        cv2.putText(bar, f"{v:+.0f}", (45, y + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bar, "DA3-RS", (8, HEADER_H - 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(bar, "(mm)", (8, HEADER_H - 4), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 0, 0), 1, cv2.LINE_AA)
    return bar


def main() -> None:
    ap = argparse.ArgumentParser(description="DA3 vs RealSense depth disparity video.")
    ap.add_argument("--scan", default="/home/csrobot/MOAD_DATA/ex2_006/pose-b")
    ap.add_argument("--da3", default="DSLR_depth", help="DA3 depth folder (relative to scan)")
    ap.add_argument("--rs", default="DSLR_depth_rs", help="RealSense depth folder (relative to scan)")
    ap.add_argument("--vmin", type=float, default=-50.0, help="colour scale min, mm (blue)")
    ap.add_argument("--vmax", type=float, default=100.0, help="colour scale max, mm (red)")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--out", default=None,
                    help="output video (default: <scan>/output/depth_diff_da3_vs_rs.mp4)")
    args = ap.parse_args()

    if not (args.vmin < 0 < args.vmax):
        sys.exit("ERROR: need vmin < 0 < vmax so that 0 mm maps to white")

    scan = Path(args.scan).expanduser()
    da3_dir, rs_dir = scan / args.da3, scan / args.rs
    for d in (scan, da3_dir, rs_dir):
        if not d.is_dir():
            sys.exit(f"ERROR: folder not found: {d}")

    da3_names, rs_names = list_frames(da3_dir), list_frames(rs_dir)
    if len(da3_names) != len(rs_names):
        sys.exit(f"ERROR: {len(da3_names)} frames in {da3_dir.name} vs "
                 f"{len(rs_names)} in {rs_dir.name}")
    if not da3_names:
        sys.exit(f"ERROR: no frame_XXXX.png files in {da3_dir}")
    if da3_names != rs_names:
        only = sorted(set(da3_names) ^ set(rs_names))
        sys.exit(f"ERROR: frame names differ, e.g. {only[:5]}")
    print(f"{len(da3_names)} frame pairs   scale [{args.vmin:+.0f}, {args.vmax:+.0f}] mm")

    out = Path(args.out).expanduser() if args.out else scan / "output" / "depth_diff_da3_vs_rs.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)

    # scan-wide accumulators
    edges = np.arange(-2000.0, 2000.1, 0.1)
    hist = np.zeros(len(edges) - 1, np.int64)
    n_valid = n_total = n_clipped = 0
    s1 = s2 = sabs = 0.0
    writer = None

    for i, name in enumerate(da3_names):
        da3 = read_depth(da3_dir / name)
        rs = read_depth(rs_dir / name)
        if da3.shape != rs.shape:
            sys.exit(f"ERROR: {name}: shape {da3.shape} vs {rs.shape}")

        valid = (da3 > 0) & (rs > 0)
        diff = da3.astype(np.float32) - rs.astype(np.float32)
        d = diff[valid].astype(np.float64)

        n_total += valid.size
        n_valid += d.size
        s1 += d.sum()
        s2 += (d * d).sum()
        sabs += np.abs(d).sum()
        n_clipped += int(((d < -2000) | (d >= 2000)).sum())
        hist += np.histogram(np.clip(d, -2000, 2000 - 1e-6), edges)[0]

        img = bwr(diff_to_t(diff, args.vmin, args.vmax))
        img[~valid] = INVALID_BGR

        h, w = img.shape[:2]
        header = np.full((HEADER_H, w, 3), 255, np.uint8)
        txt = (f"{name}   valid {d.size / valid.size * 100:5.1f}%   "
               f"median {np.median(d):+.1f} mm   mean|d| {np.abs(d).mean():.1f} mm"
               if d.size else f"{name}   no overlapping valid pixels")
        (tw, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)
        fs = min(0.7, 0.7 * (w - 20) / tw)     # shrink to fit narrow frames
        cv2.putText(header, txt, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, fs,
                    (0, 0, 0), 1, cv2.LINE_AA)
        frame = np.vstack([header, img])
        frame = np.hstack([frame, colour_bar(frame.shape[0], args.vmin, args.vmax)])

        if writer is None:
            fh, fw = frame.shape[:2]
            writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"),
                                     args.fps, (fw, fh))
            if not writer.isOpened():
                sys.exit(f"ERROR: could not open video writer for {out}")
        if frame.shape[:2] != (fh, fw):
            frame = cv2.resize(frame, (fw, fh), interpolation=cv2.INTER_NEAREST)
        writer.write(frame)
        print(f"\r  {i + 1}/{len(da3_names)}", end="", flush=True)

    writer.release()
    print(f"\nvideo: {out}")

    # ── scan-wide statistics ────────────────────────────────────────────
    print("\n=== DA3 - RS depth disparity (whole scan) ===")
    print(f"  overlap coverage : {n_valid / n_total * 100:.1f}% of pixels ({n_valid:,})")
    if n_valid == 0:
        return
    mean = s1 / n_valid
    std = np.sqrt(max(s2 / n_valid - mean ** 2, 0.0))
    cdf = np.cumsum(hist) / hist.sum()
    centres = (edges[:-1] + edges[1:]) / 2

    def pct(p):
        return centres[np.searchsorted(cdf, p / 100.0)]

    def within(t):
        lo, hi = np.searchsorted(edges, -t), np.searchsorted(edges, t)
        return hist[lo:hi].sum() / n_valid * 100

    print(f"  mean (bias)      : {mean:+.2f} mm   (+ = DA3 farther)")
    print(f"  median           : {pct(50):+.2f} mm")
    print(f"  std              : {std:.2f} mm")
    print(f"  mean |diff|      : {sabs / n_valid:.2f} mm")
    print(f"  RMSE             : {np.sqrt(s2 / n_valid):.2f} mm")
    print("  percentiles      : " + "  ".join(f"p{p}={pct(p):+.1f}" for p in (5, 25, 75, 95)))
    print("  within           : " + "  ".join(f"±{t}mm {within(t):.1f}%" for t in (5, 10, 20)))
    if n_clipped:
        print(f"  note: {n_clipped:,} pixels beyond ±2000 mm were clipped for percentiles")


if __name__ == "__main__":
    main()
