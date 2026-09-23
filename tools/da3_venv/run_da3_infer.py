#!/usr/bin/env python3
"""
run_da3_infer.py — flexible DA3 depth inference (the utility knife)
===================================================================
Target specific turntable positions and/or cameras, switch output types
on and off, and optionally drop the calibration entirely for debugging.

Every setting comes from da3_config.yaml; the flags below override it for
this invocation only. Omitted flags leave the config value untouched.

EXAMPLES
    # one position, everything else from the config
    run_da3_infer.py --scan ex2_006/pose-b --positions 0

    # three positions, cameras 2-4 only, with clouds, no depth PNGs
    run_da3_infer.py --scan ex2_006/pose-b --positions 0 90 180 \\
        --cameras 2 3 4 --cloud --no-depth-raw

    # unconditioned debug run (scale-free, model's own frame)
    run_da3_infer.py --scan ex2_006/pose-b --positions 0 \\
        --pose-source none --depth-undistorted --no-depth-raw \\
        --out-dir /tmp/da3_debug

    # what would run, without loading the model
    run_da3_infer.py --scan ex2_006/pose-b --positions 0 --dry-run
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from da3_venv import ensure_venv

import da3_frames as frames_mod
from da3_config import get_dotted, load_config, summary, validate
from da3_pipeline import process_scan, resolve_out_dir


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Flexible Depth Anything 3 inference for MOAD scans.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Flags override da3_config.yaml; anything omitted uses the config.")

    g = ap.add_argument_group("scan selection")
    g.add_argument("--scan", required=True,
                   help="scan/pose relative to paths.data_root (e.g. ex2_006/pose-b), "
                        "or an absolute path")
    g.add_argument("--positions", nargs="+", type=int, default=None,
                   help="turntable positions in degrees; default all")
    g.add_argument("--cameras", nargs="+", type=int, default=None,
                   help="camera ids (1-5); default all. Fewer cameras means a "
                        "weaker multi-view constraint for the model")

    g = ap.add_argument_group("config / paths")
    g.add_argument("--config", default=None, help="config file (yaml or json)")
    g.add_argument("--data-root", default=None)
    g.add_argument("--calib-dir", default=None)
    g.add_argument("--out-dir", default=None,
                   help="absolute output dir; default <scan>/<output.subdir>")

    g = ap.add_argument_group("model / inference")
    g.add_argument("--model", default=None, help="HuggingFace model id")
    g.add_argument("--process-res", type=int, default=None,
                   help="model working resolution (long side)")
    g.add_argument("--pose-source", choices=["calibrated", "none"], default=None,
                   help="'none' ignores the calibration: output is scale-free "
                        "and in the model's own frame (debug only)")
    g.add_argument("--images-subdir", default=None,
                   help="which images_N folder to read (does NOT change the "
                        "model's output resolution)")

    g = ap.add_argument_group("outputs")
    g.add_argument("--depth-raw", dest="depth_raw", action="store_true", default=None,
                   help="depth in raw (distorted) DSLR geometry")
    g.add_argument("--no-depth-raw", dest="depth_raw", action="store_false")
    g.add_argument("--depth-undistorted", dest="depth_undist", action="store_true",
                   default=None, help="depth as the model produced it")
    g.add_argument("--no-depth-undistorted", dest="depth_undist", action="store_false")
    g.add_argument("--cloud", dest="cloud", action="store_true", default=None,
                   help="colored PLY per position")
    g.add_argument("--no-cloud", dest="cloud", action="store_false")
    g.add_argument("--save-npz", dest="save_npz", action="store_true", default=None)
    g.add_argument("--preview-every", type=int, default=None,
                   help="preview every N positions (0 = off)")

    g = ap.add_argument_group("runtime")
    g.add_argument("--skip-existing", dest="skip_existing", action="store_true",
                   default=None)
    g.add_argument("--no-strict-meta", dest="strict_meta", action="store_false",
                   default=None, help="allow writing into a directory whose "
                                      "meta.json disagrees (not recommended)")
    g.add_argument("--dry-run", action="store_true",
                   help="resolve config and frames, then stop before the model")
    return ap


def resolve_scan_dir(cfg: dict, scan: str) -> Path:
    p = Path(scan).expanduser()
    if p.is_absolute():
        return p
    return Path(str(get_dotted(cfg, "paths.data_root"))).expanduser() / scan


def main() -> int:
    args = build_parser().parse_args()

    cfg = load_config(args.config, overrides={
        "paths.data_root": args.data_root,
        "paths.calib_dir": args.calib_dir,
        "model.name": args.model,
        "inference.process_res": args.process_res,
        "inference.pose_source": args.pose_source,
        "input.images_subdir": args.images_subdir,
        "output.depth.raw": args.depth_raw,
        "output.depth.undistorted": args.depth_undist,
        "output.cloud.enabled": args.cloud,
        "output.save_npz": args.save_npz,
        "output.preview_every": args.preview_every,
        "runtime.skip_existing": args.skip_existing,
        "runtime.strict_meta": args.strict_meta,
    })
    try:
        validate(cfg)
    except ValueError as e:
        print(f"[ERROR] config: {e}", file=sys.stderr)
        return 2

    scan_dir = resolve_scan_dir(cfg, args.scan)
    if not scan_dir.is_dir():
        print(f"[ERROR] scan not found: {scan_dir}", file=sys.stderr)
        return 2

    try:
        all_frames = frames_mod.load_frames(scan_dir)
    except (FileNotFoundError, ValueError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 2
    for w in frames_mod.check_consistency(all_frames):
        print(f"  [WARN] {w}")

    frames = frames_mod.select(all_frames, args.positions, args.cameras)
    if not frames:
        print(f"[ERROR] no frames match positions={args.positions} "
              f"cameras={args.cameras}", file=sys.stderr)
        return 2

    print(f"  scan        : {scan_dir}")
    print(summary(cfg))
    print(f"  selection   : {frames_mod.describe(frames)}")

    if args.dry_run:
        print(f"  output      : {resolve_out_dir(cfg, scan_dir, args.out_dir)}")
        print("  dry run — stopping before model load")
        return 0

    ensure_venv()
    try:
        process_scan(cfg, scan_dir, frames,
                     out_dir=Path(args.out_dir).expanduser() if args.out_dir else None)
    except (FileNotFoundError, KeyError, ValueError) as e:
        # Missing images, a camera absent from the calibration, mismatched
        # image sizes: report plainly rather than with a traceback.
        print(f"[ERROR] {type(e).__name__}: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
