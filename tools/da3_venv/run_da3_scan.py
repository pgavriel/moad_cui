#!/usr/bin/env python3
"""
run_da3_scan.py — generate DA3 depth for an entire scan
=======================================================
The everyday entry point: point it at a scan/pose folder and it produces
depth for every frame using the settings in da3_config.yaml.

Shares all logic with run_da3_infer.py through da3_pipeline.process_scan
(it does NOT shell out to that script), so the model loads once and the
run produces a single aggregate meta.json.

EXAMPLES
    run_da3_scan.py --scan ex2_006/pose-b
    run_da3_scan.py --scan ex2_006/pose-b --resume
    run_da3_scan.py --scan ex2_006/pose-b --model depth-anything/da3-large \\
        --process-res 756

For anything more selective — specific positions or cameras, clouds,
unconditioned debug runs — use run_da3_infer.py.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from da3_venv import ensure_venv

import da3_frames as frames_mod
from da3_config import get_dotted, load_config, summary, validate
from da3_pipeline import process_scan, resolve_out_dir


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run Depth Anything 3 over a full MOAD scan.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Settings come from da3_config.yaml; see run_da3_infer.py "
               "for finer-grained control.")
    ap.add_argument("--scan", required=True,
                    help="scan/pose relative to paths.data_root "
                         "(e.g. ex2_006/pose-b), or an absolute path")
    ap.add_argument("--config", default=None, help="config file (yaml or json)")
    ap.add_argument("--model", default=None, help="override model.name")
    ap.add_argument("--process-res", type=int, default=None,
                    help="override inference.process_res")
    ap.add_argument("--out-dir", default=None,
                    help="absolute output dir; default <scan>/<output.subdir>")
    ap.add_argument("--resume", action="store_true",
                    help="skip positions whose outputs already exist")
    ap.add_argument("--dry-run", action="store_true",
                    help="show what would run, without loading the model")
    args = ap.parse_args()

    cfg = load_config(args.config, overrides={
        "model.name": args.model,
        "inference.process_res": args.process_res,
        "runtime.skip_existing": True if args.resume else None,
    })
    try:
        validate(cfg)
    except ValueError as e:
        print(f"[ERROR] config: {e}", file=sys.stderr)
        return 2

    scan = Path(args.scan).expanduser()
    scan_dir = scan if scan.is_absolute() else \
        Path(str(get_dotted(cfg, "paths.data_root"))).expanduser() / args.scan
    if not scan_dir.is_dir():
        print(f"[ERROR] scan not found: {scan_dir}", file=sys.stderr)
        return 2

    try:
        frames = frames_mod.load_frames(scan_dir)
    except (FileNotFoundError, ValueError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 2
    warnings = frames_mod.check_consistency(frames)
    for w in warnings:
        print(f"  [WARN] {w}")

    print(f"  scan        : {scan_dir}")
    print(summary(cfg))
    print(f"  frames      : {frames_mod.describe(frames)}")

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
