#!/usr/bin/env python3
"""
da3_pipeline.py
---------------
The layer both run_* scripts share: take a resolved config plus a set of
frames, run inference group by group, and write every configured output.

Neither CLI calls the other — they both call process_scan() here, so the
model is loaded exactly once per invocation and the aggregate meta.json,
ETA and pose-deviation summary are produced in one place.
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

import da3_io as dio
from da3_calib import Calibration, redistort_depth
from da3_config import config_hash, get_dotted, output_subdir, settings_fingerprint
from da3_frames import Frame, group_by_position
from da3_inference import DA3Runner, points_from_result, prepare_group

BACKEND = "da3"


# ---------------------------------------------------------------------------
# Output-directory plumbing
# ---------------------------------------------------------------------------

def resolve_out_dir(cfg: dict, scan_dir: Path, explicit: str | None = None) -> Path:
    return Path(explicit).expanduser() if explicit else scan_dir / output_subdir(cfg)


def depth_subdir(out_dir: Path, kind: str, both: bool) -> Path:
    """`raw` and `undistorted` share the dir unless both are enabled."""
    return (out_dir / kind) if both else out_dir


def enabled_depth_kinds(cfg: dict) -> list[str]:
    return [k for k in ("raw", "undistorted") if get_dotted(cfg, f"output.depth.{k}")]


def build_meta(cfg: dict, scan_dir: Path, calib: Calibration | None,
               output_size: tuple[int, int]) -> dict:
    """Meta for this run; `geometry` names the primary depth product."""
    kinds = enabled_depth_kinds(cfg)
    pose_source = get_dotted(cfg, "inference.pose_source")
    return dio.build_meta(
        backend=BACKEND,
        scan=scan_dir,
        calibration=calib.path if calib else "",
        geometry=kinds[0] if kinds else "undistorted",
        metric=(pose_source == "calibrated"),
        pose_source=pose_source,
        output_size=output_size,
        settings=settings_fingerprint(cfg),
        config_hash=config_hash(cfg),
        model=get_dotted(cfg, "model.name"),
    )


def outputs_exist(out_dir: Path, cfg: dict, frames: list[Frame]) -> bool:
    """True when every configured depth product already exists for `frames`."""
    kinds = enabled_depth_kinds(cfg)
    if not kinds:
        return False
    both = len(kinds) > 1
    return all((depth_subdir(out_dir, k, both) / f"{fr.stem}.png").is_file()
               for k in kinds for fr in frames)


# ---------------------------------------------------------------------------
# Per-group output writing
# ---------------------------------------------------------------------------

def write_group_outputs(result, group, cfg: dict, out_dir: Path,
                        calib: Calibration | None, *, write_preview: bool
                        ) -> dict[str, dict]:
    """
    Write depth PNGs (and optionally cloud/preview) for one group.

    Returns per-frame stats keyed by frame stem, for meta.json.
    """
    kinds = enabled_depth_kinds(cfg)
    both = len(kinds) > 1
    upsample = bool(get_dotted(cfg, "output.depth.upsample_to_images"))
    stats: dict[str, dict] = {}
    img_w, img_h = group.image_size

    for i, fr in enumerate(group.frames):
        depth_m = result.depth[i].astype(np.float64)
        depth_m = np.where(np.isfinite(depth_m) & (depth_m > 0), depth_m, 0.0)

        for kind in kinds:
            out = depth_subdir(out_dir, kind, both)
            if kind == "undistorted":
                dio.write_depth_png(out / f"{fr.stem}.png", depth_m)
            else:
                # Raw geometry: resample at the prediction's own resolution
                # (depth values are unchanged by distortion), then optionally
                # upsample so the map is pixel-aligned with images_N.
                if calib is None:
                    raise ValueError("raw geometry requires a calibration")
                raw = redistort_depth(depth_m.astype(np.float32),
                                      result.intrinsics[i], calib.k1)
                if upsample and (img_w, img_h) != raw.shape[::-1]:
                    raw = cv2.resize(raw, (img_w, img_h),
                                     interpolation=cv2.INTER_NEAREST)
                dio.write_depth_png(out / f"{fr.stem}.png", raw)

        stats[fr.stem] = {
            "camera": fr.camera_key,
            "position_deg": fr.position_deg,
            "conf_mean": float(result.conf[i].mean()),
            **dio.depth_stats(depth_m),
        }

    if write_preview and get_dotted(cfg, "output.preview_every"):
        fr = group.frames[0]
        dio.write_preview(
            out_dir / "previews" / f"{fr.stem}.jpg",
            cv2.cvtColor(group.images_rgb[0], cv2.COLOR_RGB2BGR),
            np.where(np.isfinite(result.depth[0]), result.depth[0], 0.0),
            f"{fr.stem} {fr.camera_key} pos{group.position_deg:03d}",
            conf=result.conf[0])

    if get_dotted(cfg, "output.cloud.enabled"):
        _write_cloud(result, group, cfg, out_dir, calib)

    if get_dotted(cfg, "output.save_npz"):
        np.savez_compressed(
            out_dir / "npz" / f"pred_pos{group.position_deg:03d}.npz",
            depth=result.depth, conf=result.conf, extrinsics=result.extrinsics,
            intrinsics=result.intrinsics, metric=result.metric,
            cams=[f.camera_key for f in group.frames],
            images=[f.name for f in group.frames])

    return stats


def _write_cloud(result, group, cfg: dict, out_dir: Path,
                 calib: Calibration | None) -> None:
    """Merged colored cloud for one group, plus optional camera frustums."""
    pose_source = get_dotted(cfg, "inference.pose_source")
    conf_pct = float(get_dotted(cfg, "output.cloud.conf_pct"))
    units_m = get_dotted(cfg, "output.cloud.units") == "meters"
    unit = 1.0 if units_m else (1.0 / calib.scale_m if calib else 1.0)

    xyz_all, rgb_all = [], []
    for i in range(len(group.frames)):
        xyz, rgb = points_from_result(
            result, i, calib=calib, pose_source=pose_source,
            frames=group.frames, conf_pct=conf_pct,
            colors_bgr=cv2.cvtColor(group.images_rgb[i], cv2.COLOR_RGB2BGR))
        xyz_all.append(xyz)
        rgb_all.append(rgb)
    xyz = np.concatenate(xyz_all) * unit
    rgb = np.concatenate(rgb_all)

    cap = int(get_dotted(cfg, "output.cloud.max_points"))
    if len(xyz) > cap:
        idx = np.random.default_rng(0).choice(len(xyz), cap, replace=False)
        xyz, rgb = xyz[idx], rgb[idx]
    tag = f"pos{group.position_deg:03d}"
    dio.write_ply(out_dir / "clouds" / f"cloud_{tag}.ply", xyz, rgb)

    if get_dotted(cfg, "output.cloud.cameras") and calib is not None:
        size = 0.1 if units_m else 0.8
        pts, cols = [], []
        for i, fr in enumerate(group.frames):
            E = result.extrinsics[i]
            pred_c2w = np.eye(4)
            pred_c2w[:3, :3] = E[:3, :3].T
            pred_c2w[:3, 3] = -E[:3, :3].T @ E[:3, 3]
            known = calib.c2w(fr.camera_key).copy()
            known[:3, 3] *= calib.scale_m
            for c2w, color in ((pred_c2w, (220, 40, 40)), (known, (40, 200, 60))):
                c2w = c2w.copy()
                c2w[:3, 3] *= unit
                p, c = dio.frustum_points(c2w, size, color)
                pts.append(p)
                cols.append(c)
        dio.write_ply(out_dir / "clouds" / f"cameras_{tag}.ply",
                      np.concatenate(pts), np.concatenate(cols))


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def process_scan(cfg: dict, scan_dir: Path, frames: list[Frame], *,
                 out_dir: Path | None = None, runner: DA3Runner | None = None,
                 verbose: bool = True) -> dict:
    """
    Run inference over `frames` (grouped by turntable position) and write
    every configured output. Returns the meta dict that was written.
    """
    scan_dir = Path(scan_dir)
    pose_source = get_dotted(cfg, "inference.pose_source")
    calib = (Calibration(Path(str(get_dotted(cfg, "paths.calib_dir"))).expanduser())
             if pose_source == "calibrated" or get_dotted(cfg, "input.undistort")
             else None)
    out_dir = out_dir or resolve_out_dir(cfg, scan_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if get_dotted(cfg, "output.save_npz"):
        (out_dir / "npz").mkdir(exist_ok=True)

    groups = group_by_position(frames)
    if verbose:
        print(f"  output      : {out_dir}")
        print(f"  work        : {len(groups)} position(s), {len(frames)} frame(s)")

    runner = runner or DA3Runner(get_dotted(cfg, "model.name"),
                                 get_dotted(cfg, "model.device"), verbose=verbose)

    existing = dio.read_meta(out_dir)
    meta = None
    all_stats: dict[str, dict] = {}
    pose_devs: list[float] = []
    skip_existing = bool(get_dotted(cfg, "runtime.skip_existing"))
    preview_every = int(get_dotted(cfg, "output.preview_every"))
    t_start = time.time()

    for gi, (pos, gframes) in enumerate(groups.items()):
        if skip_existing and outputs_exist(out_dir, cfg, gframes):
            if verbose:
                print(f"  pos {pos:03d}: outputs present, skipped")
            continue

        group = prepare_group(gframes, scan_dir,
                              str(get_dotted(cfg, "input.images_subdir")), calib,
                              undistort=bool(get_dotted(cfg, "input.undistort")),
                              pose_source=pose_source)
        result = runner.run_group(
            group,
            process_res=int(get_dotted(cfg, "inference.process_res")),
            align_to_input_ext_scale=bool(
                get_dotted(cfg, "inference.align_to_input_ext_scale")))

        # First group determines the on-disk size; verify the meta agrees
        # with any existing directory BEFORE writing anything into it.
        if meta is None:
            kinds = enabled_depth_kinds(cfg)
            size = (group.image_size
                    if ("raw" in kinds and get_dotted(cfg, "output.depth.upsample_to_images"))
                    else result.size)
            meta = build_meta(cfg, scan_dir, calib, size)
            conflicts = dio.check_compatible(existing, meta)
            if conflicts:
                msg = ("output directory holds incompatible data:\n    "
                       + "\n    ".join(conflicts)
                       + f"\n  directory: {out_dir}")
                if get_dotted(cfg, "runtime.strict_meta"):
                    raise SystemExit(f"[ERROR] {msg}\n  Use a different "
                                     f"output.subdir, or set runtime.strict_meta=false.")
                print(f"  [WARN] {msg}")

        stats = write_group_outputs(
            result, group, cfg, out_dir, calib,
            write_preview=bool(preview_every and gi % preview_every == 0))
        all_stats.update(stats)
        pose_devs.append(result.pose_deviation_mm)

        if verbose:
            el = time.time() - t_start
            eta = el / (gi + 1) * (len(groups) - gi - 1)
            med = np.median([s["median_m"] for s in stats.values()])
            print(f"  pos {pos:03d} [{gi+1}/{len(groups)}]  median {med:.3f} m  "
                  f"pose dev {result.pose_deviation_mm:.2f} mm  "
                  f"{result.seconds:.1f}s  (elapsed {el:.0f}s, ~{eta:.0f}s left)")

    if meta is None:
        if verbose:
            print("  nothing to do — all outputs already present")
        return existing or {}

    meta["frames"] = all_stats
    meta["max_pose_deviation_mm"] = max(pose_devs) if pose_devs else None
    dio.write_meta(out_dir, dio.merge_frames(existing, meta))

    if verbose:
        _summarize(all_stats, pose_devs, pose_source, runner, out_dir)
    return meta


def _summarize(stats: dict, pose_devs: list[float], pose_source: str,
               runner: DA3Runner, out_dir: Path) -> None:
    if not stats:
        return
    valid = np.array([s["valid"] for s in stats.values()])
    med = np.array([s["median_m"] for s in stats.values()])
    print(f"\n  {len(stats)} frames written to {out_dir}")
    print(f"  valid {valid.mean()*100:.1f}% mean   median depth "
          f"{med.min():.3f}-{med.max():.3f} m (mean {med.mean():.3f})")
    if pose_source == "calibrated" and pose_devs:
        worst = max(pose_devs)
        note = ("poses preserved, depth is metric" if worst < 1.0
                else "WARNING: poses were altered — depth scale is suspect")
        print(f"  max pose deviation: {worst:.2f} mm  [{note}]")
    vram = runner.vram_peak_gib()
    if vram:
        print(f"  peak VRAM: {vram:.2f} GiB")
