#!/usr/bin/env python3
"""
da3_frames.py
-------------
Mapping between MOAD frame names and (camera, turntable position).

The scan's own transforms.json is the source of truth: each entry gives
file_path (images/frame_00001.jpg), camera_id (1..5) and position_deg
(0..355). The camera-major index formula

    index = (camera_id - 1) * frames_per_camera + position_index

is only used as a CONSISTENCY CHECK, never to derive the mapping, so a
scan with dropped or reordered frames still resolves correctly.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Frame:
    """One DSLR capture: which file, which camera, which turntable angle."""
    name: str            # "frame_00001.jpg"
    stem: str            # "frame_00001"
    camera_id: int       # 1..5
    camera_key: str      # "cam1" — key into cam_parameters.json
    position_deg: int    # 0..355
    index: int           # position in transforms.json (0-based)

    def image_path(self, scan_dir: Path, images_subdir: str) -> Path:
        return Path(scan_dir) / images_subdir / self.name


def load_frames(scan_dir: str | Path) -> list[Frame]:
    """Read transforms.json and return every frame, in file order."""
    scan_dir = Path(scan_dir)
    tf = scan_dir / "transforms.json"
    if not tf.is_file():
        raise FileNotFoundError(f"no transforms.json in {scan_dir}")
    with open(tf) as f:
        data = json.load(f)
    if "frames" not in data:
        raise ValueError(f"{tf}: no 'frames' array")

    frames = []
    for i, fr in enumerate(data["frames"]):
        for key in ("file_path", "camera_id", "position_deg"):
            if key not in fr:
                raise ValueError(f"{tf}: frame {i} is missing '{key}'")
        name = Path(fr["file_path"]).name
        frames.append(Frame(
            name=name,
            stem=Path(name).stem,
            camera_id=int(fr["camera_id"]),
            camera_key=f"cam{int(fr['camera_id'])}",
            position_deg=int(fr["position_deg"]),
            index=i,
        ))
    return frames


def group_by_position(frames: list[Frame]) -> dict[int, list[Frame]]:
    """Turntable position (deg) -> its frames, sorted by camera id."""
    groups: dict[int, list[Frame]] = {}
    for fr in frames:
        groups.setdefault(fr.position_deg, []).append(fr)
    return {pos: sorted(g, key=lambda f: f.camera_id)
            for pos, g in sorted(groups.items())}


def select(frames: list[Frame],
           positions: list[int] | None = None,
           cameras: list[int] | None = None) -> list[Frame]:
    """Filter by turntable position (deg) and/or camera id."""
    out = frames
    if positions is not None:
        want = set(positions)
        out = [f for f in out if f.position_deg in want]
    if cameras is not None:
        want_c = set(cameras)
        out = [f for f in out if f.camera_id in want_c]
    return out


def check_consistency(frames: list[Frame]) -> list[str]:
    """
    Cheap structural checks. Returns human-readable warnings (empty = clean)
    so callers can fail fast before loading a model.
    """
    warnings: list[str] = []
    if not frames:
        return ["transforms.json contains no frames"]

    cams = sorted({f.camera_id for f in frames})
    positions = sorted({f.position_deg for f in frames})
    n_cams, n_pos = len(cams), len(positions)

    if len(frames) != n_cams * n_pos:
        warnings.append(
            f"{len(frames)} frames != {n_cams} cameras x {n_pos} positions "
            f"— some camera/position pairs are missing or duplicated")

    counts = {}
    for f in frames:
        counts[(f.camera_id, f.position_deg)] = counts.get((f.camera_id, f.position_deg), 0) + 1
    dupes = [k for k, v in counts.items() if v > 1]
    if dupes:
        warnings.append(f"{len(dupes)} duplicated camera/position pairs, e.g. {dupes[:3]}")

    for pos, group in group_by_position(frames).items():
        if len(group) != n_cams:
            warnings.append(f"position {pos:03d} has {len(group)} frames, expected {n_cams}")

    # Camera-major ordering check (informational: the mapping never uses it)
    per_cam = len(frames) // n_cams if n_cams else 0
    if per_cam and len(frames) == n_cams * per_cam:
        step = 360 // per_cam if per_cam else 0
        mism = sum(1 for i, f in enumerate(frames)
                   if (f.camera_id, f.position_deg) != (i // per_cam + 1, (i % per_cam) * step))
        if mism:
            warnings.append(
                f"{mism} frames deviate from camera-major ordering "
                f"(harmless — mapping comes from transforms.json)")
    return warnings


def describe(frames: list[Frame]) -> str:
    cams = sorted({f.camera_id for f in frames})
    pos = sorted({f.position_deg for f in frames})
    return (f"{len(frames)} frames, cameras {cams}, "
            f"{len(pos)} positions ({pos[0]}..{pos[-1]} deg)" if frames else "no frames")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Inspect a scan's frame mapping.")
    ap.add_argument("scan_dir")
    ap.add_argument("--position", type=int, default=None)
    a = ap.parse_args()
    fr = load_frames(a.scan_dir)
    print(describe(fr))
    for w in check_consistency(fr):
        print(f"  [WARN] {w}")
    if a.position is not None:
        for f in group_by_position(fr).get(a.position, []):
            print(f"  {f.name}  {f.camera_key}  pos {f.position_deg:03d}")
