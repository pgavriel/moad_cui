#!/usr/bin/env python3
"""
create_calibration.py
---------------------
Consolidates COLMAP outputs + alignment transform + scaling info into a single
cam_parameters.json, and copies the four source files into {output}/base_files/.

Usage:
    python3 create_calibration.py <output_folder> <input_folder>

Expected input layout:
    {input}/alignment_tf.txt
    {input}/scaling.yaml
    {input}/sparse/0/cameras.txt
    {input}/sparse/0/images.txt
"""

import sys
import os
import json
import shutil
import math
import numpy as np
import yaml


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def section(title: str) -> None:
    """Print a clearly visible section header."""
    bar = "─" * 60
    print(f"\n{bar}")
    print(f"  {title}")
    print(bar)


def ok(msg: str) -> None:
    print(f"  [OK]      {msg}")


def info(msg: str) -> None:
    print(f"  [INFO]    {msg}")


def warn(msg: str) -> None:
    print(f"  [WARN]    {msg}")


def fail(msg: str) -> None:
    print(f"  [MISSING] {msg}")


# ─────────────────────────────────────────────────────────────────────────────
# File parsers
# ─────────────────────────────────────────────────────────────────────────────

def parse_alignment_tf(path: str) -> list[list[float]]:
    """
    Parse a 4×4 homogeneous transform matrix from a text file.
    Lines beginning with '#' are treated as comments and skipped.
    """
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = [float(v) for v in line.split()]
            if len(values) != 4:
                raise ValueError(
                    f"alignment_tf.txt: expected 4 values per row, got {len(values)}: '{line}'"
                )
            rows.append(values)
    if len(rows) != 4:
        raise ValueError(
            f"alignment_tf.txt: expected 4 rows, got {len(rows)}"
        )
    info(f"Alignment transform parsed  ({len(rows)}×4 matrix)")
    for r in rows:
        info(f"    {r}")
    return rows


def parse_scaling_yaml(path: str) -> dict:
    """
    Parse scaling.yaml and derive the scale factor:
        scale = d_real / d_cloud
    Returns a dict with all raw fields plus the computed scale.
    """
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    required = {"d_cloud", "d_real", "units"}
    missing = required - data.keys()
    if missing:
        raise ValueError(f"scaling.yaml is missing keys: {missing}")

    d_cloud = float(data["d_cloud"])
    d_real  = float(data["d_real"])
    units   = str(data["units"])
    scale   = d_real / d_cloud

    info(f"Scaling loaded:")
    info(f"    d_cloud = {d_cloud}")
    info(f"    d_real  = {d_real}  ({units})")
    info(f"    scale   = {scale}  (d_real / d_cloud)")

    return {
        "d_cloud": d_cloud,
        "d_real":  d_real,
        "units":   units,
        "scale":   scale,
    }


def parse_colmap_cameras(path: str) -> dict:
    """
    Parse COLMAP cameras.txt.
    Supported models: SIMPLE_RADIAL, RADIAL, OPENCV, PINHOLE, SIMPLE_PINHOLE.
    Returns a dict keyed by camera_id (int).
    """
    cameras = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            cam_id = int(parts[0])
            model  = parts[1]
            width  = int(parts[2])
            height = int(parts[3])
            params = [float(v) for v in parts[4:]]

            # Map params to named intrinsics depending on model
            if model == "SIMPLE_PINHOLE":
                # params: f, cx, cy
                fx = fy = params[0]
                cx, cy = params[1], params[2]
                dist = {}
            elif model == "PINHOLE":
                # params: fx, fy, cx, cy
                fx, fy = params[0], params[1]
                cx, cy = params[2], params[3]
                dist = {}
            elif model == "SIMPLE_RADIAL":
                # params: f, cx, cy, k1
                fx = fy = params[0]
                cx, cy = params[1], params[2]
                dist = {"k1": params[3], "k2": 0.0, "p1": 0.0, "p2": 0.0}
            elif model == "RADIAL":
                # params: f, cx, cy, k1, k2
                fx = fy = params[0]
                cx, cy = params[1], params[2]
                dist = {"k1": params[3], "k2": params[4], "p1": 0.0, "p2": 0.0}
            elif model == "OPENCV":
                # params: fx, fy, cx, cy, k1, k2, p1, p2
                fx, fy = params[0], params[1]
                cx, cy = params[2], params[3]
                dist = {"k1": params[4], "k2": params[5],
                        "p1": params[6], "p2": params[7]}
            else:
                warn(f"Unrecognised camera model '{model}' for cam {cam_id}. "
                     f"Storing raw params only.")
                cameras[cam_id] = {
                    "model": model, "width": width, "height": height,
                    "raw_params": params,
                }
                continue

            cameras[cam_id] = {
                "model":  model,
                "width":  width,
                "height": height,
                "fx": fx, "fy": fy,
                "cx": cx, "cy": cy,
                "distortion": dist,
            }

    info(f"cameras.txt parsed  →  {len(cameras)} camera(s)")
    for cid, c in cameras.items():
        info(f"    cam_id={cid}  model={c['model']}  {c['width']}×{c['height']}"
             f"  fx={c.get('fx','?'):.3f}  fy={c.get('fy','?'):.3f}"
             f"  cx={c.get('cx','?'):.1f}  cy={c.get('cy','?'):.1f}")
    return cameras


def qvec_to_rotmat(qw, qx, qy, qz) -> np.ndarray:
    """Convert a unit quaternion (w, x, y, z) to a 3×3 rotation matrix."""
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    return R


def parse_colmap_images(path: str) -> dict:
    """
    Parse COLMAP images.txt.

    Format (two lines per image):
      IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
      POINTS2D[] as (X Y POINT3D_ID) per detected keypoint

    The quaternion (QW QX QY QZ) and translation (TX TY TZ) describe the
    world-to-camera transform:
        p_cam = R * p_world + t

    We also compute the camera-to-world (c2w) transform:
        c2w_R = R^T
        c2w_t = -R^T @ t

    Returns a dict keyed by image name (str).
    """
    images = {}
    with open(path, "r") as f:
        lines = [l.rstrip() for l in f if l.strip() and not l.startswith("#")]

    if len(lines) % 2 != 0:
        raise ValueError(
            "images.txt: expected an even number of non-comment lines "
            f"(got {len(lines)}).  Each image needs exactly 2 lines."
        )

    for i in range(0, len(lines), 2):
        meta_line   = lines[i]
        # lines[i+1] is the 2D point list — we don't need it here

        parts = meta_line.split()
        img_id    = int(parts[0])
        qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        tx, ty, tz      = float(parts[5]), float(parts[6]), float(parts[7])
        cam_id    = int(parts[8])
        name      = parts[9]          # e.g. "images/cam1_00001.jpg"

        # World-to-camera
        R_w2c = qvec_to_rotmat(qw, qx, qy, qz)
        t_w2c = np.array([tx, ty, tz])

        w2c = np.eye(4)
        w2c[:3, :3] = R_w2c
        w2c[:3,  3] = t_w2c

        # Camera-to-world
        R_c2w = R_w2c.T
        t_c2w = -R_w2c.T @ t_w2c

        c2w = np.eye(4)
        c2w[:3, :3] = R_c2w
        c2w[:3,  3] = t_c2w

        images[name] = {
            "image_id": img_id,
            "camera_id": cam_id,
            "qvec": [qw, qx, qy, qz],
            "tvec": [tx, ty, tz],
            "w2c":  w2c,
            "c2w":  c2w,
        }

    info(f"images.txt parsed  →  {len(images)} image(s)")
    for name, d in sorted(images.items(), key=lambda x: x[1]["image_id"]):
        t = d["c2w"][:3, 3]
        info(f"    id={d['image_id']}  cam={d['camera_id']}  {name}"
             f"  pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})")
    return images


# ─────────────────────────────────────────────────────────────────────────────
# Math utilities
# ─────────────────────────────────────────────────────────────────────────────

def mat4_to_nested_list(M: np.ndarray) -> list[list[float]]:
    """Convert a 4×4 numpy array to a nested Python list (JSON-serialisable)."""
    return M.tolist()


def apply_alignment(c2w: np.ndarray, align_tf: np.ndarray) -> np.ndarray:
    """
    Apply the alignment transform to a camera-to-world matrix.
        c2w_aligned = align_tf @ c2w
    """
    return align_tf @ c2w


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # ── 0. Argument parsing ──────────────────────────────────────────────────
    script_dir = os.path.dirname(os.path.realpath(__file__))
    if len(sys.argv) != 3:
        print("Usage: python3 create_calibration.py <output_name> <input_folder>")
        print("\tRunning with defaults...")
        out_dir = os.path.join(script_dir,"../calibration/55mm_joint")
        in_dir  = "/home/csrobot/MOAD_DATA/calibration_v4/pose-b/calib-test"
        # sys.exit(1)
    else:
        out_dir = os.path.join(script_dir,"../calibration",sys.argv[1])
        in_dir  = sys.argv[2]

    section("ARGUMENTS")
    info(f"Input  folder : {os.path.abspath(in_dir)}")
    info(f"Output folder : {os.path.abspath(out_dir)}")

    # ── 1. Verify required input files ──────────────────────────────────────
    section("VERIFYING INPUT FILES")

    required_files = {
        "alignment_tf.txt": os.path.join(in_dir, "alignment_tf.txt"),
        "scaling.yaml":     os.path.join(in_dir, "scaling.yaml"),
        "cameras.txt":      [os.path.join(in_dir, "sparse", "0", "cameras.txt"),os.path.join(in_dir, "cameras.txt")],
        "images.txt":       [os.path.join(in_dir, "sparse", "0", "images.txt"),os.path.join(in_dir, "images.txt")],
    }

    all_found = True
    for label, fpath in required_files.items():
        # This branch allows the generation of new calibrations from the file structure of copied base_files
        # This means you could make an adjustment to one of the base files (alignment, scale, etc) and generate a fresh calibration file from them.
        if isinstance(fpath, list): # Check if one of the paths exists
            found = False
            for p in fpath:
                if os.path.isfile(p):
                    ok(f"{label}  →  {p}")
                    required_files[label] = p
                    found = True
            if not found:
                all_found = False
        else:
            if os.path.isfile(fpath):
                ok(f"{label}  →  {fpath}")
            else:
                fail(f"{label}  →  {fpath}")
                all_found = False

    if not all_found:
        print("\n  ✗  One or more required files are missing. Aborting.\n")
        sys.exit(1)

    print("\n  ✓  All required files found. Proceeding.\n")

    # ── 2. Create output directories ─────────────────────────────────────────
    section("CREATING OUTPUT DIRECTORIES")

    base_files_dir = os.path.join(out_dir, "base_files")
    await_confirm = False
    for d in [out_dir, base_files_dir]:
        if os.path.exists(d):
            warn(f"Directory already exists, contents may be overwritten: {d}")
            await_confirm = True
        else:
            os.makedirs(d)
            ok(f"Created: {d}")
    if await_confirm:
        input("Continue?: (Ctrl+C to Quit)")

    # ── 3. Copy source files ──────────────────────────────────────────────────
    section("COPYING BASE FILES")

    for label, src_path in required_files.items():
        dst_path = os.path.join(base_files_dir, label)
        shutil.copy2(src_path, dst_path)
        ok(f"{label}  →  {dst_path}")

    # ── 4. Parse all source files ─────────────────────────────────────────────
    section("PARSING ALIGNMENT TRANSFORM  (alignment_tf.txt)")
    align_tf_nested = parse_alignment_tf(required_files["alignment_tf.txt"])
    align_tf = np.array(align_tf_nested)

    section("PARSING SCALING  (scaling.yaml)")
    scaling = parse_scaling_yaml(required_files["scaling.yaml"])

    section("PARSING COLMAP CAMERAS  (cameras.txt)")
    colmap_cameras = parse_colmap_cameras(required_files["cameras.txt"])

    section("PARSING COLMAP IMAGES  (images.txt)")
    colmap_images = parse_colmap_images(required_files["images.txt"])

    # ── 5. Build output camera parameters ────────────────────────────────────
    section("BUILDING CAMERA PARAMETERS")

    # --- Build a per-camera-id intrinsics lookup from cameras.txt.
    # When all images share one camera model (DSLR-only calibration) only one
    # entry exists and behaviour is identical to the original code. When a joint
    # DSLR + RealSense reconstruction is provided, each sensor type gets its own
    # intrinsics block looked up by the COLMAP camera_id stored in images.txt.
    def build_intrinsics_block(cam: dict) -> dict:
        return {
            "fx":           cam.get("fx"),
            "fy":           cam.get("fy"),
            "cx":           cam.get("cx"),
            "cy":           cam.get("cy"),
            "width":        cam.get("width"),
            "height":       cam.get("height"),
            "camera_model": cam.get("model"),
            "distortion":   cam.get("distortion", {}),
        }

    intrinsics_by_id = {
        cam_id: build_intrinsics_block(cam)
        for cam_id, cam in colmap_cameras.items()
    }

    info(f"Intrinsic models loaded: {len(intrinsics_by_id)}")
    for cam_id, intr in intrinsics_by_id.items():
        info(f"    camera_id={cam_id}  {intr['width']}x{intr['height']}"
             f"  fx={intr['fx']:.4f}  model={intr['camera_model']}")

    # --- Detect which camera_id belongs to each sensor type by inspecting the
    # filename prefix of each image in images.txt:
    #   DSLR images  : start with "cam"  (e.g. cam1_000_img.jpg)
    #   RealSense    : start with "rs"   (e.g. rs1_000_img.jpg)
    # This relies on the naming convention established during collection.
    dslr_camera_id = None
    rs_camera_id   = None

    for img_name, img_data in colmap_images.items():
        basename_lower = os.path.basename(img_name).lower()
        if basename_lower.startswith("cam") and dslr_camera_id is None:
            dslr_camera_id = img_data["camera_id"]
            info(f"DSLR camera_id detected: {dslr_camera_id}  (from {os.path.basename(img_name)})")
        elif basename_lower.startswith("rs") and rs_camera_id is None:
            rs_camera_id = img_data["camera_id"]
            info(f"RealSense camera_id detected: {rs_camera_id}  (from {os.path.basename(img_name)})")
        if dslr_camera_id is not None and rs_camera_id is not None:
            break

    if dslr_camera_id is None:
        warn("No DSLR images (cam* prefix) found in images.txt — cam_parameters.json will not be written")
    if rs_camera_id is None:
        info("No RealSense images (rs* prefix) found — realsense_cam_parameters.json will not be written")

    # --- Process all images, splitting into DSLR and RealSense output dicts
    # based on each image's camera_id. The cam_key is derived from the filename
    # prefix (e.g. "cam1" or "rs3") so output keys match the naming convention.
    cameras_dslr = {}
    cameras_rs   = {}

    for img_name, img_data in sorted(colmap_images.items(),
                                     key=lambda x: x[1]["image_id"]):
        c2w_raw = img_data["c2w"]

        # Apply alignment transform to c2w
        c2w_aligned = apply_alignment(c2w_raw, align_tf)

        # Recompute w2c from aligned c2w (invert 4x4 rigid transform)
        R_aligned = c2w_aligned[:3, :3]
        t_aligned = c2w_aligned[:3,  3]
        w2c_aligned = np.eye(4)
        w2c_aligned[:3, :3] = R_aligned.T
        w2c_aligned[:3,  3] = -R_aligned.T @ t_aligned

        # Derive a short key from the filename prefix: "cam1", "rs3", etc.
        basename = os.path.basename(img_name)
        stem     = os.path.splitext(basename)[0]
        cam_key  = stem.split("_")[0] if "_" in stem else stem

        info(f"  Processing: {img_name}  →  key='{cam_key}'")
        t = c2w_aligned[:3, 3]
        info(f"    Aligned position: ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})\n")

        entry = {
            "source_file":  img_name,
            "colmap_im_id": img_data["image_id"],
            "camera_id":    img_data["camera_id"],
            "extrinsics": {
                "c2w": mat4_to_nested_list(c2w_aligned),
                "w2c": mat4_to_nested_list(w2c_aligned),
            },
        }

        if img_data["camera_id"] == dslr_camera_id:
            cameras_dslr[cam_key] = entry
        elif img_data["camera_id"] == rs_camera_id:
            cameras_rs[cam_key] = entry
        else:
            warn(f"  Unknown camera_id {img_data['camera_id']} for {img_name}, skipping")

    info(f"DSLR cameras processed    : {len(cameras_dslr)}")
    info(f"RealSense cameras processed: {len(cameras_rs)}")

    # --- Shared _info block — identical for both output files since alignment
    # and scale are derived from the same reconstruction.
    info_block = {
        "description": (
            "Consolidated camera parameters with alignment transform applied."
        ),
        "extrinsic_convention": (
            "c2w = camera-to-world (aligned). "
            "w2c = world-to-camera (aligned)."
        ),
        "alignment_tf": align_tf_nested,
        "scaling": {
            "description": (
                "Scale factor derived from reference measurement. "
                "Multiply cloud-space distances by 'scale' to obtain real-world distances."
            ),
            "d_cloud": scaling["d_cloud"],
            "d_real":  scaling["d_real"],
            "units":   scaling["units"],
            "scale":   scaling["scale"],
        },
    }

    # TODO: Create transforms.json output file for NeRF training / transform generation

    # ── 6. Write output files ─────────────────────────────────────────────────
    section("WRITING OUTPUT FILES")

    # --- cam_parameters.json (DSLR) — filename unchanged so all existing
    # downstream consumers (transform_generator, scene_replica, etc.) work
    # without modification.
    if cameras_dslr and dslr_camera_id is not None:
        dslr_output = {
            "_info": {
                **info_block,
                "intrinsic_note": "Shared intrinsics apply to all DSLR cameras.",
            },
            "intrinsics": intrinsics_by_id[dslr_camera_id],
            "cameras":    cameras_dslr,
        }
        dslr_path = os.path.join(out_dir, "cam_parameters.json")
        with open(dslr_path, "w") as f:
            json.dump(dslr_output, f, indent=2)
        ok(f"cam_parameters.json written  →  {dslr_path}  ({len(cameras_dslr)} cameras)")
    else:
        warn("Skipping cam_parameters.json — no DSLR cameras found")

    # --- realsense_cam_parameters.json — only written when rs* images are
    # present in the reconstruction. Same format as cam_parameters.json so
    # any future RealSense-aware consumers can use the same reader code.
    if cameras_rs and rs_camera_id is not None:
        rs_output = {
            "_info": {
                **info_block,
                "intrinsic_note": "Shared intrinsics apply to all RealSense cameras.",
            },
            "intrinsics": intrinsics_by_id[rs_camera_id],
            "cameras":    cameras_rs,
        }
        rs_path = os.path.join(out_dir, "realsense_cam_parameters.json")
        with open(rs_path, "w") as f:
            json.dump(rs_output, f, indent=2)
        ok(f"realsense_cam_parameters.json written  →  {rs_path}  ({len(cameras_rs)} cameras)")
    else:
        info("Skipping realsense_cam_parameters.json — no RealSense cameras found")

    # ── 7. Summary ────────────────────────────────────────────────────────────
    section("SUMMARY")
    info(f"Output folder            : {os.path.abspath(out_dir)}")
    info(f"Base files folder        : {os.path.abspath(base_files_dir)}")
    info(f"DSLR cameras written     : {len(cameras_dslr)}")
    info(f"RealSense cameras written: {len(cameras_rs)}")
    info(f"Scale factor             : {scaling['scale']:.8f}  ({scaling['units']})")
    print(f"\n  ✓  Done.\n")



if __name__ == "__main__":
    main()
