#!/usr/bin/env python3
"""
da3_config.py
-------------
Configuration loading for the MOAD Depth Anything 3 tools.

Resolution order (highest priority first):

    1. command-line overrides      (dict of dotted keys -> value)
    2. MOAD_DA3_CONFIG env var     (path to a config file)
    3. --config PATH               (explicit file)
    4. da3_config.yaml next to this module
    5. DEFAULTS below

Only keys present in a user config override the defaults; everything else
is inherited, so a user file can be three lines long.

Typical use from a run_* script:

    cfg = load_config(args.config, overrides={
        "inference.process_res": args.process_res,   # None values ignored
        "input.images_subdir":   args.images_subdir,
    })
    validate(cfg)
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:                                    # pragma: no cover
    yaml = None

# Built-in fallback: mirrors da3_config.yaml so the tools still run if the
# YAML file is missing. Keep the two in sync when adding keys.
DEFAULTS: dict[str, Any] = {
    "paths": {
        "data_root": "/home/csrobot/MOAD_DATA",
        "calib_dir": "/home/csrobot/moad_control/moad_cui/calibration/55mm_joint",
    },
    "model": {"name": "depth-anything/da3-base", "device": "auto"},
    "inference": {
        "process_res": 504,
        "pose_source": "calibrated",          # calibrated | none
        "align_to_input_ext_scale": True,
    },
    "input": {"images_subdir": "images_4", "undistort": True},
    "output": {
        "subdir": "DSLR_depth_da3",
        "auto_suffix": False,
        "depth": {"raw": True, "undistorted": False, "upsample_to_images": True},
        "cloud": {"enabled": False, "conf_pct": 20.0, "units": "meters",
                  "max_points": 3_000_000, "cameras": True},
        "preview_every": 12,
        "save_npz": False,
    },
    "runtime": {"skip_existing": False, "strict_meta": True},
}

DEFAULT_CONFIG_NAME = "da3_config.yaml"
ENV_VAR = "MOAD_DA3_CONFIG"


# ---------------------------------------------------------------------------
# Loading / merging
# ---------------------------------------------------------------------------

def _deep_merge(base: dict, over: dict) -> dict:
    """Recursively merge `over` into a copy of `base` (dicts only)."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _read_file(path: Path) -> dict:
    """Read a .yaml/.yml/.json config file into a dict."""
    text = path.read_text()
    if path.suffix.lower() in (".yaml", ".yml"):
        if yaml is None:
            raise RuntimeError(
                f"{path} is YAML but PyYAML is not installed "
                f"(pip install pyyaml), or convert the file to JSON")
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be a mapping, got {type(data).__name__}")
    return data


def config_path(explicit: str | os.PathLike | None = None) -> Path | None:
    """Resolve which config file to read, or None to use DEFAULTS only."""
    if explicit:
        p = Path(explicit).expanduser()
        if not p.is_file():
            raise FileNotFoundError(f"config file not found: {p}")
        return p
    env = os.environ.get(ENV_VAR)
    if env:
        p = Path(env).expanduser()
        if not p.is_file():
            raise FileNotFoundError(f"{ENV_VAR} points at a missing file: {p}")
        return p
    local = Path(__file__).resolve().parent / DEFAULT_CONFIG_NAME
    return local if local.is_file() else None


def set_dotted(cfg: dict, dotted: str, value: Any) -> None:
    """Set cfg["a"]["b"] from "a.b"; intermediate dicts must exist."""
    keys = dotted.split(".")
    node = cfg
    for k in keys[:-1]:
        if k not in node or not isinstance(node[k], dict):
            raise KeyError(f"unknown config section: {'.'.join(keys[:-1])}")
        node = node[k]
    if keys[-1] not in node:
        raise KeyError(f"unknown config key: {dotted}")
    node[keys[-1]] = value


def get_dotted(cfg: dict, dotted: str, default: Any = None) -> Any:
    node: Any = cfg
    for k in dotted.split("."):
        if not isinstance(node, dict) or k not in node:
            return default
        node = node[k]
    return node


def load_config(explicit: str | os.PathLike | None = None,
                overrides: dict[str, Any] | None = None) -> dict:
    """
    Build the resolved configuration.

    `overrides` maps dotted keys to values; entries whose value is None are
    IGNORED, which lets argparse defaults stay None and never silently beat
    the config file.
    """
    path = config_path(explicit)
    cfg = _deep_merge(DEFAULTS, _read_file(path) if path else {})
    cfg["_meta"] = {"config_file": str(path) if path else None}
    for dotted, value in (overrides or {}).items():
        if value is not None:
            set_dotted(cfg, dotted, value)
    return cfg


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate(cfg: dict, require_paths: bool = True) -> None:
    """Raise ValueError with a specific message on the first problem found."""
    def _need(dotted, types, check=None, hint=""):
        val = get_dotted(cfg, dotted, KeyError)
        if val is KeyError:
            raise ValueError(f"missing config key: {dotted}")
        if not isinstance(val, types):
            raise ValueError(f"{dotted}: expected {types}, got {type(val).__name__}")
        if check and not check(val):
            raise ValueError(f"{dotted}: invalid value {val!r}. {hint}")

    _need("model.name", str)
    _need("model.device", str, lambda v: v in ("auto", "cuda", "cpu"),
          "use auto | cuda | cpu")
    _need("inference.process_res", int, lambda v: 56 <= v <= 4096,
          "typical values 504-1008")
    _need("inference.pose_source", str, lambda v: v in ("calibrated", "none"),
          "use calibrated | none")
    _need("inference.align_to_input_ext_scale", bool)
    _need("input.images_subdir", str)
    _need("input.undistort", bool)
    _need("output.subdir", str, lambda v: bool(v.strip()))
    _need("output.preview_every", int, lambda v: v >= 0)
    _need("output.cloud.conf_pct", (int, float), lambda v: 0 <= v < 100)
    _need("output.cloud.units", str, lambda v: v in ("meters", "model"))

    if not (get_dotted(cfg, "output.depth.raw")
            or get_dotted(cfg, "output.depth.undistorted")
            or get_dotted(cfg, "output.cloud.enabled")):
        raise ValueError("no outputs enabled: set output.depth.raw, "
                         "output.depth.undistorted or output.cloud.enabled")

    if get_dotted(cfg, "inference.pose_source") == "none":
        if get_dotted(cfg, "output.depth.raw"):
            raise ValueError(
                "inference.pose_source='none' produces depth in the model's own "
                "frame; raw-geometry output assumes the calibrated camera model. "
                "Set output.depth.raw=false (use undistorted) for this mode.")

    if not get_dotted(cfg, "input.undistort") and \
            get_dotted(cfg, "inference.pose_source") == "calibrated":
        raise ValueError(
            "input.undistort=false with pose_source='calibrated': DA3 takes a "
            "pinhole K, so distorted images would disagree with the intrinsics")

    if require_paths:
        for key in ("paths.data_root", "paths.calib_dir"):
            p = Path(str(get_dotted(cfg, key))).expanduser()
            if not p.is_dir():
                raise ValueError(f"{key}: directory does not exist: {p}")
        calib = Path(str(get_dotted(cfg, "paths.calib_dir"))).expanduser()
        if not (calib / "cam_parameters.json").is_file():
            raise ValueError(f"paths.calib_dir: no cam_parameters.json in {calib}")


# ---------------------------------------------------------------------------
# Identity / reporting
# ---------------------------------------------------------------------------

def settings_fingerprint(cfg: dict) -> dict:
    """The subset of settings that changes the numbers in the output."""
    return {
        "model": get_dotted(cfg, "model.name"),
        "process_res": get_dotted(cfg, "inference.process_res"),
        "pose_source": get_dotted(cfg, "inference.pose_source"),
        "align_to_input_ext_scale": get_dotted(cfg, "inference.align_to_input_ext_scale"),
        "images_subdir": get_dotted(cfg, "input.images_subdir"),
        "undistort": get_dotted(cfg, "input.undistort"),
        "calib_dir": str(get_dotted(cfg, "paths.calib_dir")),
    }


def config_hash(cfg: dict) -> str:
    """Stable short hash of the result-affecting settings."""
    blob = json.dumps(settings_fingerprint(cfg), sort_keys=True).encode()
    return hashlib.sha256(blob).hexdigest()[:12]


def output_subdir(cfg: dict) -> str:
    """Output folder name, optionally suffixed with the key settings."""
    name = str(get_dotted(cfg, "output.subdir"))
    if not get_dotted(cfg, "output.auto_suffix"):
        return name
    model = str(get_dotted(cfg, "model.name")).split("/")[-1]
    return (f"{name}_{model}_r{get_dotted(cfg, 'inference.process_res')}"
            f"_{get_dotted(cfg, 'inference.pose_source')}")


def summary(cfg: dict) -> str:
    """One-block human summary printed at the start of every run."""
    src = get_dotted(cfg, "_meta.config_file") or "built-in defaults"
    depth_kinds = [k for k in ("raw", "undistorted")
                   if get_dotted(cfg, f"output.depth.{k}")]
    return "\n".join([
        f"  config      : {src}  (hash {config_hash(cfg)})",
        f"  model       : {get_dotted(cfg, 'model.name')}  "
        f"process_res={get_dotted(cfg, 'inference.process_res')}  "
        f"pose={get_dotted(cfg, 'inference.pose_source')}",
        f"  input       : {get_dotted(cfg, 'input.images_subdir')}  "
        f"(undistort={get_dotted(cfg, 'input.undistort')}) — note: input size "
        f"does not change the model's output resolution",
        f"  outputs     : depth[{', '.join(depth_kinds) or 'none'}]"
        f"  cloud={get_dotted(cfg, 'output.cloud.enabled')}"
        f"  npz={get_dotted(cfg, 'output.save_npz')}",
    ])


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Show the resolved DA3 config.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-path-check", action="store_true")
    a = ap.parse_args()
    c = load_config(a.config)
    try:
        validate(c, require_paths=not a.no_path_check)
        status = "valid"
    except ValueError as e:
        status = f"INVALID: {e}"
    print(summary(c))
    print(f"  validation  : {status}")
    print(f"  fingerprint : {json.dumps(settings_fingerprint(c), indent=2)}")
