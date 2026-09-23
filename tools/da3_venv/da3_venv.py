#!/usr/bin/env python3
"""
da3_venv.py
-----------
Make DA3 tools runnable with ANY interpreter: if the current process is
not already inside the DA3 virtualenv, re-exec the same script with the
venv's python. No shell activation involved — activation only edits PATH,
and a child process cannot change its parent's environment anyway.

Use at the very top of a tool script, before importing torch / DA3:

    from da3_venv import ensure_venv
    ensure_venv()

Then all of these work identically:
    python3 da3_batch_scan.py
    ~/venvs/da3/bin/python da3_batch_scan.py
    (from inside an activated venv) python da3_batch_scan.py

ENV
    DA3_VENV        venv location (default ~/venvs/da3)
    DA3_NO_BOOTSTRAP=1   disable re-exec (use the current interpreter)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_GUARD = "DA3_VENV_BOOTSTRAPPED"


def venv_dir() -> Path:
    return Path(os.environ.get("DA3_VENV", Path.home() / "venvs" / "da3"))


def venv_python(root: Path | None = None) -> Path:
    root = root or venv_dir()
    return root / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def in_venv(root: Path | None = None) -> bool:
    """True if the running interpreter belongs to the target venv."""
    root = (root or venv_dir()).resolve()
    try:
        return Path(sys.prefix).resolve() == root
    except OSError:
        return False


def ensure_venv(required: bool = True, verbose: bool = True) -> None:
    """
    Re-exec the current script inside the DA3 venv when needed.

    required=False: warn and continue in the current interpreter if the
    venv is missing (useful for tools whose non-DA3 paths still work).
    """
    if os.environ.get("DA3_NO_BOOTSTRAP") == "1":
        return
    root = venv_dir()
    if in_venv(root):
        return

    status()

    if os.environ.get(_GUARD) == "1":
        # Already re-exec'd once and still not inside — avoid an exec loop.
        msg = (f"[da3-venv] re-exec did not land in {root} "
               f"(sys.prefix={sys.prefix}); continuing as-is")
        print(msg, file=sys.stderr)
        return

    py = venv_python(root)
    if not py.is_file():
        msg = (f"[da3-venv] venv not found at {root}\n"
               f"           create it with:  ./setup_da3_env.sh\n"
               f"           or set DA3_VENV to an existing venv")
        if required:
            sys.exit(msg)
        print(msg, file=sys.stderr)
        return

    if verbose:
        print(f"[da3-venv] switching to {py}", file=sys.stderr)
    env = dict(os.environ, **{_GUARD: "1"})
    script = Path(sys.argv[0]).resolve()
    os.execve(str(py), [str(py), str(script), *sys.argv[1:]], env)


def status() -> str:
    root = venv_dir()
    return (f"venv        : {root}\n"
            f"exists      : {venv_python(root).is_file()}\n"
            f"active now  : {in_venv(root)}\n"
            f"interpreter : {sys.executable}\n"
            f"sys.prefix  : {sys.prefix}")


if __name__ == "__main__":
    print(status())
