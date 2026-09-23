#!/usr/bin/env bash
# setup_da3_env.sh — create / activate the Depth Anything 3 virtualenv.
#
# USAGE
#   source scripts/setup_da3_env.sh      # create if needed, then ACTIVATE
#   ./scripts/setup_da3_env.sh           # create + install only (cannot
#                                        # activate: a child process can't
#                                        # modify the parent shell)
#   DA3_VENV=/opt/venvs/da3 source scripts/setup_da3_env.sh
#
# ENV
#   DA3_VENV        venv location          (default ~/venvs/da3)
#   DA3_TORCH_INDEX torch wheel index      (default cu124 build)
#   DA3_PYTHON      interpreter to build with (default python3)
#
# Behaviour
#   venv missing          -> create, install requirements, activate (if sourced)
#   venv exists, inactive -> activate (if sourced)
#   venv exists, active   -> report and do nothing
#   requirements changed  -> reinstall (tracked by a hash marker)

DA3_VENV="${DA3_VENV:-$HOME/venvs/da3}"
DA3_PYTHON="${DA3_PYTHON:-python3}"
DA3_TORCH_INDEX="${DA3_TORCH_INDEX:-https://download.pytorch.org/whl/cu124}"

# Are we being sourced? (bash + zsh)
_da3_sourced=0
if [ -n "${BASH_SOURCE[0]}" ]; then
    [ "${BASH_SOURCE[0]}" != "$0" ] && _da3_sourced=1
elif [ -n "${ZSH_EVAL_CONTEXT}" ]; then
    case "$ZSH_EVAL_CONTEXT" in *:file) _da3_sourced=1 ;; esac
fi

_da3_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_da3_reqs="$_da3_dir/requirements-da3.txt"
_da3_marker="$DA3_VENV/.da3_requirements_hash"

_da3_log() { printf '  [da3-env] %s\n' "$1"; }

# --- already active? -------------------------------------------------------
if [ -n "$VIRTUAL_ENV" ] && [ "$VIRTUAL_ENV" = "$DA3_VENV" ]; then
    _da3_log "venv already active: $VIRTUAL_ENV"
    return 0 2>/dev/null || exit 0
fi
if [ -n "$VIRTUAL_ENV" ]; then
    _da3_log "WARNING: a different venv is active ($VIRTUAL_ENV)"
    _da3_log "         deactivate it first, or run with DA3_VENV=$VIRTUAL_ENV"
fi

# --- create if missing -----------------------------------------------------
_da3_fresh=0
if [ ! -x "$DA3_VENV/bin/python" ]; then
    _da3_log "creating venv at $DA3_VENV"
    mkdir -p "$(dirname "$DA3_VENV")" || { _da3_log "cannot create parent dir"; return 1 2>/dev/null || exit 1; }
    "$DA3_PYTHON" -m venv "$DA3_VENV" || { _da3_log "venv creation failed"; return 1 2>/dev/null || exit 1; }
    _da3_fresh=1
else
    _da3_log "venv found at $DA3_VENV"
fi

# --- install / refresh dependencies ---------------------------------------
if [ -f "$_da3_reqs" ]; then
    _da3_hash="$(sha256sum "$_da3_reqs" | cut -d' ' -f1)"
else
    _da3_hash="no-requirements-file"
fi

if [ "$_da3_fresh" = "1" ] || [ ! -f "$_da3_marker" ] || \
   [ "$(cat "$_da3_marker" 2>/dev/null)" != "$_da3_hash" ]; then
    _da3_log "installing dependencies (torch index: $DA3_TORCH_INDEX)"
    "$DA3_VENV/bin/python" -m pip install -q -U pip wheel || {
        _da3_log "pip bootstrap failed"; return 1 2>/dev/null || exit 1; }
    if ! "$DA3_VENV/bin/python" -c 'import torch' 2>/dev/null; then
        "$DA3_VENV/bin/python" -m pip install torch torchvision \
            --index-url "$DA3_TORCH_INDEX" || {
            _da3_log "torch install failed — check DA3_TORCH_INDEX vs nvidia-smi"
            return 1 2>/dev/null || exit 1; }
    fi
    if [ -f "$_da3_reqs" ]; then
        "$DA3_VENV/bin/python" -m pip install -r "$_da3_reqs" || {
            _da3_log "requirements install failed"; return 1 2>/dev/null || exit 1; }
    fi
    printf '%s' "$_da3_hash" > "$_da3_marker"
    _da3_log "dependencies installed"
else
    _da3_log "dependencies up to date"
fi

# --- activate (only possible when sourced) --------------------------------
if [ "$_da3_sourced" = "1" ]; then
    # shellcheck disable=SC1091
    . "$DA3_VENV/bin/activate"
    _da3_log "activated: $(python --version 2>&1), python at $(command -v python)"
else
    _da3_log "not sourced — venv is ready but this shell is unchanged."
    _da3_log "activate with:  source ${BASH_SOURCE[0]:-$0}"
    _da3_log "or just run tools directly:  $DA3_VENV/bin/python <script>.py"
fi

unset _da3_sourced _da3_dir _da3_reqs _da3_marker _da3_hash _da3_fresh
