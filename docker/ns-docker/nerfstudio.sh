# https://docs.nerf.studio/quickstart/installation.html#using-an-interactive-container 
# Run with SUDO if you have GPU issues

# Ensures the script exits immediately on any unexpected error rather than silently continuing
set -euo pipefail

# ─── Path Configuration ───────────────────────────────────────────────────────
# Override any of these with environment variables before running, e.g.:
#   WORKSPACE_DIR=/data/myproject ./nerfstudio.sh
#
# Or pass the workspace directory as the first argument:
#   ./nerfstudio.sh /data/myproject
 
# DEFAULTS -- CHANGE THESE AS NEEDED TO RUN WITH NO ARGUMENTS
DEFAULT_WORKSPACE_DIR="/home/csrobot/MOAD_DATA"
DEFAULT_CACHE_DIR="${HOME}/.cache"
 

WORKSPACE_DIR="${WORKSPACE_DIR:-${1:-$DEFAULT_WORKSPACE_DIR}}"
CACHE_DIR="${CACHE_DIR:-$DEFAULT_CACHE_DIR}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="${SCRIPTS_DIR:-${SCRIPT_DIR}/scripts}"
INIT_SCRIPT="${INIT_SCRIPT:-/workspace/scripts/init.sh}"
CONTAINER_NAME="${CONTAINER_NAME:-nerfstudio-container}"
SHM_SIZE="${SHM_SIZE:-32gb}"
PORT="${PORT:-7007}"
IMAGE="${IMAGE:-ghcr.io/nerfstudio-project/nerfstudio}"

# ─── Validation ───────────────────────────────────────────────────────────────
# Checks to make sure all required filepaths exist.
errors=0
 
check_dir() {
    local path="$1" label="$2"
    if [[ ! -d "$path" ]]; then
        echo "[ERROR] $label not found: $path" >&2
        (( errors++ )) || true
    fi
}
 
check_dir "$WORKSPACE_DIR" "Workspace directory"
check_dir "$CACHE_DIR"     "Cache directory"
check_dir "$SCRIPTS_DIR"   "Scripts directory"
 
if (( errors > 0 )); then
    echo ""
    echo "Fix the paths above or override them with environment variables."
    exit 1
fi

# ─── Launch ───────────────────────────────────────────────────────────────────
# Start the Docker container
 
echo "Starting NerfStudio container..."
echo "  Workspace : $WORKSPACE_DIR"
echo "  Cache     : $CACHE_DIR"
echo "  Scripts   : $SCRIPTS_DIR"
echo "  Port      : $PORT"
echo ""
 
docker run --gpus all \
    --name "$CONTAINER_NAME" \
    --hostname nerfstudio \
    --user root  \
    -v "$WORKSPACE_DIR":/workspace/ \
    -v "$CACHE_DIR":/home/user/.cache/ \
    -v "$SCRIPTS_DIR":/workspace/scripts/ \
    -p "$PORT":"$PORT" \
    --rm \
    -it \
    --shm-size="$SHM_SIZE" \
    "$IMAGE" "$INIT_SCRIPT"

# default_mount_dir="/home/csrobot/MOAD_DATA"
# # Get arguments or use default values
# mount_dir="${1:-$default_mount_dir}"
# echo "Mounting $mount_dir..."



# docker run --gpus all \
# 	    --name nerfstudio-container \
#         --hostname nerfstudio \
#             --user root \
#             -v $mount_dir:/workspace/ \
#             -v /home/csrobot/.cache/:/home/user/.cache/ \
#             -v /home/csrobot/moad_control/moad_cui/docker/ns-docker/scripts:/workspace/scripts/ \
#             -p 7007:7007 \
#             --rm \
#             -it \
#             --shm-size=32gb \
#             ghcr.io/nerfstudio-project/nerfstudio "/workspace/scripts/init.sh"
            
