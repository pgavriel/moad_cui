import os
import sys
import time
import csv
import subprocess
import glob
from datetime import datetime

# Path to blender executable
# BLENDER_PATH = "/home/csrobot/software/blender-4.3.2-linux-x64/blender"
BLENDER_PATH = "blender"

# Path to the blender python script we already wrote
CONVERSION_SCRIPT = "/home/csrobot/moad_cui/scripts/blender_model_conversion.py"


def find_meshes(root):
    """
    Recursively search for subfolders named 'fused'.
    Inside each fused folder, look for *_mesh.ply.
    If more than one, prompt the user which to use.
    """
    candidate_meshes = []
    for dirpath, dirnames, filenames in os.walk(root):
        if os.path.basename(dirpath) == "fused":
            matches = glob.glob(os.path.join(dirpath, "*_mesh.ply"))
            if not matches:
                continue
            elif len(matches) == 1:
                candidate_meshes.append(matches[0])
            else:
                print(f"\nFound multiple *_mesh.ply files in {dirpath}:")
                for i, m in enumerate(matches, 1):
                    print(f"  {i}: {os.path.basename(m)}")
                choice = input("Select one to use (number, or 's' to skip): ").strip()
                if choice.isdigit() and 1 <= int(choice) <= len(matches):
                    candidate_meshes.append(matches[int(choice) - 1])
                else:
                    print("Skipping this folder.")
    return candidate_meshes


def already_processed(mesh_path):
    """Check if the blender script has probably been run already."""
    folder = os.path.dirname(mesh_path)
    blend_file = os.path.join(folder, "blend", "fused_model.blend")
    tex_file = os.path.join(folder, "baked_texture.png")
    usd_dir = os.path.join(folder, "usd")
    obj_dir = os.path.join(folder, "obj")
    return any([
        os.path.exists(blend_file),
        os.path.exists(tex_file),
        os.path.isdir(usd_dir),
        os.path.isdir(obj_dir),
    ])


def run_blender(mesh_folder):
    """Run Blender in background mode on one folder."""
    cmd = [
        BLENDER_PATH,
        "--background",
        "--python", CONVERSION_SCRIPT,
        "--", mesh_folder
    ]
    print(f"\n🚀 Running Blender on {mesh_folder}")
    start = time.time()
    result = subprocess.run(cmd)
    elapsed = time.time() - start
    success = (result.returncode == 0)
    if success:
        print(f"✅ Completed in {elapsed:.2f} seconds")
    else:
        print(f"❌ Failed after {elapsed:.2f} seconds (exit code {result.returncode})")
    return elapsed, success


def main(search_root):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    meshes = find_meshes(search_root)
    print(f"Initial Search: {len(meshes)} meshes found...")
    

    if not meshes:
        print("No *_mesh.ply files found inside fused/ folders.")
        sys.exit(0)

    print("\nFound candidate meshes:")
    for m in meshes:
        status = "[already processed]" if already_processed(m) else ""
        print(f"  - {m} {status}")

    proceed = input("\nProceed with conversion? (y/n) ").strip().lower()
    if proceed != "y":
        print("Aborted.")
        sys.exit(0)

    # Results for CSV log
    results = []

    for mesh in meshes:
        mesh_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        folder = os.path.dirname(mesh)
        model_name = os.path.basename(mesh)

        if already_processed(mesh):
            # confirm = input(f"\n⚠️ {folder} looks already processed. Continue anyway? (y/n) ").strip().lower()
            # if confirm != "y":
            print("Skipping.")
            results.append((mesh_timestamp, model_name, 0.0, "skipped"))
            continue

        elapsed, success = run_blender(mesh)
        results.append((mesh_timestamp, model_name, elapsed, "success" if success else "fail"))

    # Write CSV summary log
    log_name = f"{timestamp}_conversion_summary.csv"
    log_path = os.path.join(search_root, log_name)
    with open(log_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Timestamp","Model", "TimeSeconds", "Status"])
        writer.writerows(results)

    print(f"\n📄 Summary written to {log_path}")



if __name__ == "__main__":
    DEFAULT_ROOT = "/home/csrobot/data-mount/MOAD_V2"
    if len(sys.argv) < 2:
        print("Usage: python batch_convert.py /path/to/root")
        print(f"Using default root: {DEFAULT_ROOT}")
        search_root = os.path.abspath(DEFAULT_ROOT)
    else:
        search_root = os.path.abspath(sys.argv[1])

    main(search_root)
