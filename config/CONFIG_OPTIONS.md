# MOAD Configuration Reference

Documentation for all parameters in `moad_config.json`.
Parameters are listed in the order they appear in the file.  
The config can be updated, saved, and reloaded during runtime through the CUI menu (main menu option 0). The moad_config.json file is also frequently updated automatically when changing object_name, or a scan is in progress (updating *prev_state*).

---

## Top-Level Parameters

| Parameter | Type | Description |
|---|---|---|
| `output_dir` | `string` | Root directory where all collected scan data is stored. Each object gets its own subfolder here. |
| `object_name` | `string` | Name of the current object being collected. Determines the subfolder created under `output_dir`. Updated automatically when set via the CUI. |
| `degree_inc` | `int` | Default turntable rotation step size in degrees per move when using `Full Scan`. Combined with `num_moves` determines the total angular coverage of a scan. |
| `num_moves` | `int` | Default for `Full Scan`.Total number of turntable moves per full scan. At 5°/move, 72 moves = 360° full rotation. |
| `thread_num` | `int` | Number of worker threads in the thread pool used for parallel sensor data collection. |

---

## `debug`

Controls logging verbosity and log file output.

| Parameter | Type | Description |
|---|---|---|
| `debug_verbosity` | `int` | Controls how much information is printed to the terminal. Higher values produce more output. `0` = errors only, `1` = threading events, `2` = general debug, `3` = verbose/low-level debug. |
| `log` | `bool` | Whether to write log output to a file. Log file is created at `output_dir/<object_name>/debug_log.txt` at startup. |

---

## `dslr`

Configuration for the Canon DSLR camera array.

| Parameter | Type | Description |
|---|---|---|
| `enable_collection` | `bool` | Whether to collect DSLR images during a scan. Set to `false` to skip DSLR setup/collection entirely (e.g. for RealSense-only scans). |
| `dslr_timeout_sec` | `int` | Timeout in seconds to wait for each camera to complete image download before reporting a failure. |
| `camera_ids` | `object` | Maps logical camera names (`CAMERA_1` through `CAMERA_5`) to their physical serial number strings. Used to identify and rename cameras at initialization. |
| `display_liveview_windows` | `bool` | Whether to display live view frames in OpenCV windows from within the CUI process. **Must be `false`** when an external live view script is running to avoid a race condition and seg fault. |
| `safe_take_picture` | `bool` | If `true`, waits for full image download confirmation before proceeding to the next capture. Slower but safer. If `false`, fires the shutter without waiting (faster, requires robust download handling). |

---

## `realsense`

Configuration for the Intel RealSense depth camera array.

| Parameter | Type | Description |
|---|---|---|
| `enable_collection` | `bool` | Whether to collect RealSense data during a scan. Set to `false` to skip RealSense setup/collection entirely. |
| `high_res` | `bool` | If `true`, streams at 1280×720. If `false`, streams at 640×480. |
| `collect_pointcloud` | `bool` | Whether to generate and save a point cloud (`.ply`) from each depth frame. |
| `collect_color` | `bool` | Whether to save the colour image (`_color.png`) from each frame. |
| `collect_depth` | `bool` | Whether to save the raw depth image (`_depth.png`) from each frame. |
| `normalize_depth_image` | `bool` | If `true`, normalises the depth image to 0–255 range before saving. Useful for visualisation but loses metric depth information — set to `false` for BOP evaluation. |
| `compute_normals` | `bool` | Whether to compute and include surface normals in the saved point cloud. Adds computation time proportional to `normals_threads`. |
| `normals_threads` | `int` | Number of CPU threads used for parallel normal estimation via OpenMP. |
| `raw_pointcloud` | `bool` | If `true`, saves the point cloud in the raw camera coordinate frame without applying any transforms or filters. Used for debugging. |
| `align_to_color` | `bool` | If `true`, aligns the depth frame to the colour camera's coordinate frame before processing. If set to `false`, aligns color data to the depth frame instead. For ease of calibration, should be left as `true`. |
| `realsense_timeout_sec` | `int` | Timeout in seconds to wait for a RealSense frame before reporting a failure. |
| `init_test_frames` | `int` | Number of warm-up frames to collect and discard at initialization. Allows auto-exposure to settle before real data collection begins. Set to `0` to skip. |
| `calibration_file` | `string` | Full path to the `realsense_cam_parameters.json` calibration file. Used by the RealSense handler to load extrinsic transforms for each camera. |
| `camera_ids` | `dict` | Maps logical RealSense camera names (`rs1` through `rs5`) to their physical serial number strings. Used to match connected devices to calibration entries. |

### `realsense.filter`

Point cloud filtering options applied after capture.

| Parameter | Type | Description |
|---|---|---|
| `sor.apply` | `bool` | Whether to apply Statistical Outlier Removal filtering to the point cloud. (Computationally heavy in high-res mode) |
| `sor.k` | `int` | Number of nearest neighbours used for mean distance estimation in SOR. |
| `sor.stddev` | `float` | Standard deviation multiplier threshold for SOR outlier rejection. Lower values are more aggressive. |
| `voxel.apply` | `bool` | Whether to apply voxel grid downsampling to the point cloud. |
| `voxel.leaf_size` | `float` | Voxel grid leaf size in metres. Larger values produce sparser clouds. |
| `xpass.apply` | `bool` | Whether to apply an X-axis passthrough filter to crop the point cloud. |
| `xpass.min` | `float` | Minimum X value (metres) to retain after passthrough filtering. |
| `xpass.max` | `float` | Maximum X value (metres) to retain after passthrough filtering. |
| `ypass.apply` | `bool` | Whether to apply a Y-axis passthrough filter. |
| `ypass.min` | `float` | Minimum Y value (metres) to retain. |
| `ypass.max` | `float` | Maximum Y value (metres) to retain. |
| `zpass.apply` | `bool` | Whether to apply a Z-axis passthrough filter. |
| `zpass.min` | `float` | Minimum Z value (metres) to retain. Typically set to `0` to remove points below the turntable surface. |
| `zpass.max` | `float` | Maximum Z value (metres) to retain. |

---

## `turntable_control`

Settings for serial communication with the turntable stepper motor controller.

| Parameter | Type | Description |
|---|---|---|
| `serial_com_port` | `string` | Serial port path for the turntable controller (e.g. `/dev/ttyACM0`). |
| `delay_after_move_ms` | `int` | Additional wait time in milliseconds after each turntable move completes before capturing data. Allows mechanical vibration to settle. |
| `command_timeout_s` | `int` | Maximum time in seconds to wait for a response from the turntable controller before reporting a timeout error. |

---

## `prev_state`

Persisted state from the last run. Updated automatically by the software after each move and scan. Used by the "scan from saved state" feature to resume an interrupted scan.

| Parameter | Type | Description |
|---|---|---|
| `current_move` | `int` | The last completed move index within the current scan (0-based). |
| `current_object` | `string` | Name of the object being collected at the time of the last save. |
| `current_pose` | `int` | ASCII integer code of the current pose letter (e.g. `97` = `'a'`, `98` = `'b'`). |
| `turntable_pos` | `int` | Turntable angle in degrees at the time of the last save. |

---

## `transform_generator`

Controls automatic generation of `transforms.json` for NeRF training after each scan. This is done by calling `scripts/transform_generator.py` from within the moad_cui program.

| Parameter | Type | Description |
|---|---|---|
| `enabled` | `bool` | Whether to automatically run `transform_generator.py` after a full scan completes. |
| `calibration_dir` | `string` | Path to the root calibration directory containing named calibration subfolders. |
| `calibration_mode` | `string` | Name of the calibration subfolder to use (e.g. `"55mm"` or `"55mm_joint"`). Must contain a `cam_parameters.json` file. |
| `force` | `bool` | If `true`, overwrites any existing `transforms.json` file for the current scan. |
| `visualize` | `bool` | If `true`, displays a 3D matplotlib visualisation of the generated camera transforms after generation. |

---

## `filecount_testing`

Controls automatic post-scan image downscaling via `scripts/filecount_test.py`, which creates the `images`, `images_2`, `images_4`, and `images_8` subfolders expected by NeRF training and annotation generation pipelines.

| Parameter | Type | Description |
|---|---|---|
| `enabled` | `bool` | Whether to run the filecount test script automatically after a full scan. |
| `check_all` | `bool` | If `true`, checks image counts across all object folders, not just the current one. |
| `check_single_object` | `bool` | If `true`, limits the check to the current object's folder only. |
| `count` | `bool` | Whether to count and report the number of images found. |
| `create` | `bool` | Whether to create downscaled image copies (`image`, `images_2`, `images_4`, `images_8`). |
| `delay` | `bool` | Whether to add a small delay between file operations for readability. |
| `manual_check` | `bool` | If `true`, pauses and prompts for user confirmation at each step. |

---

## `scene_replica`

Configuration for the Scene Replication pipeline — used for aligning virtual scenes to the physical rig and generating ground truth pose annotations.

| Parameter | Type | Description |
|---|---|---|
| `scene_root` | `string` | Root directory containing all scene subfolders (each subfolder holds an `.npz` scene layout file). |
| `scene_folder` | `string` | Name of the specific scene subfolder to load (e.g. `"batch1_007"`). |
| `scene_file` | `string` | Filename of the PyBullet scene layout file within `scene_folder` (e.g. `"scene_replica.npz"`). |
| `config_file` | `string` | Filename of the scene configuration JSON within `scene_replica_moad/config`. Specifies calibration offsets and rendering options. |

### `scene_replica.annotations`

Controls what types of ground truth annotations are generated when running `replica_generate_annotations.py` through the moad_cui interface.

| Parameter | Type | Description |
|---|---|---|
| `model_library` | `string` | Path to the object model library folder containing URDF/mesh files for all objects in the scene. |
| `pose` | `bool` | Whether to generate 6D pose annotations (object position and orientation per frame). |
| `masks` | `bool` | Whether to generate segmentation mask images. *(Not yet implemented.)* |
