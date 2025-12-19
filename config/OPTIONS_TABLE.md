## Less detailed, table formatted config options:

### Global Configuration

| Field                | Type    | Example                   | Description                                                           |
| -------------------- | ------- | ------------------------- | --------------------------------------------------------------------- |
| `debug`              | boolean | `true`                    | Displays extra timing and filter information during a scan            |
| `degree_inc`         | integer | `5`                       | Degrees the turntable moves per step during a full scan               |
| `log_debug`          | boolean | `false`                   | Saves detailed logs to `debug_log.txt` in the object folder           |
| `num_moves`          | integer | `72`                      | Number of steps in a scan (`degree_inc * num_moves = total rotation`) |
| `object_name`        | string  | `"atb3_thin-cable-mount"` | Name of object folder created inside `output_dir`                     |
| `output_dir`         | string  | `"../ALL_ITEMS"`          | Root directory where object folders are stored                        |
| `serial_com_port`    | string  | `"/dev/ttyACM0"`          | Serial port used to communicate with turntable Arduino            |
| `thread_num`         | integer | `5`                       | Number of DSLR threads (use caution—race conditions possible)         |
| `turntable_delay_ms` | integer | `10000`                   | Max wait time for turntable Arduino response                          |


### DSLR

| Field               | Type    | Example   | Description                                       |
| ------------------- | ------- | --------- | ------------------------------------------------- |
| `collect_dslr`      | boolean | `true`    | Enables DSLR image capture                        |
| `dslr_timeout_sec`  | integer | `20`      | Timeout waiting for DSLR image                    |
| `safe_take_picture` | boolean | `false`   | Uses slower, verbose EDSDK triggering             |
| `camera_ids`        | object  | see below | Mapping of logical camera names to serial numbers |

`camera_ids` 
| Key                     | Value            | Description                                                              |
| ----------------------- | ---------------- | ------------------------------------------------------------------------ |
| `CAMERA_1` … `CAMERA_5` | `"352074022019"` | Canon DSLR serial numbers (12 digits). Naming determines spatial mapping |



### Filecount Testing Script
| Field                 | Type    | Description                                                    |
| --------------------- | ------- | -------------------------------------------------------------- |
| `check_single_object` | boolean | Check only current object or all objects recursively           |
| `count`               | boolean | Count images in object folders                                 |
| `create`              | boolean | Create downscaled image folders (`images_2`, `images_4`, etc.) |
| `delay`               | boolean | Adds delay to output for readability                           |
| `enabled`             | boolean | Runs script automatically after scans                          |
| `manual_check`        | boolean | Prompts user per object for confirmation                       |



### State Saving
| Field           | Type            | Example | Description                           |
| --------------- | --------------- | ------- | ------------------------------------- |
| `current_move`  | integer         | `10`    | Current scan step                     |
| `current_pose`  | integer (ASCII) | `97`    | Current pose identifier               |
| `turntable_pos` | integer         | `315`   | Current turntable rotation in degrees |


### RealSense Configuration
| Field                   | Type    | Description                                           |
| ----------------------- | ------- | ----------------------------------------------------- |
| `align_to_color`        | boolean | Aligns depth to color frame                           |
| `collect_color`         | boolean | Collect RGB images                                    |
| `collect_depth`         | boolean | Collect depth images                                  |
| `collect_pointcloud`    | boolean | Collect pointcloud data                               |
| `collect_normals`       | boolean | Collect normal vectors                                |
| `collect_realsense`     | boolean | Master toggle for RealSense collection                |
| `high_res`              | boolean | Use 1280×720 instead of 640×480                       |
| `normalize_depth_image` | boolean | Normalize depth image to 0–255                        |
| `normals_threads`       | integer | Threads for normal calculation                        |
| `raw_pointcloud`        | boolean | Collect unfiltered pointclouds (used for calibration) |
| `realsense_timeout_sec` | integer | Timeout waiting for RealSense data                    |

### RealSense Filters
#### Statistical Outlier Removal (sor)
| Field    | Type    | Description                  |
| -------- | ------- | ---------------------------- |
| `apply`  | boolean | Enable SOR filter            |
| `k`      | integer | Number of nearest neighbors  |
| `stddev` | integer | Standard deviation threshold |
#### Voxel Grid (Deprecated) (voxel)
| Field       | Type    | Description             |
| ----------- | ------- | ----------------------- |
| `apply`     | boolean | Enable voxel filter     |
| `leaf_size` | float   | Downsampling resolution |
#### Pass Filters (xpass, ypass, zpass)
| Field   | Type    | Description          |
| ------- | ------- | -------------------- |
| `apply` | boolean | Enable axis cropping |
| `min`   | float   | Minimum bound        |
| `max`   | float   | Maximum bound        |
#### RealSense Device Info (rs_info)
| Field                          | Type             | Description                          |
| ------------------------------ | ---------------- | ------------------------------------ |
| `rs1` … `rs5.serial`           | string           | RealSense device serial number       |
| `rs1` … `rs5.transform_matrix` | 4×4 float matrix | Transform from camera to world space |
#### Transform Generator (transform_generator)
| Field              | Type    | Example                                    | Description                              |
| ------------------ | ------- | ------------------------------------------ | ---------------------------------------- |
| `calibration_dir`  | string  | `"../calibration"`                         | Directory containing calibration folders |
| `calibration_mode` | string  | `"55mm"`                                   | Active camera calibration                |
| `force`            | boolean | `false`  |Skip confirmation prompt                   |                                          |
| `visualize`        | boolean | `true`  | Generate 3D visualization of camera layout |                                          |




