## MOAD Data Collection JSON Config

The fields within the actual file are currently organized alphabetically so to reflect this, the following descriptions are also as such. 

*Note 1: We are aware this makes things a bit confusing because ideally, priority fields would be at the very top of the readme and the json file. We believe that the json parsing library we use SHOULD be able to handle this and the reason for mixup might be due to some caching inconsistencies with our IDE or version control.*

[<https://stackoverflow.com/questions/72534844/removing-alphebatical-reordering-of-key-value-pairs-in-nlohmann-json>]
[<https://github.com/nlohmann/json/discussions/4568>]

*Note 2: If any changes are made at runtime (ex., via the `Set Object Name` in the Control Menu), the config is fully rewritten (though only the necessary field will change) and saved.*
<br>

___
(config starts here)
<br>



- `"debug"` : `[true/false]` => Primarily displays extra timing and filter information during a scan. 

- `"degree_inc"` : `[integer, ex=5]` => This is the degree the turntable will move at each step of a full scan. This is only used in a full scan.

<br>


**[DSLR]:** The following field covers information used by the software to configure the Canon DSLR cameras and their captures:

- `"dslr"` : `{`
  - `"camera_ids"` : `{`
      - `"[string of a CAMERA]"` : `"[string of the serial id number]"`
      - `"CAMERA_2"` : `"352074022019"` (example)
    
  `}`
    
    => **This is where you should put the Canon camera serial numbers**. On the Canon EOS Rebel SL3 cameras, these are found on the bottom of the device itself as well as on the box. They consist of 12 digits. The software is also setup so that `CAMERA_5` refers to the camera directly above the center of the turntable and `CAMERA_1` refers to the camera closest to being on the same horizontal plane as the turntable. This naming is important because otherwise the OS would designate the mapping based on the order the devices were physically plugged in.


  - `"collect_dslr"` : `[true/false]` => Used for toggling whether or not to take DSLR images during any of the scans. Useful if you only needed realsense data for example.

  - `"dslr_timeout_sec":` : `[integer, ex=20]` => Sets a wait time for the scans, tells the software to timeout if a DSLR image is not received in this time. 

  - `"safe_take_picture"` : `[true/false]` => Toggles a slower, more deliberate version of the function communicating with the EDSDK to trigger the DSLR cameras. This is useful for if any of the cameras are experiencing mechanical issues and a user needs to walk through the process slowly. Will also show a much more detailed CLI output including EDSDK feedback and thread numbers.
  
`}`
<br>
<br>

**[Filecount Testing Script]**: The following field covers settings meant for the python script that checks if the appropriate number of images exist after taking a full scan. More info and more settings are included in both the comments at the top of the `filecount_test.py` file itself AND the .md file in `scripts/`. Reminder that these are also alphabetized, so `enabled` is not at the top even though this is likely the field that will be changed the most.

- `"filecount_testing"` : `{`
  - `"check_single_object"` : `[true/false]` => Toggles if the user would like to only check the current object or recursively iterate over ALL item folders at the end of a scan and check each for the appropriate number of images inside each respective DSLR folder.
  - `"count"` : `[true/false]` => Toggles if the user would like to count the images in the designated object folder(s). Useful on `false` if the user would like to only use the `create` functionality.
  - `"create"` : `[true/false]` => Toggles subfolder creation, specifically, if set to `true`, will create `images`, `images_2`, `images_4`, `images_8` inside the `pose/` subdirectory of the object folder. These folders contain copies of the DSLR images, but downscaled by the factor at the end of the folder names (ex. `images_4` contains copies of the DSLR images downscaled by a factor of `4`). This functionality is meant to speedup Nerfacto reconstruction training by setting the files up in the format that Nerfacto requires for a run. 
  - `"delay"` : `[true/false]` => Toggles a hardcoded delay for the printouts from counting, creating, and downscaling during this script execution. This is useful for allowing the user to read which object doesnt have the correct number of DSLR images for example.
  - `"enabled"` : `[true/false]` => Toggles this script running at the end of scans. 
  - `"manual_check"` : `[true/false]` => Toggles a CLI prompt at the end of each folder parse. Useful for allowing the user to visually confirm which object is being looked at if checking multiple objects at a time.

`}`
<br>
<br>

- `"log_debug"` : `[true/false]` => Toggles detailed logging that gets saved into the file `debug_log.txt` which gets saved into the object folder. This log include things like the date and time of when the program runs as well as when it ends, and information during the scaning process similar to the outputs shown in the CLI. 

*Note: this is incomplete, plans for this included showing thread usages during a scan. The global class `DebugUtils` exists in the codebase and is meant to be called similar to a `std::cout` wherever something needs to be logged, ex: `DebugUtils::logDebug("Current Object Name: " + object_info["Object Name"]);`*.

- `"num_moves"` : `[integer, ex=72]` => Refers to the number of steps that occur during a scan. Should be considered when changing the `degree_inc` field. Ex: for a full 360 degree scan with a degree increment of `5`, the number of moves should be set to `72`. In other words, *"Move the turntable `5` degrees for `72` steps for a total of `5 * 72 = 360` degrees during a scan."*

- `object_name` : `[string, name of object to scan ex="atb3_thin-cable-mount"]` This name will be used to create a folder inside the given `output_dir` to store all the scan data. (Inside will be pose and DSLR folders).

- `"output_dir"` : `[string, directory path ex="home/uml_nerve/data-mount/ALL_ITEMS"]"` => Location where object *folders* will be stored. Recommended setup is something like `../ALL_ITEMS/atb1_timing-belt/pose-a...`


**[State Saving]**: The following field covers information changed at each step of a scan that are useful if something fails partway through (for example, mechanical shutter failure on the DSLR cameras). If "Scan from Save State" option is selected within the Control Menu option, these subfields will be read and used to run a scan starting at the *previous state* meaning whatever current data that was taken at this step in the process will be overwritten with new data of the exact same locations/rotation of the turntable before moving to the next step. Workflow example: 
```text
  [scanning on move `44/72`]
  > DSLR camera breaks
  > user closes program and fixes camera
  > user restarts software
  > user selects "scan from save state" 
  [scanning on move `44/72`]
  [scanning on move `45/72`] 
  ...
```

- `"prev_state"` : `{`
  - `"current_move"` : `[integer, ex=10]` => Rewritten at each step of a full scan, counts up to the value at the `num_moves` field.
  - `"current_pose"` : `[integer, ASCII of a char, ex=97]` => Rewritten at each step of a full scan, also useful for the user if software issues cause the pose to be misread, since the config is distinct from the internal pose check.
  - `"turntable_pos"` : `[integer, ex=315]` => Rewritten at each step of a full scan, refers to the current degree measurement of the turntable. This counts up by `degree_inc` and ends at `degree_inc * num_moves` inclusive.

`}`

<br>
<br>

**[RealSense Cameras]**: The following field covers necessary settings for the RealSense cameras that allow the software to recognize the devices as well as take accurate data. Some of the fields may seem overlapping but the goal is to allow users full control over what's being taken. A possible usecase for different toggle configurations is if the user wants to only take Realsense depth images. Another usecase would be a user wanting to only take DSLR camera data and toggling Realsense collection off complete.y

- `"realsense"` : `{`
  - `"align_to_color"` : `[true/false]` => Toggles whether to take color images or depth images
  - `"collect_color"` : `[true/false]` => Toggles whether or not to take color images at all. 
  - `"collect_depth"` : `[true/false]` => Toggles whether or not to take depth images at all. 
  - `"collect_pointcloud"` : `[true/false]` => Toggles whether or not to take pointcloud data. 
  - `"collect_realsense"` : `[true/false]` => Toggles Realsense collection.
  - `"collect_normals"` : `[true/false]` => Toggles collection of information for normal vector calculations of surfaces by the Realsense cameras.
  - `"collect_realsense"` : `[true/false]` => Toggles Realsense collection.
  
  - `"filter"` : `{`
  
    "Statistical Outlier Removal", calculates average of k-nearest neighbors, removes points outside of average + the standard deviation.
    - `"sor"` : `{`
      - `apply` : `[true/false]`
      - `k` : `[integer, ex=6]`
      - `stddev` : `[integer, ex=1]`

    `}`
    <br>

    Deprecated, meant for downsampling points with their centroid. [<https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html#details>]

    - `"voxel"` : `{`
      - `apply` : `[true/false]`
      - `leaf_size` : `[floating point, ex=0.01]`

    `}`
    <br>

    The following filters control cropping of the pointcloud along their respective axes. If misalignment occurs, whether due to the cameras phyisically moving or if possible transformation matrix inaccuracy, the Realsense library might spit out errors that cause the program to crash. To check this, enable `raw_pointcloud`.
    - `"[x/y/z]pass"` : `{`
      - `apply` : `[true/false]`
      - `max` : `[floating point, ex=0.3]`
      - `min` : `[floating point, ex=-0.3]`

    `}`

  `}`



  - `"high_res"` : `[true/false]` => Toggles collected image sizes to be 1280x720 or 640x480. Performance impact is almost negligible if using the higher resolution since the amount of time this takes is mostly determined by how fast the DSLR cameras take a picture and how quickly the turntable moves.
  - `"normalize_depth_image"` : `[true/false]` => An OpenCV setting meant for the depth images to enhance contrast. Normalizes image data between 0 and 255.
  - `"normals_threads"` : `[integer, ex=1]` => Refers to the number of threads used for calculating the normals. Varying impact to performance, best kept at `1` if the pointcloud size is not that large.
  - `"raw_pointcloud"` : `[true/false]` => Toggles pointcloud collection with no added filters. **This is used in the calibration process** that generates the transform matrices relating the realworld positioning to the pointclouds.
  - `"realsense_timeout_sec"` : `[integer, ex=9]` => Sets the maximum wait time for Realsense data. If the program keeps hitting this wait time and crashing, try checking physical connection points between the Realsense cameras and the motherboard. Also try unplugging the cameras, waiting for a bit, the plugging them back in (sometimes they get stuck in a 'waiting to send' state and never actually finishing the send).

  - `"rs_info"` : `{`
  
    - `rs[1...5]` : `{`
      - `serial` : `[string, 12 digits]`
      - `transform_matrix` : `[4 lists of 4 floating point values, creates a 4x4 matrix]`

    `}`

  `}`

`}`

<br>
<br>

  - `"serial_com_port"` : `[string, ex="/dev/ttyACM0"]` => Port that the DSLR cameras connect to/communicate with.

  - `"thread_num"` : `[integer, ex=5]` => Sets the number of threads to use for the DSLR cameras. BE VERY CAREFUL WHEN CHANGING THIS. Thread race conditions between the software and EDSDK have lead to some errors and possible mechanical shutter failures.




**[Transform Matrix Generation Script]**: The following fields set up the directory and a few arguments made for the `transform_generator.py` script in the `scripts/` directory. After each scan, this script runs and generates a matrix used by Nerfacto to reconstruct a pointcloud based on the current camera settings. It is important to make sure these line up because otherwise you will have to retrain the nerf for a positive result and each reconstruction can take over ~30 minutes depending on hardware.  


  - `"transform_generator"` : `{`

    - `"calibration_dir"` : `[string, ex="../calibration"]` => Path to the seperate transform matrices for each DSLR calibration. These should be located `../calibration/55mm` for example. The calibrations themselves are found by following the steps in the calibration document for DSLR cameras.

    - `"calibration_mode"` : `[string, ex="55mm"]` => This sets the current calibration of the cameras. There should be an associated folder with the same name containing transform matrices. This is an important field to set because reconstructions with the assumption that the cameras are set to 18mm even though they were set to 55mm do not come out very well.

    - `"force"` : `[true/false]` => Bypasses a prompt check before generating transforms.

    - `"visualize"` : `[true/false]` => Tells the script to generate a 3d representation of the DSLR camnera locations in space. 


  `}`


  - `"turntable_delay_ms"` : `[integer, ex=10000]` => Sets the wait time for the turntable arduino to respond. Note: for bug-checking, make sure to double check arduino code as well.









