# MOAD CUI  

#### Official MOAD Website: [<https://www.robot-manipulation.org/nist-moad>]

## Quick Start
1. Clone this repo and simple_serial_port into a folder with EDSDK (For Linux v13.19.10) from Canon's website
2. Run bash script to auto-install packages.
3. Configure moad_config.json
4. Run:
```
cd moad_cui/
cmake --build build
sudo ./build/MultiCamCui
```


## Table of contents

  * [Quick Start](#Quick-Start)
  * [Pre-Requirements and Suggestions](#pre-requirements-and-suggestions)
  * [Requirements](#Requirements)
    * [Third Party Software](#Third-Party-Software)
    * [System Libraries and Packages](#System-libraries-and-packages)
  * [Json Config Setup](#Setting-up-the-JSON-Config)
  * [Transform Matrices](#Transformation-Matrices)
    * [Links to calibration tutorial docs](####Tutorial-Docs)
  * [Build and Run](#Build-and-run-the-software)
  * [Using the program](#Using-the-program)
    * [Control Menu](#Control-Menu)



Below is slightly more in-depth setup.

## Pre-Requirements and Suggestions
To minimize dependency pathing, I suggest cloning this repo into its own directory that we can place some of the required third party software alongside. 

**Target Directory Structure:**
```
moadrig_NERVE_location/
├── EDSDK/
├── moad_cui/ # <-- THIS REPO
│   ...
└── simple_serial_port/
    ├── linux/
    ...
```
The CMake file is setup for this structure.

...

**<u>Also, to avoid any potential auto-settings conflicts, I suggest unplugging all DSLR and Realsense cameras before installing requirements.</u>**

## Requirements:

#### This repo
Latest code is in branch `dev-linux`. Navigate into overall project folder (in the above directory tree, this is named `moadrig_NERVE_location/` as an example) and run:
```
git clone -b dev-linux https://github.com/pgavriel/moad_cui.git
```

#### Third Party Software
- **Canon `EDSDK`:**

Install instructions found here: [<https://developercommunity.usa.canon.com/s/article/How-do-I-apply-for-a-development-tool-SDK-API-Etc>], but also included below:

1. Register for an account, uses email verification, should end up on Canon home page.
2. Click "`SDK | API | DOWNLOADS`" on the option bar (should be one option as of 12/9/25).
3. Click "`EOS & POWERSHOT CAMERAS`".
4. Type in and submit information into their form that requests access to the SDK (this seems automated and for data collection, it will automatically allow you to download and use the software).
5. After submitting, click "`---->  Canon Developer Community Downloads  <---`"
6. This takes you back to the "`SDK | API | DOWNLOADS`" result screen, click on "`EOS & POWERSHOT CAMERAS`" again. 
7. The versions are set to Windows, click on the "`EDSDK FOR LINUX`" option.
8. Fill in form info again and submit, should be automated.
9. Click "`---->  Canon Developer Community Downloads  <---`" again.
10. Click "`EOS & POWERSHOT CAMERAS`" and "`EDSDK FOR LINUX`" again.
11. As of 12/9/25, the moadrig uses "`EDSDK v13.19.10 for Linux"`. Download by clicking on this version.
12. Extract into the overall project directory, make sure the repo structure matches the above visual, where `EDSDK/` has direct `Header` and `Library` subdirectories.



- **`simple_serial_port`:**

Original repo is from  [<https://github.com/dmicha16/simple_serial_port>] though it was made for windows. I have added a linux-capable version in a forked repo at [<https://github.com/grahamstelzer/simple_serial_port.git>].
1. `cd` into the overall project folder if you are not already there
2. run:
```
git clone https://github.com/grahamstelzer/simple_serial_port.git
```
*(note: the installation script in the next step will also attempt these two steps if the user allows it)*

<br>

#### System libraries and packages

All these have seperate install instructions, but for ease of usage I have consolidated the commands into a single bash script.

1. navigate into `moad_cui/`
```
cd moad_cui/
```
2. run install script

```
./install_requirements.sh
```

However, for the more skeptical and cautious individuals that like to make life slightly more difficult than necessary, included below are the official documentation and install readmes provided by each package:

- RealSense SDK: [<https://github.com/realsenseai/librealsense/blob/master/doc/installation.md>]
- OpenCV : [<https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html>] (links to source installation)
- PCL: [<https://pointclouds.org/downloads/>] (scroll down to linux version)
- Nholmann Json for Modern C++: [<https://github.com/nlohmann/json>]
- libudev: [<https://man7.org/linux/man-pages/man3/libudev.3.html>] (this should already be on a linux system but in case not, a line is included in the bash script)
  

#### After installing all these, you should be able to build with cmake and run the executable (feel free to test this), but we need to configure the moad_config.json file first or else the cameras will not be recognized and nothing will be saved correctly.




## Ok now plug the cameras in...

...but make sure to write down their serial numbers first. These are found all the cameras themselves OR on their packaging. They consist of 12 numbers. They need to be placed in the moad_config.json file BUT INSTRUCTIONS FOR THIS ARE INCLUDED IN THE NEXT STEP AND IN [config/CONFIG_OPTIONS.md](config/CONFIG_OPTIONS.md).

Mount the DSLR and Realsense cameras in the desired configuration and plug them into the PC.

**Ideally, connect them directly to the motherboard** (versus via usb hubs, wire extensions, etc). We have encountered many issues that have boiled down to slightly delayed or deteriorating connection between the software and hardware so this step is done in prevention of that.







## Setting up the JSON Config  

*Quick setup: type serial id numbers into the correct locations in moad_config.json, set the output directories, double check settings are configured correctly.*

In order to reduce the amount of rebuilding code during usage, an effort was made to paramaterize code using the text file `moad_config.json`, which allows the user to specify things like output directory, which data streams to collect, pointcloud filter options, COM port for motor control, delays, and timeouts.

In-depth explanations for each config value is included in [config/CONFIG_OPTIONS.md](config/CONFIG_OPTIONS.md). However, for starting up, I've included a few of the more important ones below:

- `dslr.camera_ids.{CAMERA_1 ... CAMERA_5}`: these are the camera serial ID numbers found on the bottom label of the physical device itself. You must enter them manually. This provides each camera with a consistent deterministic name based on serial number, such that terminal outputs and output files will reflect the camera with the specified serial number (i.e. CAMERA_1 -> cam1). For our rig, <u>we set CAMERA_5 to be directly over the object on the vertical axis and CAMERA_1 as the camera closest to side-on view (horizontal axis).</u>

- `object_name`: Meant to refer to the object currently being scanned, this will also affect the folder name of the directory that the images are output to. Example: `"bar-4mm"`. Note that this sets the initial object name when the program starts, but can be modified during runtime to scan new objects (see Control Menu).
- `output_dir`: Refers to the general output directory of where the named object folders will be. Example: `atb1`
- `realsense.rs_info.rs1 ... rs5`: This stores vital information that allows the realsense cameras to work and capture correctly oriented pointclouds.
  - `serial`: This number is 12 digits long and found on the Realsense cameras themselves or the box they are packaged in (ours were under the field S/N) and is used by the realsense SDK to send the commands to capture and save a pointcloud. 
  - `transform_matrix`: The matrix used to align the pointclouds with their real life orientations. **Instructions on how to get these are included in the next section with a short explanation.**
- `transform_generator`: These fields are meant for Python script that generates a transform matrix that aligns the DSLR cameras with their real life orientations. This is automatically run at the end of full scans.
  - `calibration_dir`: Location of the DSLR `alignment_tf.txt` and `transforms.json` in their respective zoom calibration folders (`55mm/` , `18mm/`). Example: `"../calibration"`
  - `calibration_mode`: Zoom level of the DSLR cameras. Example: `"55mm"`









## Transformation Matrices

*> something to note is that currently, the <u>DSLR transforms are stored in a directory called `calibration`</u> based on their zoom level (55mm, 18mm). The <u>Realsense transforms are stored in `moad_config.json`</u> and were previously in the same directory but were moved to cut down on some of the overhead in the Realsense code.*


#### Quick Guide

In order for the Realsense pointclouds to align correctly and for the Nerfacto reconstruction network to function correctly on the DSLR images, the algorithms need information about the positioning of the cameras relative to each other and the turntable. This is accomplished for both types of cameras with the following methods (abstracted; meant for you to keep in mind while reading the instruction docs):

- DSLR: 
```
for each camera:
  take a picture

using colmap, 
  1. get dense pointcloud
  2. get individual camera transforms
  3. get camera intrinsics

using cloudcompare:
  1. manually align
  2. copy and save transformation matrices
```
- Realsense:
```
for each camera:
  take a raw pointcloud (unfiltered)

using cloudcompare:
  1. manually align
  2. copy and save transformation matrices
```

#### Tutorial Docs
Latest DSLR calibration instructions: [<https://docs.google.com/document/d/1jnYCCUdmyryIeSsuyZUs1_pchJEBfH6QofjJOnQRerQ/edit?usp=sharing>]

Realsense calibration: [<https://docs.google.com/document/d/18nGBK1lqk_UkK3qeKK1O4t33_pRx8Umx7rD395rmtt8/edit?usp=sharing>]








## Build and Run the software
1. `cd` into `moad_cui/`
2. Create build directory (only necessary on fresh install)
```
cmake -S . -B build
```
3. Run
```
cmake --build build
```

To build everything in `build/` directory.

4. Run
```
sudo ./build/MultiCamCui
```

Note: in our case, we needed root perms for the file creation that occurs during scanning.








## Using the program
At this point, everything is probably setup correctly. After running, should see the control menu come up. However, before this, you can read the cli outputs. They are set up for inline coloring.

<mark style="background-color: aquamarine; color: black;">Aquamarine refers to any Realsense information</mark>

<mark style="background-color: magenta; color: black;">Magenta refers to any DSLR information</mark>

<mark style="background-color: grey; color: black;">Grey refers to any serial port information</mark>

<mark style="background-color: black; color: yellow;">Yellow text refers to any file/directory creation information</mark> (specifically just yellow text, black highlight is for readability)

Any other text outputs usually refer to outputs from the MOADCui.cpp where main() occurs.



### Control Menu  
  The control menu is the following:  

  ``` txt
    MOAD - CLI Menu
    ('r' to return)

    +---------------+--------------+
    | Object Name   : test_name    |
    | Pose          : u            |
    | Turntable Pos : 200          |
    +---------------+--------------+

    0  Reload Config
    1  Full Scan
    2  Custom Scan
    3  Collect Single Data
    4  Set Object Name
    5  Set Pose
    6  Camera Calibration...
    7  Camera Options...
    8  Turntable Options...
    9  Live View...
  ```


  
  Type the number of the option you want to select and press enter. The following is a description of the operation for each input number.

- **'r'**  Always acts as [return to previous menu] unless on the initial menu where it acts as [safely exit program and shut down sensors].

- **0  Reload Config**: Reload the moad_config JSON file, used when you want to change some config values during runtime, for example a filter that you no longer want to apply or changing the angle of the scan.

- **1  Full Scan**: It does an object scan using the object name, pose, and scan configuration stated in the moad_config.json.

- **2  Custom Scan**: It does an object scan using the existing object name and pose, however the degree and number of steps will be prompted by the user at runtime.

- **3  Collect Single Data**: It scans a single batch of data from the current turntable position and active object name.

- **4  Set Object Name**: It it sets the name of the object, if the object does not exist, then it will create the respective folders and file needed in order to scan. It calculates the current pose based on the last available pose in the object file.

- **5  Set Pose**: Manually set the pose of the object file.

- **6  Camera Calibration...**: A submenu where all the camera focusing options reside.

*Quick notes: should NOT use these options if currently running the Live View.*

``` txt
Calibration Menu
('r' to return)

+---------------+-----------------+
| Object Name   : testing_speed_3 |
| Pose          : u               |
| Turntable Pos : 0               |
+---------------+-----------------+



 1  Press Halfway    
 2  Press Completely 
 3  Press Off
 ```

- **6.1  Press Halfway**: Equivalent to physically pressing shoot buttom halfway in the camera, used to reset focus of the camera.

- **6.2  Press Completely**: Equivalent to physically pressing the button completely in the camera, it takes a photo and saves it in memory.

- **6.3  Press Off**: Equivalent to letting go of the button on the camera, it resets the state of the camera to ready.

- **7  Camera Options...**: A submenu where you can find and change some of the configurations values of the DSLR cameras, such as AV, TV and ISO.

Helpful resource: [<https://www.canon.com.au/get-inspired/understanding-camera-modes-and-settings-for-beginners>] 

*Quick note 1: The menus for each setting are laid out where the number on the left is the option to type into the console, and the numbers/characters on the right refer to the setting being applied to the camera. Ex: under the TV menu, if we want the shutter to stay open for 3.2 seconds, we would locate this in the menu: `11: (3''2)`, type `11` into the console, and press enter.*

*Quick note 2: Open and turn on the Live View while change these settings.*

``` txt
Camera Options
('r' to return)

+---------------+-----------------+
| Object Name   : testing_speed_3 |
| Pose          : u               |
| Turntable Pos : 0               |
+---------------+-----------------+



 1  TV
 2  AV
 3  ISO
 4  White Balance
 5  Drive Mode
 6  AE Mode
 7  Change Camera 
 ```

- **7.1  TV**: Set the TV value to one of the values given by a table, The TV is in charge of setting the shutter speed of the camera, the lower the value the sharper the image will look like at the cost of shooting speed.
- **7.2  AV**: Set the AV value to one of the values given by a table. The AV is in charge stating how much light the camera can recieve during the shooting.
- **7.3  ISO**: Set the ISO value to one of the given values by a table. The ISO is in charge of the light sensitivity of the camera.
- **7.4  White Balance**: Set the White Balance value to one of the values given by a table. The White Balance is a post processing option that is in charge of setting the ambience lighting depending of the intensity and color of the light. Recommendation, use the one that keeps the image true to the real life color, avoid using auto.
- **7.5  Drive Mode**: Set the Drive Mode value to one of the given by a table. Used to change the behavior of the shutter button. **Recommendation: Set all cameras to single shooting, do NOT use silent shooting. We suspect the extra machinery for this option caused the shutters to break over the massive number of photos taken**
- **7.6  AE Mode**: (DEPRICATED)
- **7.7  Change Camera**: Changes the camera that the options will be applied to, by default those option will be applied to all cameras, however you can set which individual camera you want to change the settings of.

- **8  Turntable Options (has a few issues after linux port)**: A submenu where you can find options related to the turntable itself.

``` txt
Turntable Options
('r' to return)

+---------------+-----------------+
| Object Name   : testing_speed_3 |
| Pose          : u               |
| Turntable Pos : 0               |
+---------------+-----------------+



 1  Turntable Control  
 2  Turntable Position`
 ```

- **8.1  Turntable Control**: Enter the degrees to move. Ex: 90 degrees should move the turntable 1/4th around (since out of 360 degrees). *Currently the wait time is broken, recommended not moving anything more than 10 degrees at a time. As of 12/17, turntable degrees cannot be something like 360 since this overflows the arduino code (note that it the arduino DOES receive the correct value).*
- **8.2  Turntable Position**: Changes the turntable position in the runtime config without moving the turntable. 

- **9  Live View...**: A submenu where you can get a live feed of how each DSLR camera is looking at each object.

``` txt
Live View Menu
('r' to return)

+---------------+-----------------+
| Object Name   : testing_speed_3 |
| Pose          : u               |
| Turntable Pos : 0               |
+---------------+-----------------+



 1  Start Live View 
 2  End Live View
```

- **9.1  Start Live View**: Starts the liveview, creates 5 windows, each one will have the image feed of each camera. Under the hood, we are constantly calling images through file streams from the camera. These are saved to `live_view_filestream` which is included in the gitignore.
- **9.2  End Live View**: Closes all windows. **IMPORTANT**: Each shutter's current state being on ready is very vital for taking pictures without errors. Make sure to close the Live View windows before trying to scan anything. Make sure that if the program exits due to some error before the Live View is ended, that on startup, End Live View is toggled. If shutter issues continue to occur I recommend turning the cameras off and cleaning lenses.


- **f Run Filecount Check Script**: This is a python script tool with configurable options in `moad_config.json` and located in the `scripts/` subdirectory. It's meant to check that the correct number of images exist after the most recent full scan (runs at the end of image colleciton loop) but can be configured to check every object folder. For more details, see the comments at the top of the file itself (`scripts/filecount_test.py`).

- **p Scan from saved state**: Ocassionally, due to mechanical failures/issues on the cameras or resource usage/segfaults in the code, a scan might fail before finishing. To address this, at each step, the current state, pose and turntable position are saved in `moad_config.json`. After re-exucuting `MultiCamCui`, you can use this option to read that info and start a scan from the saved state.