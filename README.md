# MOAD CUI  

## MOAD Website  

<https://www.robot-manipulation.org/nist-moad>
  
## Requirements  

- Canon EDSDK : <https://developercommunity.usa.canon.com/s/article/How-do-I-apply-for-a-development-tool-SDK-API-Etc>
- RealSense SDK : <https://www.intelrealsense.com/sdk-2/>  
- simple_serial_port : <https://github.com/dmicha16/simple_serial_port>  
- PCL : Install with vcpkg  <https://pointclouds.org/downloads/>
- OpenCV : <https://opencv.org/releases/>
  
This repository should be cloned into the same folder as the Canon SDK (EDSDK and EDSDK_64 folders), as well as the simple_serial_port. The CMake may need to be updated to point to the install locations for OpenCV, Intel RealSense SDK, and PCL. This project was developed in Visual Studio Code and built with the VS 2022 Release-amd64 build tools.  
  
## Parameter Usage (Config JSON File)  

In order to reduce the amount of rebuilding code during usage, an effort was made to paramaterize code using the text file *moad_config.json*, which allows the user to specify things like output directory, which data streams to collect, pointcloud filter options, COM port for motor control, delays, and timeouts.

The 4x4 matrices used to appropriately transform the pointclouds are inside the "calibration/realsense/transform.json", where you can find the serial number, alias, and the transformation matrix applied to each camera. When doing calibration you only need to change the appropiate matrix with the newly calibrated one.

## Config JSON Values

### General

- **Debug (debug)**: Display the debug prints, useful to test the flow of the program as well as checking process time.

- **Output Dir (output_dir)**: Path of the directory where the scan will be saved.

- **Object Name (object_name)**: The name of the scan object, this name will be used to create a folder inside the given **Output Dir** to store all the scan data.

- **Thread Numbers (thread_num)**: The number of threads used when scanning using the Realsense, it is recommended to not use a value greater than 8 due to dimminishing result due to the numbers of threads living at the same time in the program.

- **Degree Increment (degree_inc)**: The degree that the turntable will move in each scan step (Used only in full scan).

- **Number of Moves (num_moves)**: The number of steps that a scan will take (Used only in full scan).

- **Serial Connection Port (serial_con_port)**: The port number where the arduino controlling the turntable is connected.

- **Turntable Delay (turntable_delay_ms)**: The wait time that turntable takes after moving, used to avoid any type of sudden movement to the object of the table that can be cause when moving.

### DSLR

- **Collect DSLR (collect_dslr)**: Boolean check to decide if we collect DSLR image on scan

- **DSLR Timeout (dslr_timeout_sec)**: Set the maximun wait time to get an image

### Realsense

- **Collect Realsense (collect_realsense)**: Boolean check to decide if we collect Realsense data on scan.

- **Realsense Timeout (realsense_timeout_sec)**: Set the maximum wait time to get the realsense data.

- **Transform Path (transform_path)**: Path of the directory where the transform json is located.

- **Transform File (transform_file)**: Name of the json file for the transform matrix.

- **Collect Color (collect_color)**: Boolean check to get the color image from the realsense.

- **Collect Pointcloud (collect_pointcloud)**: Boolean check to get the pointclouds from the realsense.

- **Raw Pointclods (raw_pointcloud)**: Boolean check to state if you want the pointcloud without any transformation and filter applied

- **Compute Normals (compute_normals)**: Boolean check to state if you want the normals computed for the given pointcloud

- **Normals Threads (normals_threads)**: Interger value that state the number of threads used when calculating the normals. In my experience, when the pointcloud size is not that big, it is better to set the threads to just 1, because it is faster than splitting the calculation into different threads.

### Pointcloud Filters

All of the pointclouds filter objects have a similar structure, where they have a boolean value that check if we apply said filter and some integer value that state either the min max value or some value asked for the specific filter

Passthrough filters options: **apply**, **min**, **max**.\
SOR filter options:  **apply**, **k**, **stddev**.\
Voxel filter options:  **apply**, **leaf_size**.

- **X Passthrough (xpass)**: Filter all the points between the **min** and **max** points of the pointcloud in the X coordinate.

- **Y Passthrough (ypass)**: Filter all the points between the **min** and **max** points of the pointcloud in the Y coordinate.

- **Z Passthrough (zpass)**: Filter all the points between the **min** and **max** points of the pointcloud in the Z coordinate.

- **K Value (k)**: Used in the SOR filter, it states the number of neighbors from an specific point that it will take into consideration for the outlier removal.

- **Standard Deviation (stddev)**: Used in the SOR filtering, this number is used to determine if the neighboring points are outlier or not

- **Leaf Size (leaf_size)**: Used in the Voxel filtering, the leaf size it is used to set the amount of downsampling that the filter will take. The lower the value the less points will be downsampled but it will maintain the main structure of the data.

### Transform Generator

This option are used specifically to generate the transform of each camera after scan as if the camera is the one that is rotating.

- **Linux Directory (dir_linux)**: Directory in linux where the scan object is saved (the same as object dir).

- **Windows Directory (dir_windows)**: Directory in windows where the scan object is saved (the same as object dir).

- **Linux Calibration Directory (calibration_dir_linux)**: Directory in Linux where the calibration files are located, which is a json file for the transform of each DSLR camera based on the focal length.

- **Windows Calibration Directory (calibration_dir_windows)**: Directory in Windows where the calibration files are located, which is a json file for the transform of each DSLR camera based on the focal length.

- **Calibration Mode (calibration_mode)**: Equivalent to the focal length of each camera.

- **Degree Angle (degree_angle)**: The same as **Degree Increment**.

- **Scan Range (scan_range)**: The maximun degree that the turntable will rotate, it can be calculated by using the **Degree increment** times the **Number of moves**.

- **Visualize (visualize)**: Boolean flag that display a 3D graph after the transform are calculated.

- **Force (force)**: Boolean flag that force the program to run without any user input or confirmation.

## Using the program

 1. Connect the cameras to your PC with a USB cable.  
 2. Configure moad_config.txt as needed for data collection  
 3. Run MultiCamCui.exe: It will attempt to connect to the arduino COM port for turntable motor control, start the realsense cameras, and connect to the DSLR cameras. If anything goes wrong, there should be output to reflect what happened. Finally, the control menu will be displayed.  
 4. Control menu  
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

  Entering 'r' (return) will safely shut down the sensors and exit the program.  
  
  Select the item number you want to control. The following is a description of the operation for each input number.
  Enter "r" key to move to the "Top Menu".

- **0  Reload Config**: Reload the moad_config JSON file, used when you want to change some config values during runtime, for example a filter that you no longer want to apply or changing the angle of the scan.

- **1  Full Scan**: It does an object scan using the object name, pose, and scan configuration stated in the moad_config.json.

- **2  Custom Scan**: It does an object scan using the existing object name and pose, however the degree and number of steps will be prompted by the user at runtime.

- **3  Collect Single Data**: It scans a single batch of data from the current angle to the active object name.

- **4  Set Object Name**: It it set the name of the object, if the object does not exist, then it will create the respective folders and file needed in order to scan. It calculate the current pose based on the last available pose in the object file.

- **5  Set Pose**: Manually set the pose of the object file.

- **6  Camera Calibration...**: A submenu where all the camera focus option will reside

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

- **6.1  Press Halfway**: Equivalent to phisically pressing shoot buttom halfway in the camera, used to reset focus of the camera.

- **6.2  Press Completely**: Equivalent to phisically pressing the button completely in the camera, it takes a photo and save it in memory.

- **6.3  Press Off**: Equivalent to letting go the button in the camera, it reset the state of the camera to ready.

- **7  Camera Options...**: A submenu where you can find and change some of the configurations values of the DSLR cameras, such as AV, TV and ISO.

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

- **7.1  TV**: Set the TV value to one of the given by a table, The TV is in charge of setting the shutter speed of the camera, the lower the value the sharper the image will look like at a cost of shooting speed.
- **7.2  AV**: Set the AV value to one of the given by a table. The AV is in charge stating how much light the camera can recieve during the shooting.
- **7.3  ISO**: Set the ISO value to one of the given by a table. The ISO is in charge of the sensitiviy of the light sensor of the camera.
- **7.4  White Balance**: : Set the White Balance value to one of the given by a table. The White Balance is a post processing option that is in charge of setting the ambience light depending of the intensity and color of the light. Recommendation, use the one that keep the image true to the real life color, avoid using auto.
- **7.5  Drive Mode**: : Set the Drive Mode value to one of the given by a table. Used to change the behavior of the shutter buttom.
- **7.6  AE Mode**: (DEPRICATED)
- **7.7  Change Camera**: It change the camera that the options will be applied to, by default those option will be applied to all cameras, however you can set which individual camera you want the option to change,

- **8  Turntable Options...**: A submenu where you can find options related to the turntable itself.

``` txt
Turntable Options
('r' to return)

+---------------+-----------------+
| Object Name   : testing_speed_3 |
| Pose          : u               |
| Turntable Pos : 0               |
+---------------+-----------------+



 1  Turntable Control  
 2  Turntable Position
 ```

- **8.1  Turntable Control**: It gives you control of the turntable to send degrees to move the table to said degree.
- **8.2  Turntable Position**: It changes the turntable position without moving the turntable.

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

- **9.1  Start Live View**: Starts the liveview, creates 5 windows, each one will have the image feed of each camera.
- **9.2  End Live View**: Closes all the window, and change the configuration of the camera back to normal.
