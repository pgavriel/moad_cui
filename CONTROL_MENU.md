
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