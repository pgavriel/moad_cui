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
  
## Parameter Usage (Config Text File)  

In order to reduce the amount of rebuilding code during usage, an effort was made to paramaterize code using the text file *moad_config.txt*, which allows the user to specify things like output directory, which data streams to collect, pointcloud filter options, COM port for motor control, delays, and timeouts.  
TODO: The 4x4 matrices used to appropriately transform the pointclouds should be parameterized as well, but this is not currently implemented and the source needs to be changed to do so. Instructions on finding the right transforms for the Realsense cameras can be found on the MOAD website. The same is true for setting the RS serial number that corresponds to a specified name.  
The parameters *collect_rs*, and *collect_dslr* are booleans and determine which sensors are being used. If either of those parameters is set to 0, the corresponding initialization for those sensors will be skipped, if they are set to 1, it will connect to all available sensors of that type.  
**For Realsense data,**  
*rs_color=1* saves color images  
*rs_depth=1* saves depth images  
*rs_pointcloud=1* saves colored pointcloud.ply files. The pointclouds are also transformed and filtered unless *rs_raw_pointclouds=1* is also set  
*rs_timeout_sec* is an integer that represents the number of seconds before the realsense pipelines will timeout  
*rs_xpass, rs_ypass, rs_zpass, rs_sor, and rs_voxel* are all boolean parameters that determine which filters are applied (after the pointcloud is transformed), and they each have some parameters associated with them that are all float values except *rs_sor_meank* which is an integer value.  
**For DSLR data,**  
*dslr_timeout_sec* is an integer of how many seconds it should wait for all the captured images to be recieved before timing out.  
**Other parameters,**  
*output_dir* is the directory path to which all output data should be saved.  
*object_name* is the name of the object being scanned, and the program will create subfolders within output_dir of that name to save data for that particular object. The object_name can also be changed while the program is running and it will create new folders as needed.  
*serial_com_port* the COM port that the arduino controlling the motor is connected to (if that route was taken for motor control)(currently can only handle single digit ports)  
*turntable_delay_ms* the number of milliseconds to wait after the turntable stops moving to start collecting data.

## Using the program

 1.Connect the cameras to your PC with a USB cable.  
 2.Configure moad_config.txt as needed for data collection  
 3.Run MultiCamCui.exe: The terminal output of the program will first show the parameters it read from the moad_config.txt file. Next, it will attempt to connect to the arduino COM port for turntable motor control, start the realsense cameras, and connect to the DSLR cameras. If anything goes wrong, there should be output to reflect what happened. Finally, the control menu will be displayed.  
 4.Control menu  
   The control menu is the following:  
   [ 1] Collect & Download Data  
   [ 2] Press Halfway  
   *[ 3] Press Completely*  
   [ 4] Press Off  
   *[ 5] TV*  
   *[ 6] AV*  
   *[ 7] ISO*  
   *[ 8] White Balance*  
   *[ 9] Drive Mode*  
   *[10] AE Mode (read only)*  
   *[11] Get Live View*  
   *[12] File Download*  
   [13] Run Object Scan  
  [14] Turntable Control  
  [15] Set Object Name  
  [16] Set Turntable Position  

  NOTE: Italic menu items or either not yet implemented or not tested.  
  Select the desired menu item by entering the number into the terminal and pressing ENTER.  
  Entering 'r' (return) will safely shut down the sensors and exit the program.  
  
   Select the item number you want to control.
   The following is a description of the operation for each input number.
   *Enter "r" key to move to "Top Menu"

   **[ 1] Collect & Download Data**
    Press and release the shutter button without AF action,
    create a "cam + number" folder in the folder where MultiCamCui.exe is located
    and save the pictures taken with each camera.

    * If you can't shoot, change the mode dial to "M" and then try again.
    * The camera number changes in the order you connect the camera to your PC.

   **[ 2] Press Halfway**
    Press the shutter button halfway. Causes DSLR cameras to adjust auto-focus.  

   *[ 3] Press Completely*
    Press the shutter button completely.
    When Drive mode is set to continuous shooting,
    Continuous shooting is performed.

   **[ 4] Press Off**
    Release the shutter button.

   *[ 5] TV*
    *Set Tv settings.*

   *[ 6] AV*
    *Set Av settings.*

   *[ 7] ISO*
    *Set ISO settings.*

   *[ 8] White Balance*
    *Set White Balance settings.*

   *[ 9] Drive Mode*
    *Set Drive mode settings.*

   *[10] AE Mode (read only)*
    *Refers to AE mode settings (not configurable)*

   *[11] Get Live View*
    *Get one live view image.
    In the folder where MultiCamCui.exe is located
    Automatically create a "cam number" folder and save each camera's image.*

   *[12] File Download*
    *Download all picture File in the camera's card to PC.
    In the folder where MultiCamCui.exe is located
    automatically create a "cam number" folder and save each camera's image.*

  **[13] Run Object Scan**  
   Performs an object scan which will collect data as specified in the text configuration file, then move the turntable, and repeat for a specified number of times.  
   The user will be prompted to input the number of degrees to move the turn table between each collection, and how many turns to make, and will then perform the scan. There is no clean way to interrupt the scan manually, but if any of the sensor pipelines fail, it will stop automatically.  

  **[14] Turntable Control**  
   Enters a mode that allows direct control of the turntable. Enter the number of degrees for the turntable to move (negative or positive). Enter 'r' to return to the main menu.  

  **[15] Set Object Name**  
   Sets the name of the output folder for data being collected, and creates new folders if needed.  

  **[16] Set Turntable Position**  
   Sets the current position (in degrees) of the turntable. This does not move the turntable, it only effects how the output files are named.  

