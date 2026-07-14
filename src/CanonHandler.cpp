#include <CanonHandler.h>
#include "DebugUtils.h" 
#include "ConfigHandler.h"

#include "PreSetting.h"

CanonHandler::CanonHandler() {
    // DebugUtils::logCanon("CanonHandler created...");
}

CanonHandler::~CanonHandler() {
    shutdown();
}

void CanonHandler::shutdown() {
    if (!isSDKLoaded) return;  // guard against double-calls
    
    DebugUtils::logCanon("Shutting down canonhandler...");
    // Release camera list
    if (cameraList != NULL) {
        EdsRelease(cameraList);
    }
    DebugUtils::logCanon("Released camera list.");
    // Release Camera
    for (i = 0; i < cameraArray.size(); i++) {
        if (cameraArray[i] != NULL) {
            EdsRelease(cameraArray[i]);
            cameraArray[i] = NULL;
        }
    }
    DebugUtils::logCanon("Released camera array.");
    //Remove elements before looping. Memory is automatically freed at the destructor of the vector when it leaves the scope.
    cameraArray.clear();
    bodyID.clear();

    // Termination of SDK
    DebugUtils::logCanon("Terminating SDK.");
    EdsTerminateSDK();

    DebugUtils::logCanon("Done.");
    isSDKLoaded = false;       // flag prevents destructor from repeating it
}

// Discover and connect to all available cameras
int CanonHandler::camera_check() {
    cameraArray.clear();
    bodyID.clear();

    //Acquisition of camera list
    if (err == EDS_ERR_OK)
    {
        err = EdsGetCameraList(&cameraList);
    }

    //Acquisition of number of Cameras
    if (err == EDS_ERR_OK)
    {
        err = EdsGetChildCount(cameraList, &count);
        if (count == 0)
        {
            // std::cout << "Cannot detect any camera" << std::endl;
            DebugUtils::logWarning("No active cameras found. Are they turned on and awake?");
            // pause_return();
            return count;
            // exit(EXIT_FAILURE);
        }
        else if (count > 30)
        {
            // std::cout << "Too many cameras detected" << std::endl;
            DebugUtils::logWarning("Too many cameras detected...");
            // pause_return();
            return count;
            // exit(EXIT_FAILURE);
        }

        // magenta text via ANSI escape codes
        // std::cout << "\033[1;45m" << count << " cameras detected." << "\033[0m" << std::endl;
        DebugUtils::logCanon("Detected " + std::to_string(count) + " active cameras.");
    }

    //Acquisition of camera at the head of the list
    for (i = 0; i < count; i++)
    {
        if (err == EDS_ERR_OK)
        {
            err = EdsGetChildAtIndex(cameraList, i, &camera);
            EdsDeviceInfo deviceInfo;
            err = EdsGetDeviceInfo(camera, &deviceInfo);
            // EdsUInt32 serial;
            // err = EdsGetPropertyData(camera, kEdsPropID_BodyIDEx, 0, sizeof(serial), &serial);
            // EdsChar serialNumber[EDS_MAX_NAME];
            // err = EdsGetPropertyData(camera, kEdsPropID_BodyIDEx, 0, sizeof(serialNumber), serialNumber);
            
            // std::cout << serialNumber << std::endl;
            if (err == EDS_ERR_OK && camera == NULL)
            {
                DebugUtils::logWarning("Camera not found.");
                // std::cout << "Camera is not found." << std::endl;
                // pause_return();
                exit(EXIT_FAILURE);
            }
            std::stringstream cam_info;
            cam_info << "[" << i + 1 << "]\t" << deviceInfo.szDeviceDescription 
                << "\t" << "(Serial # not accessible until session started)";
            DebugUtils::logCanon(cam_info.str());
        }else{ 
            DebugUtils::logWarning("Something went wrong... Error: " + std::to_string(err));
        }
    }
    DebugUtils::logWhitespace();

    //Connect to all available cameras
    if (err == EDS_ERR_OK) {
        DebugUtils::logCanon("Creating camera array...");
        for (unsigned int i = 0; i < count; i++)
        {
            err = EdsGetChildAtIndex(cameraList, i, &camera);
            cameraArray.push_back(camera);
            bodyID.push_back(i + 1);
        }
    }

    return count;
}

void CanonHandler::initialize() {
    ConfigHandler& config = ConfigHandler::getInstance();
    // Early return if DSLR data collection not enabled.
    if (!config.getValue<bool>("dslr.enable_collection")) {
        DebugUtils::logRS("Skipping DSLR setup, (dslr.enable_collection=false in config).");
        DebugUtils::logWhitespace();
        return;
    }
    
    DebugUtils::logInfo("Initializing DSLR Cameras...");

    // Gather Camera Serials from config
    std::string cam1 = config.getValue<std::string>("dslr.camera_ids.CAMERA_1");
	std::string cam2 = config.getValue<std::string>("dslr.camera_ids.CAMERA_2");
	std::string cam3 = config.getValue<std::string>("dslr.camera_ids.CAMERA_3");
	std::string cam4 = config.getValue<std::string>("dslr.camera_ids.CAMERA_4");
	std::string cam5 = config.getValue<std::string>("dslr.camera_ids.CAMERA_5");

    // Initialize SDK
    err = EdsInitializeSDK();
    if (err == EDS_ERR_OK)
	{
		isSDKLoaded = true;
        DebugUtils::logCanon("SDK Initialized...");
	}
    
    // Discover and connect to all cameras
    cameras_found = camera_check();

    PreSetting(cameraArray, bodyID);

    // Automatic remapping of camera names from moad_config
    DebugUtils::logCanon("Remapping Camera Names...");
    EdsChar serial[13];
    EdsError err;
    std::stringstream cam_message;
    int index = 1;
    for (const auto& camera: cameraArray) {
        // Fetch the serial number
        err = EdsGetPropertyData(camera, kEdsPropID_BodyIDEx, 0, sizeof(serial), &serial);
        if (err != 0)  {
            // std::cout << "canon initialization err: " << err << std::endl;
            DebugUtils::logError("Canon initialization error: " + std::to_string(err));
        }

        // Convert it into string
        std::string serial_str = "";
        for (size_t i = 0; i < sizeof(serial) - 1; i++){
            serial_str += serial[i];
        }

        cam_message.str("");
        cam_message.clear();
        cam_message << "Camera " << index << " (Serial Number: " << serial_str <<"):";


        if (serial_str == cam1) {
            cam_message << "\n\t\t|- Renamed to Camera 1";
            camera_names[std::to_string(index)] = "1";
            camera_name[camera] = "Camera 1";
        }
        else if (serial_str == cam2) {
            cam_message << "\n\t\t|- Renamed to Camera 2";
            camera_names[std::to_string(index)] = "2";
            camera_name[camera] = "Camera 2";
        }
        else if (serial_str == cam3) {
            cam_message << "\n\t\t|- Renamed to Camera 3";
            camera_names[std::to_string(index)] = "3";
            camera_name[camera] = "Camera 3";
        }
        else if (serial_str == cam4) {
            cam_message << "\n\t\t|- Renamed to Camera 4";
            camera_names[std::to_string(index)] = "4";
            camera_name[camera] = "Camera 4";
        }
        else if (serial_str == cam5) {
            cam_message << "\n\t\t|- Renamed to Camera 5";
            camera_names[std::to_string(index)] = "5";
            camera_name[camera] = "Camera 5";
        }else {
            // If serial number is not matched, set the camera name to it's serial number.
            cam_message << "\n\t\t|- NO MATCHING SERIAL NUMBER FOUND IN CONFIG (dslr/camera_ids), setting name to serial...";
            camera_names[std::to_string(index)] = serial_str;
            camera_name[camera] = "Camera " + serial_str;
        }
        
        DebugUtils::logCanon(cam_message.str());
        index++;
    }
    DebugUtils::logWhitespace();
}


// Create global CanonHandler object that CanonSDK functions can reference
CanonHandler canonhandle;