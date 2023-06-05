#include <CanonHandler.h>

CanonHandler::CanonHandler() {
    std::cout << "Canon Handle created.\n";
}

CanonHandler::~CanonHandler() {
    std::cout << "Shutting down Canon Handler... ";
    // Release camera list
	if (cameraList != NULL) {
		EdsRelease(cameraList);
	}
    // Release Camera
    for (i = 0; i < cameraArray.size(); i++) {
		if (cameraArray[i] != NULL) {
			EdsRelease(cameraArray[i]);
			cameraArray[i] = NULL;
		}
	}
    //Remove elements before looping. Memory is automatically freed at the destructor of the vector when it leaves the scope.
	cameraArray.clear();
	bodyID.clear();
    // Termination of SDK
    if(isSDKLoaded) {
         EdsTerminateSDK();
    }
    std::cout << "Done.\n";
}

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
            std::cout << "Cannot detect any camera" << std::endl;
            // pause_return();
            return count;
            // exit(EXIT_FAILURE);
        }
        else if (count > 30)
        {
            std::cout << "Too many cameras detected" << std::endl;
            // pause_return();
            return count;
            // exit(EXIT_FAILURE);
        }
        std::cout << count << " cameras detected." << std::endl;
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
            std::cout << err << std::endl;
            if (err == EDS_ERR_OK && camera == NULL)
            {
                std::cout << "Camera is not found." << std::endl;
                // pause_return();
                exit(EXIT_FAILURE);
            }
            std::cout << "[" << i + 1 << "]\t" << deviceInfo.szDeviceDescription 
                << "\t" << "serial" << std::endl;
            // Sleep(1000);
            // if  (serial == 3435973836 ) std::cout << "true!\n";
        }else{ std::cout << "SOMETHING WRONG: Error Code "<< err << "\n";}
    }
    std::cout << "--------------------------------" << std::endl;

    //Connect to all available cameras
    if (err == EDS_ERR_OK) {
        std::cout << "connecting to all cameras..." << std::endl;
        for (int i = 0; i < count; i++)
        {
            err = EdsGetChildAtIndex(cameraList, i, &camera);
            cameraArray.push_back(camera);
            bodyID.push_back(i + 1);
        }
    }

    return count;
}

void CanonHandler::initialize() {
    // Initialize SDK
    err = EdsInitializeSDK();
    if (err == EDS_ERR_OK)
	{
		isSDKLoaded = true;
        std::cout << "SDK Initialized...\n";
	}
    
    cameras_found = camera_check();
}


// Create global CanonHandler object that CanonSDK functions can reference
CanonHandler canonhandle;