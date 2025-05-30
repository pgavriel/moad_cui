#include <vector>
#include <map>
#include <iostream>
#include <thread>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError TakePicture(EdsCameraRef const& camera, std::string const& bodyID) {
	EdsError err = EDS_ERR_OK;
	
	std::cout << "Shooting cam" << bodyID << std::endl;
	// Press the shutter button completely to take a picture
	err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely_NonAF); // kEdsCameraCommand_ShutterButton_Completely
	// Release the shutter button after taking the picture
	err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF);
	
	return err;
}

EdsError TakePicture(std::vector<EdsCameraRef> const& cameraArray, std::map<EdsCameraRef, std::string> const& bodyID) {
	EdsError err = EDS_ERR_OK;
	
	// For each camera in the array, take a picture
	for (auto& camera : cameraArray) {
		TakePicture(camera, bodyID.at(camera));
	}

	return err;
}
