#include <vector>
#include <iostream>
#include <thread>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError TakePicture(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	EdsError	 err = EDS_ERR_OK;
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		std::cout << "Shooting cam" << bodyID[i] << std::endl;
		err = EdsSendCommand(cameraArray[i], kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely_NonAF); // kEdsCameraCommand_ShutterButton_Completely
//	}
//	for (i = 0; i < cameraArray.size(); i++)
//	{
		err = EdsSendCommand(cameraArray[i], kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF);
	}
	return err;
}

