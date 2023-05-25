#include <vector>
#include <iostream>
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "CameraEvent.h"

EdsError PreSetting(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	EdsError	 err = EDS_ERR_OK;
	EdsUInt32 saveto;
	// Specify where to save images
	saveto = kEdsSaveTo_Host;	//kEdsSaveTo_Host or kEdsSaveTo_Camera or kEdsSaveTo_Both;

	std::cout << "session openning" << std::endl;

	for (EdsUInt32 i = 0; i < cameraArray.size(); i++)
	{
		err = EdsOpenSession(cameraArray[i]);

		//for powershot
		err = EdsSendCommand(cameraArray[i], kEdsCameraCommand_SetRemoteShootingMode, kDcRemoteShootingModeStart);

		err = EdsSetPropertyData(cameraArray[i], kEdsPropID_SaveTo, 0, sizeof(saveto), &saveto);

		if (err == EDS_ERR_OK)
		{
			err = EdsSendStatusCommand(cameraArray[i], kEdsCameraStatusCommand_UILock, 0);
		}

		if (err == EDS_ERR_OK)
		{
			EdsCapacity capacity = { 0x7FFFFFFF, 0x1000, 1 };
			err = EdsSetCapacity(cameraArray[i], capacity);
		}
		if (err == EDS_ERR_OK)
		{
			EdsSendStatusCommand(cameraArray[i], kEdsCameraStatusCommand_UIUnLock, 0);
		}

		//Set Property Event Handler
		if (err == EDS_ERR_OK)
		{
			err = EdsSetPropertyEventHandler(cameraArray[i], kEdsPropertyEvent_All, handlePropertyEvent, (EdsVoid*)bodyID[i]);// bodyID
		}

		//Set Object Event Handler
		if (err == EDS_ERR_OK)
		{
			err = EdsSetObjectEventHandler(cameraArray[i], kEdsObjectEvent_All, handleObjectEvent, (EdsVoid*)bodyID[i]);
		}

		//Set State Event Handler
		if (err == EDS_ERR_OK)
		{
			err = EdsSetCameraStateEventHandler(cameraArray[i], kEdsStateEvent_All, handleSateEvent, (EdsVoid*)bodyID[i]);
		}
	}
	return err;
}
