#include <vector>
#include <iostream>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError PressShutter(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsUInt32 status)
{
	EdsError	 err = EDS_ERR_OK;
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsSendCommand(cameraArray[i], kEdsCameraCommand_PressShutterButton, status);
	}
	return err;
}