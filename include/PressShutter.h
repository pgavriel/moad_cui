#include <vector>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError PressShutter(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsUInt32 status);