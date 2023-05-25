#include <vector>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError StartEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
EdsError DownloadEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& _bodyID);
EdsError EndEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
