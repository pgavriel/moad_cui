#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError downloadImage(EdsDirectoryItemRef  directoryItem, EdsVoid* _bodyID);
EdsError DownloadImageAll(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
