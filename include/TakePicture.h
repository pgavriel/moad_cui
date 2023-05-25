#include <vector>
#include "EDSDKTypes.h"

EdsError TakePicture(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
EdsError TakePictureMultiThread(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);

