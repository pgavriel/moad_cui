#include <vector>
#include <map>
#include "EDSDKTypes.h"

EdsError TakePicture(std::vector<EdsCameraRef> const& cameraArray, std::map<EdsCameraRef, std::string> const& bodyID);
EdsError TakePicture(EdsCameraRef const& cameraArray, std::string const& bodyID);
EdsError TakePictureMultiThread(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);

