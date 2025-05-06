#include <vector>
#include "EDSDK.h"
#include "EDSDKTypes.h"

void ReleaseStream(EdsStreamRef& stream, EdsEvfImageRef& image);
void throwCameraException(EdsError err, const char* message);

EdsError StartEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& bodyID);
EdsError DownloadEvfCommand(EdsCameraRef const& camera, std::string const& cameraName, std::thread::id const& parentThreadID);
EdsError EndEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& bodyID);
EdsError StartEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
EdsError DownloadEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& _bodyID);
EdsError EndEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
