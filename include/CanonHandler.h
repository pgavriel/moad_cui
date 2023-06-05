
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <regex>
#include "EDSDK.h"
#include "EDSDKTypes.h"


class CanonHandler {
private:
    int i;
	EdsCameraRef camera;
public:
    CanonHandler();
    ~CanonHandler();
    void initialize();
    int camera_check();

    int turntable_position = 0;
    int images_downloaded = 0;
    bool save_image = true;
    std::string save_dir;
    int cameras_found = 0;

    EdsError err = EDS_ERR_OK;
	EdsUInt32 count = 0;
	bool isSDKLoaded = false;
    EdsCameraListRef cameraList = NULL;
    EdsUInt32 data;
    std::vector<EdsCameraRef> cameraArray;
	std::vector<EdsUInt64> bodyID;
};

extern CanonHandler canonhandle;
