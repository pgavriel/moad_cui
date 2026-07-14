#pragma once
/*
    MOADGlobals.h
    Forward-declares all globals defined in MOADCui.cpp that are shared
    across scan/data collection modules. Include this in any source file
    that needs access to these shared variables.
*/
#include <string>
#include <map>
#include <atomic>
#include <thread>
#include <vector>

#include "CanonHandler.h"
#include "RealSenseHandler.h"
#include "SerialCommunication.h"
#include "MenuHandler.h"

extern char         curr_pose;
extern int          degree_tracker;
extern std::string  scan_folder;
extern std::string  json_path;
extern std::string  moad_dir;

extern std::atomic<bool>            liveview_active;
extern std::map<std::string, std::string> object_info;

extern RealSenseHandler rshandle;
extern SimpleSerial*    Serial;
extern MenuHandler*     curr_menu;

// Forward declarations for utility functions defined in MOADCui.cpp
// that ScanManager needs to call.
int  create_folder(std::string path, bool quiet = false);