#include <iostream>
#include <filesystem>
#include <iterator>
#include <list>
#include <regex>
#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "ScanManager.h"		// Functions for coordinating data collection
#include "MenuHandler.h"  		// CUI Menu functionality
#include "tabulate.hpp"			// Renders CUI menus
#include "ConfigHandler.h" 		// Handles the global config data loaded from moad_config.json
#include "CanonHandler.h" 		// Handles initialization and  data collection from the DSLR camera
#include "RealSenseHandler.h" 	// Handles initialization and data collection from realsense
#include "ThreadPool.h" 		// Thread pool implementation for parallel data collection
#include "DebugUtils.h" 		// Handles terminal output and logging
#include "ScriptRunner.h" 		// Runs external python scripts
#include "SerialCommunication.h"// Handles serial communication for turntable control 

// Canon SDK Headers
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "Download.h"
#include "DownloadEvf.h"
#include "PreSetting.h"
#include "PressShutter.h"
#include "Property.h"
#include "TakePicture.h"
#include "CameraException.h"


// MACRO for cross platform file seperator: (Can be deprecated since we're solely supporting linux moving forward)
// 	TODO: use std::filesystem that auto handles cross platform? 
// 	the reason I am NOT doing this as of 10/30/2025:
//		- many paths are hardcoded and built at runtime via string concats
//		- i prefer 1 less library to have to worry about and update
//	example usage: std::string debug_log_dir = scan_folder + PATH_SEP + "debug_log.txt";
#ifdef _WIN32
	const char PATH_SEP = '\\';
#else
	const char PATH_SEP = '/';
	const bool OS_LINUX = true;
#endif

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using ordered_json = nlohmann::ordered_json; // TODO: remove aliasing?

// GLOBAL VARIABLE DEFINITIONS ================================================================
//	TODO: Cleanup / remove need for globals

// used in CheckKey()
std::string control_number = ""; // Stores user input for menu selection
bool keyflag;					 // Flags user input has been stored

// used in multiple functions, should maybe be *directly* config based?
char curr_pose = 'a';
int degree_tracker = 0;
std::string scan_folder;

// Liveview Threads
std::vector<std::thread> liveview_th;
std::atomic<bool> liveview_active = false;
std::thread::id liveview_thread_id;

// Camera
bool all_cameras = true;
EdsCameraRef activeCamera;

RealSenseHandler rshandle;
SimpleSerial* Serial;

// Menu pointer
MenuHandler* curr_menu;

// TXT Config 
std::map<std::string, std::string> object_info;
std::string json_path;
std::string moad_dir;


// Initialize some functions
void setObjectName(std::string object_name);
bool WaitForCameraReady(EdsCameraRef camera, int timeoutMs);
void rotate_turntable(int degree_inc);
int create_folder(std::string path, bool quiet);

// Function wrapper so run_filecount_check(scan_folder) can be 
// bound to the menu, which only accepts plain bool(*)() function pointers.
bool runFilecountCheck() {
    return run_filecount_check(scan_folder);
}


/* ===========================================================================================
For reloading the moad_config during runtime. Checks if data collection for DSLR/RS has been enabled,
and will go through the initialization process if so. 
TODO: Can probably be refactored into the config handler?
=========================================================================================== */
bool reloadConfig() {
	if (liveview_active) {
		std::cout << "Liveview is active, please stop it before reloading the config." << std::endl;
		return false;
	}

	// If config was already loaded, save current state of collect DSLR/RS.
	ConfigHandler& config = ConfigHandler::getInstance();
	std::optional<bool> previous_DSLR;
	std::optional<bool> previous_RS;
	
	if (!config.emptyConfig()) {
		DebugUtils::logDebug("Checking previously loaded config...");
		previous_DSLR = config.getValue<bool>("dslr.enable_collection");
		previous_RS = config.getValue<bool>("realsense.enable_collection");
	}

	// json_path is a global set within main()
	config.loadConfig(json_path);

	// Check if the configuration has changed for DSLR, if so, initialize Canon Handler
	if (!previous_DSLR.has_value() || previous_DSLR != config.getValue<bool>("dslr.enable_collection")) {
		if (config.getValue<bool>("dslr.enable_collection") && !canonhandle.isSDKLoaded) {
			canonhandle.initialize();
		}
	}
	
	// Check if the configuration has changed for RealSense, if so, initialize RS handler
	if (!previous_RS.has_value() || previous_RS != config.getValue<bool>("realsense.enable_collection")) {
		if (config.getValue<bool>("realsense.enable_collection")) {
			rshandle.initialize(json_path);
		}
	}

	// Set the object name from the configuration
	std::string object_name = config.getValue<std::string>("object_name");
	setObjectName(object_name);
	DebugUtils::logWhitespace();

	return false;
}


/* -----------------------------------------------------------------------------
	FUNCTIONS FOR HANDLING USER INPUT
----------------------------------------------------------------------------- */
void CheckKey()// After key is entered, _ endthread is automatically called.
{
	std::cin >> control_number;
	std::cin.ignore();// ignore "\n"
	keyflag = true;
}

EdsInt32 getvalue()
{
	std::string input;
	std::smatch  match_results;

	std::cin >> input;
	if (std::regex_search(input, match_results, std::regex("[0-9]"))) {
		return stoi(input);
	}

	return -1;
}

void validate_input(std::string text, std::string& input, std::regex validation) {
	bool validated = false;
	do {
		std::cout << text;
		std::cin >> input;

		validated = std::regex_match(input, validation);

		if (!validated) {
			std::cout << "Input Invalid";
		}
	}
	while (!validated);
}

// definition
// takes string of path and bool for printout statements
// also checks if path already exists before attempting to create
int create_folder(std::string path, bool quiet=false) {
	DebugUtils::logFileSys("Creating Folder: " + path);

	if (!fs::exists(path)) {
        // Create the folder and any necessary higher level folders
        if (fs::create_directories(path)) {
			DebugUtils::logFileSys("Folder created: " + path);
        } else {
			DebugUtils::logError("Failed to create folder: " + path);
            return 1;
        }
    } else { 
		DebugUtils::logWarning("Folder already exists: " + path + " (You may accidentally overwrite data.)");
    }
	return 0;
}


char get_last_pose() {
	ConfigHandler& config = ConfigHandler::getInstance();
	std::string object_name = config.getValue<std::string>("object_name");
	std::string output_dir = config.getValue<std::string>("output_dir");
	std::string path = output_dir + PATH_SEP + object_name;
	char last_pose = 'a'; // First Pose
	
	// Check if the directory exists
	// NOTE: possibly fails on this if statement -GS 7/3

	// if the filepath of the new object does NOT exist OR if NOT a directory OR if path is empty: return a (early exit)
	if (!fs::exists(path) || !fs::is_directory(path) || fs::is_empty(path)) {
		return 'a';
	}

	// print out path and contents of path
	// std::cout << "printing contents of directory " << path << std::endl;
	// for (const auto & entry : fs::directory_iterator(path)) {
	// 	std::cout << entry.path() << std:: endl;
	// }
	
	// Check for files that start with 'pose-'
	for (const auto& entry : fs::directory_iterator(path)) {
		// first check if directory exists
		if (fs::is_directory(entry)) {

			// setup entry as string filepath so we can run find() with "pose-"
			std::string name = entry.path().filename().string();
			if (name.find("pose-") != std::string::npos) {

				// get the current letter by checking the last character of the entry filepath with pose-
				int length = name.length();
				char letter = name[length - 1];

				// Check if the folder is empty
				if (fs::is_empty(path + PATH_SEP + "pose-" + letter + PATH_SEP)) {
					return letter; // Current pose
				}
				else {
					// If is not empty, check if the subfolder is empty
					for (const auto& sub_entry : fs::directory_iterator(path + PATH_SEP + "pose-" + letter)) {
						if (fs::is_directory(entry) && fs::is_empty(sub_entry)) {
							return letter; // Current pose
						}
					}
				}

				if (last_pose < letter) {
					last_pose = name[length - 1];
				}
			}
		}
	}



	// If all the folder are not empty, then go the next letter
	return last_pose; // removed + 1? 
}


/*
Sets the name for the object being scanned / data being collected.
	- creates the necessary output folders
	- sets the global "scan_folder"
	- Opens the debug log for that object.
	- creates and object_info.json template file
	- determines the last pose
	- updates the moad config file with the new object name / pose
*/
void setObjectName(std::string object_name) {
	ConfigHandler& config = ConfigHandler::getInstance();

	DebugUtils::logInfo("Setting object name to: " + object_name);

	// Get the scan folder path
	std::string output_dir = config.getValue<std::string>("output_dir");
	scan_folder = output_dir + PATH_SEP + object_name;

	// Create base Object data Folder
	create_folder(scan_folder);

	// Start new debug log within the object folder
	std::string debug_log_dir = scan_folder + PATH_SEP + "debug_log.txt";
	DebugUtils::initLogFile(debug_log_dir);

	// Create object info.json (template)
	object_info["Object Name"] = object_name;
	create_obj_info_json(config.getValue<std::string>("output_dir"), object_info["Object Name"]);
	DebugUtils::logWhitespace();

	// save the object name to the moadfig file
	DebugUtils::logInfo("Updating moad_config.json file...");
	config.writeObjectName(object_name);

	// Change pose
	// POSSIBLE EXTRA POSE CHANGE? - GS 8/7
	curr_pose = get_last_pose();
	object_info["Pose"] = curr_pose;
	// write_pose_to_moadfig(curr_pose); // write pose to moadfig
	config.writePose(curr_pose);
	create_folder(scan_folder + PATH_SEP + "pose-" + curr_pose);

	// Update the save directories for RealSense and DSLR (folders are created upon data collection)
	rshandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "realsense";
	canonhandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "DSLR";
}

bool setObjectName() {
	// Prompt for object name 
	std::string object_name_input;
	std::cout << "\n\nEnter Object Name: ";
	std::cin >> object_name_input;

	// Set the object name using the given input
	setObjectName(object_name_input);

	return false;
}

bool setPose() {
	ConfigHandler& config = ConfigHandler::getInstance();
	char last_pose = get_last_pose();
	std::cout << "Enter Pose (Last pose: '" << last_pose << "'): ";
	std::cin >> curr_pose;
	object_info["Pose"] = curr_pose;
	// write_pose_to_moadfig(curr_pose); // write pose to moadfig
	config.writePose(curr_pose);


	return false;
}

/* ----------------------------------------------------------------------
	CAMERA BUTTON CONTROL
---------------------------------------------------------------------- */

bool pressHalfway() {
	PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
	return false;
}

bool pressCompletely() {
	PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Completely);
	return false;
}

bool pressOff() {
	PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_OFF);
	return false;
}


/* ----------------------------------------------------------------------
	CAMERA SETTINGS MENUS
---------------------------------------------------------------------- */

void _getCurrCameraValue(const std::vector<std::string>& value_arr) {
	tabulate::Table table;
	table.add_row({"Current Value:"});
	tabulate::Table cam_val_table;
	tabulate::Table::Row_t values;

	// Create a row for camera names
	for (const auto& name: canonhandle.camera_names) {
		values.push_back(name.first);
	}
	cam_val_table.add_row(values);
	
	values.clear();

	// Create a row for camera values
	for (const auto& value: value_arr) {
		values.push_back(value);
	}
	cam_val_table.add_row(values);
	table.add_row({cam_val_table});

	std::cout << table << std::endl; 
}

bool setTV() {
	// Check if the operation will be applied to all cameras
	if (all_cameras) {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table, out_table);
		
		// Get the actual camera value
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, out_table, value_arr);
		
		// Display the current camera value
		_getCurrCameraValue(value_arr);
		
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		std::cout << "input no. (ex. 54 = 1/250)" << std::endl;
		std::cout << ">";
		
		// Get user input for the new value
		int validReceivedValue = getvalue(); // NOTE: this is lowercase because theres another function getValue()
									 // 	i did not write this and am pretty annoyed. TODO: fix.
		if (validReceivedValue != -1) {	
			canonhandle.data = validReceivedValue;
			// Set the new value for all cameras
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, canonhandle.data, out_table);
		}
	}
	else {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, tv_table, out_table);
		
		// Get the actual camera value
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, out_table, value);
		
		std::cout << "Modifing " << canonhandle.camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		std::cout << "input no. (ex. 54 = 1/250)" << std::endl;
		std::cout << ">";
		
		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, canonhandle.data, out_table);
		}
		
	}

	return false;
}

bool setAV() {
	if(all_cameras) {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table, out_table);
		
		// Get the actual camera value
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, out_table, value_arr);
		
		// Display the current camera value
		_getCurrCameraValue(value_arr);
		
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		std::cout << "input Av (ex. 21 = 5.6)" << std::endl;
		std::cout << ">";
		
		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, canonhandle.data, out_table);
		}

		return false;
	}
	else {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_Av, av_table, out_table);
		
		// Get the actual camera value
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Av, out_table, value);
		

		std::cout << "Modifing " << canonhandle.camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		std::cout << "input Av (ex. 21 = 5.6)" << std::endl;
		std::cout << ">";


		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Av, canonhandle.data, out_table);
		}

	
		return false;
	}
}

bool setISO() {
	EdsError err;
	if (all_cameras) {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table, out_table);

		// Get the actual camera value
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, out_table, value_arr);
		
		// Display the current camera value
		_getCurrCameraValue(value_arr);

		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		std::cout << "input ISOSpeed > ";


		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			err = SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, out_table);
			std::cout << "setISO error: " << err << std::endl;
		}

		return false;
	}
	else {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, iso_table, out_table);
		
		// Get the actual camera value
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, out_table, value);
		
		std::cout << "Modifing " << canonhandle.camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		std::cout << "input ISOSpeed (ex. 8 = ISO 200)" << std::endl;
		std::cout << ">";


		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, canonhandle.data, out_table);
		}


		return false;
	}
}

bool setWhiteBalance() {
	if (all_cameras) {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table, out_table);

		// Get the actual camera value
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, out_table, value_arr);
		
		// Display the current camera value
		_getCurrCameraValue(value_arr);

		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		std::cout << "input WhiteBalance (ex. 0 = Auto)" << std::endl;
		std::cout << ">";
		

		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, canonhandle.data, out_table);
		}


		return false;
	}
	else {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, whitebalance_table, out_table);

		// Get the actual camera value
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, out_table, value);
		
		std::cout << "Modifing " << canonhandle.camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		std::cout << "input WhiteBalance (ex. 0 = Auto)" << std::endl;
		std::cout << ">";
		


		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, canonhandle.data, out_table);
		}


		
		return false;
	}
}

bool setDriveMode() {
	if (all_cameras) {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table, out_table);
		
		// Get the actual camera value
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, out_table, value_arr);
		
		// Display the current camera value
		_getCurrCameraValue(value_arr);
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		std::cout << "input Drive Mode (ex. 0 = Single shooting)" << std::endl;
		std::cout << ">";
		

		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, canonhandle.data, out_table);
		}


		
		return false;
	}
	else {
		// Get accepted property values
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, drivemode_table, out_table);

		// Get the actual camera value
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, out_table, value);
		
		std::cout << "Modifing " << canonhandle.camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		std::cout << "input Drive Mode (ex. 0 = Single shooting)" << std::endl;
		std::cout << ">";
		


		// Get user input for the new value
		int validReceivedValue = getvalue();

		if (validReceivedValue != -1) {
			canonhandle.data = static_cast<EdsUInt32>(validReceivedValue);
			// Set the new value for the active camera
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, canonhandle.data, out_table);
		}


		return false;
	}
}

// DEPRICATED: This function does not do anything at the moment 
bool setAEMode() {
	// That does nothing at the moment
	std::map<EdsUInt32, const char*> out_table;
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table, out_table);
	std::vector<std::string> value_arr;
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, out_table, value_arr);

	_getCurrCameraValue(value_arr);

	return false;
}

// Sets which cameras have their settings changes in the menus.
// Useful for changing the settings on a single camera.
bool changeCamera() {
	int option = -1;
	std::cout << "Change Camera. Select which camera to use (0 = all camera):" << std::endl;
	int i = 1;
	
	// Display all available cameras 
	std::cout << "0. All cameras" << std::endl;
	for (const auto& camera : canonhandle.cameraArray) {
		std::cout << i << ". " << canonhandle.camera_name[camera] << std::endl;
		i++; 
	}
	option = getvalue();

	// Switch to the selected camera
	all_cameras = false;
	switch (option){
		case 0:
			all_cameras = true;
			break;
		case 1: 
			activeCamera = canonhandle.cameraArray[0];
			break;
		case 2: 
			activeCamera = canonhandle.cameraArray[1];
			break;
		case 3: 
			activeCamera = canonhandle.cameraArray[2];
			break;
		case 4: 
			activeCamera = canonhandle.cameraArray[3];
			break;
		case 5: 
			activeCamera = canonhandle.cameraArray[4];
			break;
		default:
			std::cout << "Invalid value" << std::endl;
			break;
	}
	return false;
}


/* ----------------------------------------------------------------------
	LIVE VIEW MENUS
---------------------------------------------------------------------- */

bool getLiveView() {
	ConfigHandler& config = ConfigHandler::getInstance();

	// setup directory for filestream output (otherwise it creates wherever the
	//	MultiCamCui exec is run out of)
	std::string liveViewDir = moad_dir + PATH_SEP + "live_view_filestream" + PATH_SEP;


	if (!config.getValue<bool>("dslr.enable_collection")){
		DebugUtils::logError("DSLR option is set to false, Liveview unavaliable.");
		return false;
	}

	if (!liveview_active) {
		// Start Live View configuration
		DebugUtils::logInfo("Starting Live View...");
		DebugUtils::logTimestamp("Starting live view.");

		// liveview_active = true;
		liveview_thread_id = std::this_thread::get_id();

		// Change camera configuration to liveview
		StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
		std::this_thread::sleep_for(.5s);
		liveview_active = true;
		
		// Download and display Image from camera 
		int i = 0;
		for (auto& camera : canonhandle.cameraArray) {
			// Download the image from the camera
			liveview_th.push_back(std::thread([&]() {
				DownloadEvfCommand(camera, liveViewDir + canonhandle.camera_name[camera], liveview_thread_id);
			}));
			std::this_thread::sleep_for(.5s);
			i++;
		}
		DebugUtils::logDebug("Live View Threads Created...");
	}
	else {
		DebugUtils::logWarning("Live View is already active.");
	}

	return true;
}

bool endLiveView() {
	ConfigHandler& config = ConfigHandler::getInstance();
	// Check if the DSLR option is enabled
	if (!config.getValue<bool>("dslr.enable_collection")) {
		DebugUtils::logError("DSLR option is set to false, Liveview unavaliable.");
		return false;
	}
	DebugUtils::logInfo("Ending Live View...");
	DebugUtils::logTimestamp("Ending live view.");

	// Join all liveview threads
	liveview_active = false;
	for (auto& th : liveview_th) {
		if (th.joinable()) {
			th.join();
		}
	}
	// Clear the thread vector
	liveview_th.clear();

	// Revert camera configuration to normal mode
	EndEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	DebugUtils::logDebug("Liveview sucessfully closed.");
	return false;
}

bool liveViewMenu() {
	MenuHandler live_view_menu({
		{"1", "Start Live View"},
		{"2", "End Live View"},
	},{
		{"1", getLiveView},
		{"2", endLiveView},
	}, object_info);
	live_view_menu.setTitle("Live View Menu");
	live_view_menu.initialize(curr_menu);
	return true;
}


/* ----------------------------------------------------------------------
	TURNTABLE FUNCTIONS
---------------------------------------------------------------------- */
// NOTE: The function to send serial commands that actually move the turntable
// are defined in ScanManager.cpp

// Gets user input to manually move the turntable
bool turntableControl() {
	DebugUtils::logInfo("Entered Manual Turntable Control");
	std::string degree_inc;
	while(degree_inc != "r"){
		// Prompt for degrees to move
		std::cout << "Enter degrees to move (r = Return): ";
		std::cin >> degree_inc;

		// Check if the input is "r" to return
		if (degree_inc == "r")
			break;
		
		// TODO: Validate this input before sending it over serial
		// Send motor command over serial (Defined in ScanManager.cpp)
		rotate_turntable(std::stoi(degree_inc));

		// Update degree tracker
		degree_tracker += std::stoi(degree_inc);
		degree_tracker = degree_tracker % 360;
	}

	// Update the object info with the new turntable position
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	return false;
}

// Sets the INTERNAL turntable position (does not move the turntable)
bool turntablePosition() {
	// Prompt for the turntable position
	std::cout << "\n\nEnter Turntable Position: ";
	std::cin >> degree_tracker;

	// Update the turntable position
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Add a success message to the current menu
	curr_menu->addMessage(MenuMessageStatus::SUCCESS, "Turntable Position updated sucessfully");
	
	return false;
}


// DEFINE SUBMENU OPTIONS =============================================================================
// Calibration submenu for controlling camera shooting button
bool CalibrationSubMenu(){
	MenuHandler calibration_menu_handler({
		{"1", "Press Halfway"},
		{"2", "Press Completely"},
		{"3", "Press Off"}
	},{
		{"1", pressHalfway},
		{"2", pressCompletely},
		{"3", pressOff}
	}, object_info);
	calibration_menu_handler.setTitle("Calibration Menu");
	calibration_menu_handler.initialize(curr_menu);
	return true;
}
// Camera submenu for controlling camera settings
bool CameraSubmenu(){
	MenuHandler camera_menu_handler({
		{"1", "TV"},
		{"2", "AV"},
		{"3", "ISO"},
		{"4", "White Balance"},
		{"5", "Drive Mode"},
		{"6", "AE Mode"},
		{"7", "Change Camera"},
	},{
		{"1", setTV},
		{"2", setAV},
		{"3", setISO},
		{"4", setWhiteBalance},
		{"5", setDriveMode},
		{"6", setAEMode},
		{"7", changeCamera},
	}, object_info);
	camera_menu_handler.setTitle("Camera Options");
	camera_menu_handler.initialize(curr_menu);
	return true;
}
// Turntable submenu for manually controlling turntable position and internal position tracking state
bool TurntableSubMenu(){
	MenuHandler turntable_handler({
		{"1", "Turntable Control"},
		{"2", "Turntable Position"},
	},{
		{"1", turntableControl},
		{"2", turntablePosition},
	}, object_info);
	turntable_handler.setTitle("Turntable Options");
	turntable_handler.initialize(curr_menu);
	return true;
}


/* ----------------------------------------------------------------------------------
	MAIN FUNCTION
---------------------------------------------------------------------------------- */
int main(int argc, char* argv[]) 
{	
	DebugUtils::logInfo("START OF MAIN");

	// SETUP ----------------------------------------------------------------------------------------------
	// NOTE: degree_tracker is a global variable so that every function can use it
	//		initially it is set to 0, but in order to save state for potential cam
	//		interrupts, it needs to be loaded from config
	degree_tracker = 0;

	// use "fs::canonical" to get the moad directory FROM THE MultiCamCui EXECUTABLE
	//	this is meant to avoid direct pathing and get the config from the moad_cui directory
	// NOTE: this is a global variable...
	moad_dir = fs::canonical(argv[0]).parent_path().parent_path();

	// Load the JSON Config file
	// NOTE: this a global variable...
	json_path = (moad_dir + PATH_SEP + "config" + PATH_SEP + "moad_config.json");
	// Provides global config access
	ConfigHandler& config = ConfigHandler::getInstance();
	// Loads json config into handler object
	config.loadConfig(json_path);

	// Set the object name from the configuration
	/* NOTE: the setObjectName function is currently quite overloaded. It set's the 
	internal object name, but also handles setting/creating global output folders for that object,
	creating an object_info.json template file, and starting a new Debug Log in that object folder. */
	std::string object_name = config.getValue<std::string>("object_name");
	setObjectName(object_name);
	DebugUtils::logWhitespace();

	// Initialize Sensors (internally checks the config to decide whether to really connect)
	canonhandle.initialize();
	rshandle.initialize(json_path);

	// Setup Arduino serial port connection
	std::string com_port;
	com_port = config.getValue<std::string>("turntable_control.serial_com_port");

	typedef unsigned long DWORD;
	DWORD COM_BAUD_RATE = B9600; // Serial baud rate currently hardcoded

	DebugUtils::logInfo("Attempting Serial Port Connection...");
	Serial = new SimpleSerial(com_port.c_str(), COM_BAUD_RATE); // input must be c string

	// Log some additional information at startup
	DebugUtils::logDebug("Current Object Name: " + object_info["Object Name"]);
	DebugUtils::logDebug("Current Pose: " + object_info["Pose"]);
	DebugUtils::logDebug("Current Turntable Position: " + object_info["Turntable Pos"]);
	DebugUtils::logDebug("Current Scan Folder: " + scan_folder);
	DebugUtils::logDebug("Current JSON Path: " + config.getConfigPath());
	DebugUtils::logTimestamp("Ready.");

	// Main Menu initialization - maps menu options to function calls
	MenuHandler menu_handler({
		{"1", "Full Scan"},
		{"2", "Custom Scan"},
		{"3", "Collect Single Data"},
		{"4", "Set Object Name"},
		{"5", "Set Pose"},
		{"6", "Camera Calibration..."},
		{"7", "Camera Options..."},
		{"8", "Turntable Options..."},
		{"9", "Live View..."},
		{"p", "Scan from saved state"},
		{"0", "Reload Config"},
		{"f", "Run Filecount Check Script"}
	},
	{
		{"1", fullScan},
		{"2", customScan},
		{"3", collectSampleData},
		{"4", setObjectName},
		{"5", setPose},
		{"6", CalibrationSubMenu},
		{"7", CameraSubmenu},
		{"8", TurntableSubMenu},
		{"9", liveViewMenu},
		{"p", scanFromSaveState}, // TODO: "10" seemed to not work??
		{"0", reloadConfig},
		{"f", runFilecountCheck}
	}, object_info);
	menu_handler.setTitle("MOAD - CLI Menu");
	menu_handler.ClearScreen();

	// Run the CUI Interface
	menu_handler.initialize(curr_menu);

	// Cleanly shutdown sensor handlers
	DebugUtils::log("END","Shutting down...\n",0,true);
	canonhandle.shutdown();
	rshandle.shutdown();

	// Close log file
	DebugUtils::closeLogFile();
	DebugUtils::notifyConfigReady(false);

	return false;
}