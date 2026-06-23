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

#include "MenuHandler.h"
#include "ConfigHandler.h"
#include "CanonHandler.h"
#include "RealSenseHandler.h"
#include "ThreadPool.h"
#include "DebugUtils.h" // used with logfile global variable for now - GS 8/12

// #include <windows.h> // commented out for linux port
#include "tabulate.hpp"
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "Download.h"
#include "DownloadEvf.h"
#include "PreSetting.h"
#include "PressShutter.h"
#include "Property.h"
#include "TakePicture.h"

#include "SerialCommunication.h" 
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
std::map<EdsCameraRef, std::string> camera_name;
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
void initializeCanon();
void initializeRealsense();
bool WaitForCameraReady(EdsCameraRef camera, int timeoutMs);
void rotate_turntable(int degree_inc);
bool run_filecount_check(); // Calls ./scripts/filecount_test.py (Copying/downscaling images for NeRF)
int create_folder(std::string path, bool quiet);



// functions for writing to config.json ==========================================================================================================
//   TODO: can be one function? deterministic on args?
//	 NOTE: REWRITES the config each time, though should only change 1 value each call

// void write_obj_to_moadfig(std::string object_name);
void write_obj_to_moadfig(std::string object_name) {
	std::ifstream config_file(json_path);

	// std::cout << "Writing object to moadfig, path is " << json_path << std::endl;
	DebugUtils::logConfig("Updating Object Name: " + json_path);

	ordered_json config_json;

	if (config_file.is_open()) {
		config_file >> config_json;
		config_file.close();
	} else {
		// std::cerr << "Failed to open config file: " << json_path << std::endl;
		DebugUtils::logError("Failed to open config file: " + json_path);
		return;
	}

	// set object name
	config_json["object_name"] = object_name;

	// write out
	std::ofstream out_file(json_path);
	if (out_file.is_open()) {
		out_file << config_json.dump(4);
		out_file.close();
		// std::cout << "Updated object_name in moad_config.json" << std::endl;
		DebugUtils::logConfig("Updated Object Name: " + object_name);
	} else {
		// std::cerr << "Failed to write to config file: " << json_path << std::endl;
		DebugUtils::logError("Failed to write to config file: " + json_path);
	}
}

// void write_pose_to_moadfig(char pose);
void write_pose_to_moadfig(char pose) {
	std::ifstream config_file(json_path);

	// std::cout << "Writing pose to moadfig, path is " << json_path << std::endl;
	DebugUtils::logConfig("Updating Previous Pose: " + json_path);

	ordered_json config_json;
	if (config_file.is_open()) {
		config_file >> config_json;
		config_file.close();
	} else {
		// std::cerr << "Failed to open config file: " << json_path << std::endl;
		DebugUtils::logError("Failed to open config file: " + json_path);
		return;
	}

	// set object name
	config_json["prev_state"]["current_pose"] = pose;
	std::string pose_str{pose};

	// write out
	std::ofstream out_file(json_path);
	if (out_file.is_open()) {
		out_file << config_json.dump(4);
		out_file.close();
		// std::cout << "Updated prev_state.current_pose in moad_config.json" << std::endl;
		DebugUtils::logConfig("Updated prev_state.current_pose in moad_config.json: " + pose_str);
	} else {
		// std::cerr << "Failed to write to config file: " << json_path << std::endl;
		DebugUtils::logError("Failed to write to config file: " + json_path);
	}
}

// void write_degree_move_to_moadfig(int degree, int current_move); // template this later for int item
void write_degree_move_to_moadfig(int degree, int current_move) {
	
	std::ifstream config_file(json_path);

	DebugUtils::logConfig("Updating Previous Turntable Position: " + json_path);

	ordered_json config_json;
	if (config_file.is_open()) {
		config_file >> config_json;
		config_file.close();
	} else {
		DebugUtils::logError("Failed to open config file: " + json_path);
		return;
	}

	// Set prev_state.turntable_pos
	config_json["prev_state"]["turntable_pos"] = degree;

	// Set prev_state.current_move to current_move
	config_json["prev_state"]["current_move"] = current_move;

	// write out
	std::ofstream out_file(json_path);
	if (out_file.is_open()) {
		out_file << config_json.dump(4);
		out_file.close();
		// std::cout << "Updated prev_state.turntable_pos in moad_config.json" << std::endl;
		DebugUtils::logConfig("Updated prev_state.turntable_pos/current_move in moad_config.json:: " + std::to_string(degree) + " / " + std::to_string(current_move));
	} else {
		// std::cerr << "Failed to write to config file: " << json_path << std::endl;
		DebugUtils::logError("Failed to write to config file: " + json_path);
	}

}



/* 	
Handles loading/re-loading the moadconfig. If it's called during runtime after the config
has already been loaded, it checks whether DSLR/RS collection has been enabled and if so,
initializes the objects responsible for handling their data collection. 

	calls:
		- loadConfig()
		- initializeCanon() - if it's been enabled
		- initializeRealsense() - if it's been enabled
		- setObjectName()
*/
void loadJsonConfig(std::string path) {
	
	// If config was already loaded, save current state of collect DSLR/RS.
	ConfigHandler& config = ConfigHandler::getInstance();
	std::optional<bool> previous_DSLR;
	std::optional<bool> previous_RS;
	
	if (!config.emptyConfig()) {
		DebugUtils::logDebug("Checking previously loaded config...");
		previous_DSLR = config.getValue<bool>("dslr.collect_dslr");
		previous_RS = config.getValue<bool>("realsense.collect_realsense");
	}

	// Reload the config .json file to update all setting values
	// std::cout << "Path in loadJsonConfig(): " << path << std::endl;
	config.loadConfig(path);

	// Check if the configuration has changed for DSLR, if so, initialize Canon Handler
	if (!previous_DSLR.has_value() || previous_DSLR != config.getValue<bool>("dslr.collect_dslr")) {
		// std::cout << "Initializing CanonHandler" << std::endl;
		if (config.getValue<bool>("dslr.collect_dslr") && !canonhandle.isSDKLoaded) {
			initializeCanon();
			DebugUtils::logWhitespace();
		}
	}
	
	// Check if the configuration has changed for RealSense, if so, initialize RS handler
	if (!previous_RS.has_value() || previous_RS != config.getValue<bool>("realsense.collect_realsense")) {
		// std::cout << "Initializing RealSenseHandler" << std::endl;
		if (config.getValue<bool>("realsense.collect_realsense")) {
			initializeRealsense();
			DebugUtils::logWhitespace();
		}
	}
	

	// Set the object name from the configuration
	std::string object_name = config.getValue<std::string>("object_name");
	setObjectName(object_name);
	DebugUtils::logWhitespace();
}

bool reloadConfig() {
	if (liveview_active) {
		std::cout << "Liveview is active, please stop it before reloading the config." << std::endl;
		return false;
	}

	loadJsonConfig(json_path);
	return false;
}


void saveCameraConfig(std::string path) {
DebugUtils::logFileSys("Saving camera_config.json to : " + path);
// std::cout << "path in saveCamConfig: " + path << std::endl;// declaration was missing before 10/29 - GS

	std::vector<std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>>> propertyIDs = {
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_ISOSpeed, iso_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Tv, tv_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Av, av_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_WhiteBalance, whitebalance_table),
	};

// std::cout << "getting confighandler: " << std::endl;
	ConfigHandler& config = ConfigHandler::getInstance();
// std::cout << "successfully got instance " << std::endl;
	nlohmann::json json_data;
// std::cout << "successfulyl got json data" << std::endl;


	// check camera array to prevent segfault:
	

	// Get the model and focal length for each camera
	for (auto& camera : canonhandle.cameraArray) {

		// segfaults since camera is empty
		std::string cam = camera_name[camera];
		DebugUtils::logDebug("Getting config for " + cam);
		// std::cout << "camera " + cam << std::endl;

		EdsDeviceInfo deviceInfo;
		EdsGetDeviceInfo(camera, &deviceInfo);

		json_data[cam]["Model"] = deviceInfo.szDeviceDescription; 
		json_data[cam]["Focal Length"] = config.getValue<std::string>("transform_generator.calibration_mode");
	}

	// Get the configuration properties for each camera
	for (auto propertyID : propertyIDs) {
		std::string name = std::get<0>(getPropertyString(std::get<0>(propertyID)));
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, std::get<0>(propertyID), std::get<1>(propertyID), out_table, false);
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, std::get<0>(propertyID), out_table, value_arr);
		
		for (auto& camera : canonhandle.cameraArray) {
			std::string cam = camera_name[camera];
			json_data[cam][name] = value_arr[0];
		}
	}

	// Save the file as a JSON file
	std::string output_file = "/camera_config.json";
    std::ofstream file(path + output_file);
    if (file.is_open()) {
        file << json_data.dump(4); // Pretty print with 4 spaces indentation
        file.close();
		DebugUtils::logDebug("Camera configuration saved to " + output_file);
        // std::cout << "Camera configuration saved to " << output_file << std::endl;
    } else {
		DebugUtils::logError("Failed to open file for writing: " + output_file);
        // std::cerr << "Failed to open file for writing: " << output_file << std::endl;
	}
}

void saveScanTime(std::chrono::milliseconds duration, std::string path) {
	// Get the given duration in minutes and seconds
	std::string minutes = std::to_string(duration.count() / 60000);
	std::string seconds = std::to_string((duration.count() % 60000) / 1000);

	// Open the JSON file for writing
	std::string full_path = path + PATH_SEP + "camera_config.json";
	std::ifstream file(full_path);
	if (!file.is_open()) {
		// std::cerr << "Failed to open file for writing: " << full_path << std::endl;
		DebugUtils::logError("Failed to open file for writing: " + full_path);
		return;
	}
	nlohmann::json json_data;
	file >> json_data;

	// Add the scan time to the JSON data
	json_data["Scan Time"] = minutes + ":" + seconds;

	// Save the updated JSON data back to the file
	std::ofstream output_file(full_path);
	if (output_file.is_open()) {
		output_file << json_data.dump(4); // Pretty print with 4 spaces indentation
		output_file.close();
		DebugUtils::logDebug("Scan time saved to camera_config.json");
		// std::cout << "Scan time saved to camera_config.json" << std::endl;
	} else {
		DebugUtils::logError("Failed to open file for writing: " + full_path);
		// std::cerr << "Failed to open file for writing: " << full_path << std::endl;
	}

	// Close the file
	file.close();
}


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



// definition
// takes string of path and bool for printout statements
// also checks if path already exists before attempting to create
int create_folder(std::string path, bool quiet=false) {

	// yellow colored text to notify of folder creations:
    // std::cout << "\033[1;33m" << "creating folder: " << path << "\033[0m" << std::endl;
	DebugUtils::logFileSys("Creating Folder: " + path);

	if (!fs::exists(path)) {
        // Create the folder and any necessary higher level folders
        if (fs::create_directories(path)) {
			DebugUtils::logFileSys("Folder created: " + path);
            //  std::cout << "\033[1;33m" << "Folder created: " << path << "\033[0m\n";
        } else {
			DebugUtils::logError("Failed to create folder: " + path);
            // std::cerr << "Failed to create folder: " << path << std::endl;
            return 1;
        }
    } else { 
		// DebugUtils::logFileSys("Folder already exists: " + path);
		DebugUtils::logWarning("Folder already exists: " + path + " (You may accidentally overwrite data.)");
		// std::cout << "\033[1;33m" << "WARNING!: Folder already exists: " << path << std::endl
		// 	<< "\tYou may accidentally overwrite data.\n" << "\033[0m\n";
    }
	return 0;
}


// TODO: Move to python script file
void create_obj_info_json(std::string path) {
	std::string object_name = object_info["Object Name"];
	DebugUtils::logInfo("Creating object info JSON file: " + object_name);
	// std::cout << "Creating object info JSON file: " << object_name << std::endl;

	// Check if the path exists
	if (fs::exists(path)){
		std::string full_path = path + PATH_SEP + object_name + PATH_SEP + "object_info.json";
		if(fs::exists(full_path)){
			DebugUtils::logDebug("Object info file already exists. Skipping...");
			return;
		}
		// Generate the command to execute scripts\create_object_info.py
		std::stringstream command_stream;
		command_stream 
			<< "python3 " 
			<< "scripts/create_object_info.py "
			<< object_name << " "
			<< "-p " << path << " ";

		// Execute command
		std::string command = command_stream.str(); 
		const char* c_command = command.c_str();
		// std::cout << "\nExecuting Command: " << command; 
		system(c_command);
	}
	else {
		DebugUtils::logWarning("Folder \'" + path + "\' does not exist.");
		// std::cout << "WARNING: folder " << path << " does not exist." << std::endl;
	}
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
Master scan function which is called by other scan functions (full,custom,from state)
	- Creates necessary output folders depending on the data being collected
	- Collects a single data frame from each enabled sensor
*/
bool scan(ThreadPool* pool = nullptr) {
	// Get current config and setup local variables
	ConfigHandler& config = ConfigHandler::getInstance();
	std::string object_name = config.getValue<std::string>("object_name");
	std::string output_dir = config.getValue<std::string>("output_dir");
	scan_folder = output_dir + PATH_SEP + object_name;

	// thread locking and timings to discern every piece of thread usage
	bool safe_take_picture = false;
	if (config.getValue<bool>("dslr.safe_take_picture")) {
		// std::cout << "Safe TakePicture() is enabled." << std::endl;
		DebugUtils::logConfig("Safe TakePicture() is enabled.");
		safe_take_picture = true;
	}

	DebugUtils::logDebug("entering scan() code under location " + scan_folder);

	// Collect RealSense Data
	if(config.getValue<bool>("realsense.collect_realsense")) {

		// Create RS Scan Folder
		rshandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "realsense";

		create_folder(rshandle.save_dir, true);

		DebugUtils::logDebug("Getting RealSense Data...");

		// Get the current frame from RealSense
		int rs_timeout = config.getValue<int>("realsense.realsense_timeout_sec") * 1000;
		rshandle.turntable_position = degree_tracker;
		rshandle.get_current_frame(degree_tracker, rs_timeout, pool);

		DebugUtils::logDebug("RealSense frame captured successfully.");

		if (rshandle.fail_count > 0) {
			std::cout << "RS Failure - " << rshandle.fail_count << std::endl;
			DebugUtils::logError("RealSense failed to get frames.");
			return false; // Return false if RealSense failed to get frames
		}
	}


	// Collect DSLR Data
	if(config.getValue<bool>("dslr.collect_dslr")) {
		canonhandle.images_downloaded = 0;
		canonhandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "DSLR";

		create_folder(canonhandle.save_dir,true);
	
		// std::cout << "Getting DSLR Data...\n";
		DebugUtils::logDebug("Getting DSLR Data...");
		canonhandle.turntable_position = degree_tracker;

		// Create thread vector
		std::vector<std::thread> threads;

		for (auto& camera : canonhandle.cameraArray) {
			std::string cam_name = camera_name[camera];

			threads.emplace_back([&, camera, cam_name]() {
				EdsError err = EDS_ERR_OK;

				if (safe_take_picture)
					err = TakePicture(camera, cam_name);
				else
					err = TakePictureNoWait(camera, cam_name);

				if (err != EDS_ERR_OK)
					// std::cout << "Error taking picture with camera: " << cam_name << std::endl;
					DebugUtils::logError("Error taking picture with camera: " + cam_name );

				// DebugUtils::logPictureLoop("running DSLR picture loop, : " + std::to_string(err));
			});
		}

		// Wait for all threads to finish
		DebugUtils::logDebug("Waiting for threads to join...");
		for (auto& t : threads) {
			t.join();
		}

		// Wait until all Canon images have been downloaded.
		while(canonhandle.images_downloaded != canonhandle.cameras_found) {
			EdsError err = EDS_ERR_OK;
			err = EdsGetEvent(); // NOTE: this does not save the cmaeras, this just asks for current event
			// std::cout << err; // this line meant to prevent annoying unsed variable error
			DebugUtils::logDebug("Waiting for Canon images... (Error = " + std::to_string(err) + ")");
			std::this_thread::sleep_for(100ms);
			// std::cerr << "waiting: " << canonhandle.images_downloaded << "/" << canonhandle.cameras_found << " images downloaded" << std::endl;
		}
	}
	return true;
}


/*
Writes the proper command over serial connection to move the turntable a specified number of degrees.
TODO: Currently has hardcoded wait time, and I think it just continues if it times out, which breaks data collection for large 
		degree increments. Need to test this.
*/
void rotate_turntable(int degree_inc) {

	std::string degree_inc_str = std::to_string(degree_inc);

	char *send = &degree_inc_str[0];

	// actual command sent to arduino, must be sent as a string/char
	bool is_sent = Serial->WriteSerialPort(send);

	if (is_sent) {


		// int wait_time = std::ceil(((abs(degree_inc)*200)+500)/1000) + 5; // faster????
		int wait_time = 30; // 2 is probably fastest/safest
		// std::cout << "Message sent, moving: " << degree_inc << " degrees, waiting up to " << wait_time << " seconds.\n";
		DebugUtils::logTurntable("Message sent: MOVING " + std::to_string(degree_inc) + " degrees, WAITING up to " + std::to_string(wait_time) + " seconds...");
		
		std::string incoming = Serial->ReadSerialPort(wait_time, "json");
		// std::cout << "Incoming: " << incoming;// << std::endl;
		DebugUtils::logTurntable("Read from turntable serial: " + incoming);
		
		// DebugUtils::logTurntable("Message sent, moving: " + std::to_string(degree_inc) + " degrees, waiting up to " + std::to_string(wait_time) + " seconds.");
		// std::this_thread::sleep_for(250ms);
		
		// int turntable_delay_ms = config.getValue<int>("turntable_delay_ms");
		// std::this_thread::sleep_for(std::chrono::milliseconds(turntable_delay_ms));

	} else {
		// std::cout << "WARNING: Serial command not sent, something went wrong.\n";
		DebugUtils::logTurntable("WARNING: Serial command not sent, something went wrong.");
	}
}




bool generateTransform(int degree_inc, int num_moves) {
	DebugUtils::logWhitespace();
	DebugUtils::logInfo("Generating transforms.json...");
	ConfigHandler& config = ConfigHandler::getInstance();

	// Collect parameters from config
	bool force = config.getValue<bool>("transform_generator.force");
	bool visualize = config.getValue<bool>("transform_generator.visualize");
	std::string calibration_dir = config.getValue<std::string>("transform_generator.calibration_dir");
	std::string calibration = config.getValue<std::string>("transform_generator.calibration_mode");
	std::string output_dir = config.getValue<std::string>("output_dir");
	std::string object_name = config.getValue<std::string>("object_name");

	// Prepare command to execute scripts/transform_generator.py
	int range = degree_inc * num_moves;
	std::stringstream command_stream;
	command_stream 
		<< "python3 " 
		<< "scripts/transform_generator.py " // relative path
		<< object_name << " "
		<< "-d " << degree_inc << " "
		<< "-r " << range << " "
		<< "-c " << calibration << " "
		<< "--calibration_dir " << calibration_dir << " "
		<< "-p " << output_dir << " "
		<< "--pose " << "pose-" << curr_pose;
 
	if (visualize) {
		command_stream << " -v";
	}

	if (force) {
		command_stream << " -f";
	}

	// Execute command
	std::string command = command_stream.str(); 
	const char* c_command = command.c_str();
	DebugUtils::logInfo("Executing Command: " + command);
	std::this_thread::sleep_for(500ms);
	// std::cout << "\nExecuting Command: " << command; 
	system(c_command);

	return false;
}



/*
	COMMENTS FOR fullScan(), customScan(), and scanFromSaveState()
		args: none
		returns: previously void as of 7/24/2025, now bool (UPDATE THIS COMMENT LATER ONCE APPROPRIATE PARTIES NOTIFIED)

		or rather, three functions that run scan() and rotate_turntable()

	added interrupt for if NOT 5 images were downloaded
	this checks off of "canonhandle.images_downloaded" INSIDE scan(...) which i dont quite know where canonhandle 
		comes from… possibly a global variable somewhere? i looked for it for several seconds but couldnt find it

	considering the threading part, any attempt to exit or return in the scan() function itself didnt stop the 
		overall process, so this required me to change the scan() function to bool and just run the check in the loop 
		that calls the scan method

	this means that each time scan() is called, it will check if the return is false and break from the loop

	notably, due to placement of the check:
		- degree_tracker does not increment
		- turntable does not rotate
		- moad config file is not updated
		- images ARE saved
		- realsense data IS saved
		- transforms ARE generated
		- file count check IS run
	which is desired functionality (i think) since the scanFromSaveState function will pickup scanning and saving
		from the last successful state

	bugs:
		- degree tracker calculation outside of for loop?
		- possible index out of range error occurs at the end BUT as of 7/25 when i wrote this one of the cameras
			does not work so i havent managed to hit the end of the image collection

	- GS 7/25/2025

*/
bool fullScan() {
	ConfigHandler& config = ConfigHandler::getInstance();

	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	DebugUtils::logInfo("========Starting a full scan========");
	DebugUtils::logInfo("\n\tcurrent pose: " + std::to_string(curr_pose));

	// Create thread pool
	// currently unsafe i think, we also use another set of threads inside the scan() function itself
	//	that is DIFFERENT from the ThreadPool created here.
	// note quite sure the original purpose of ThreadPool tbh - gs 11/7
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);


	// zero the degree_tracker since we are starting a new scan...
	// ...not entirely sure if this is necessary, should only affect the naming scheme for the files?
	//	- gs 11/7
	degree_tracker = 0;


	int degree_inc = config.getValue<int>("degree_inc");
	int num_moves = config.getValue<int>("num_moves");
	
	// Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		DebugUtils::logWhitespace(2);
		DebugUtils::logDebug("Starting move " + std::to_string(rots + 1) + "/" + std::to_string(num_moves));

		if(scan(&pool) == false) {
			std::cerr << "\n============================================================" << std::endl;
			std::cerr << "!!! ERROR: scan() failed at move " << rots + 1 << "/" << num_moves << ". Aborting scan. !!!" << std::endl;
			std::cerr << "============================================================\n" << std::endl;
			// add delay

			DebugUtils::logError("Failed scan at move " + std::to_string(rots + 1) + " of " + std::to_string(num_moves));

			// std::this_thread::sleep_for(3000ms);
			break;
		}

		
		// IMPORTANT: rotation must be done AFTER saving occurs since the cameras MAY NEED LONGER
		//	to get full exposure
		rotate_turntable(degree_inc);
		
		// Update the degree tracker
		degree_tracker += degree_inc;

		write_degree_move_to_moadfig(degree_tracker, rots + 1);
		
		// std::cout << "Image " << rots+1 << "/" << num_moves << " taken. " << std::endl;
		DebugUtils::logDebug("Image " + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + " taken.");
	}


	// Stop the loop timer
	auto end = std::chrono::high_resolution_clock::now();
	// Calculate the elapsed time
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// Convert the duration to minutes, seconds, and milliseconds
	int minutes = duration.count() / 60000;
	int seconds = (duration.count() % 60000) / 1000;
	std::ostringstream oss;
	oss << "Scan Time: "
		<< std::setw(2) << std::setfill('0') << minutes << ":"
		<< std::setw(2) << std::setfill('0') << seconds << "  [mm:ss]";
	// std::cout << "Scan Time: " << std::setfill('0') << std::setw(2) << minutes << ":" 
	// << std::setfill('0') << std::setw(2) << seconds << std::endl;
	// std::cout << "RS Fail Count: " << rshandle.fail_count << std::endl;

	DebugUtils::logDebug(oss.str());
	DebugUtils::logDebug("RS Fail Count: " + std::to_string(rshandle.fail_count));
	DebugUtils::logWhitespace();
	DebugUtils::logInfo("=== Full Scan Finished ===");
	//std::this_thread::sleep_for(3000ms);
	
	// Save camera configurations in a json file
	// NOTE: this occurs at the END of the full rotation. this might cause saving bugs - GS 7/24
	// saveCameraConfig(scan_folder + "\\pose-" + curr_pose);
	// std::cout << "================\nsaved to" << scan_folder + "\\pose-" + curr_pose << "\n================\n";
	// saveScanTime(duration, scan_folder + "\\pose-" + curr_pose);

	// Save camera configurations in a json file
	if (config.getValue<bool>("dslr.collect_dslr")) {
		saveCameraConfig(scan_folder + PATH_SEP + "pose-" + curr_pose);
		saveScanTime(duration, scan_folder + PATH_SEP + "pose-" + curr_pose);
		if (config.getValue<bool>("transform_generator.enabled"))
			generateTransform(degree_inc, num_moves);
	}

	// Recalculate angle
	degree_tracker = degree_tracker % 360; // why is this needed? - GS 7/24
	std::cout << "\nRecalculated degree_tracker: " << degree_tracker << std::endl;
	
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++; // TODO: last pose QoL bug, this line potentially not needed?
	object_info["Pose"] = curr_pose;

	write_pose_to_moadfig(curr_pose); // write pose to moadfig
	
	run_filecount_check();

	MenuHandler::WaitUntilKeypress();

	DebugUtils::logInfo("======Full scan for pose " + std::to_string(curr_pose) + " completed.======");
	return true;
}


/*
Initiate a scan with terminal prompts to manually set the degrees per move and number of moves.
*/
bool customScan() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		DebugUtils::logWarning("Liveview is active, please stop it before scanning.");
		// std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	DebugUtils::logInfo("========Starting a custom scan========");
	DebugUtils::logInfo("Current pose: " + std::to_string(curr_pose));

	// Create the thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);
	int degree_inc;
	int num_moves = 0;

	// Ask for the user to input the degree increment and number of moves
	std::cout << "Enter degrees per move: ";
	std::cin >> degree_inc;
	std::cout << "Enter number of moves: ";
	std::cin >> num_moves;
	
	DebugUtils::logDebug("Custom scan with degree increment: " + std::to_string(degree_inc) + 
		" and number of moves: " + std::to_string(num_moves));

	// Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		DebugUtils::logWhitespace(2);
		DebugUtils::logInfo("Getting data " + std::to_string(rots + 1) + "/" + std::to_string(num_moves));

		// Scan function is given thread pool to collect all sensor data for a single frame, returns true when successful
		if(scan(&pool) == false) {
			std::cerr << "\n============================================================" << std::endl;
			std::cerr << "!!! ERROR: scan() failed at move " << rots + 1 << "/" << num_moves << ". Aborting scan. !!!" << std::endl;
			std::cerr << "============================================================\n" << std::endl;

			DebugUtils::logError("Failed scan at move " + std::to_string(rots + 1) + " of " + std::to_string(num_moves));

			// add delay
			std::this_thread::sleep_for(3000ms);
			break;
		}

		// std::cout << "Image " << rots+1 << "/" << num_moves << " taken. " << std::endl;
		DebugUtils::logInfo("Data frame " + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + " captured.");
		DebugUtils::logWhitespace(1);
		// Rotate the turntable
		rotate_turntable(degree_inc);
		
		// Update the degree tracker 
		degree_tracker += degree_inc;
	}

	// Stop the loop timer
	auto end = std::chrono::high_resolution_clock::now();
	// Calculate the elapsed time
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// Convert the duration to minutes, seconds, and milliseconds
	int minutes = duration.count() / 60000;
	int seconds = (duration.count() % 60000) / 1000;
	std::ostringstream oss;
	oss << "Scan Time: "
		<< std::setw(2) << std::setfill('0') << minutes << ":"
		<< std::setw(2) << std::setfill('0') << seconds << "  [mm:ss]";
	// Log time and fail count
	DebugUtils::logDebug(oss.str());
	DebugUtils::logDebug("RS Fail Count: " + std::to_string(rshandle.fail_count));
	DebugUtils::logWhitespace();
	DebugUtils::logInfo("=== Custom Scan Finished ===");
	std::this_thread::sleep_for(3000ms);


	// Save camera configurations in a json file
	if (config.getValue<bool>("dslr.collect_dslr")) {
		saveCameraConfig(scan_folder + PATH_SEP + "pose-" + curr_pose);
		saveScanTime(duration, scan_folder + PATH_SEP + "pose-" + curr_pose);
		if (config.getValue<bool>("transform_generator.enabled"))
			generateTransform(degree_inc, num_moves);
	}

	// Generate transform
	// generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;
	write_pose_to_moadfig(curr_pose); // write pose to moadfig

	MenuHandler::WaitUntilKeypress();

	DebugUtils::logDebug("======Custom scan for pose " + std::to_string(curr_pose) + " completed.======");
	return true;
}



/*
	COMMENTS FOR scanFromSaveState():
		args: none
		returns: bool (as per other scan functions)

	Runs a scan based on previous state recorded in moad_config.json
	This should only be run if a camera fails like err 30 (see devlog)

	- the turntable should have stopped turning after camera fail and the rotational
			position saved in config
	- apparently degree_tracker is a global variable but for this function
			we will use the recorded item in config
	- we will also set the degree_tracker to the current position and ideally
			this will rename the images correctly and start creating them from
			the current rotation

	OR TODO IMPORTANT:
	- change away from global variable to use the config and only reset for fullscan or custom
			which should always start at 0
	
	NOTE: we cannot really account for the situation where:
	1. camera fails and then
	2. turntable moves OR object moves

	since the rotation is based off each previous movement

	bugs:
	- possible index our of range error occurs at the end BUT as of 7/24 when i wrote this one of the cameras
			does not work so i havent managed to hit the end of the image collection (same possible bug AS fullScan() 
			and customScan())

	- GS 7/25/2025
*/
bool scanFromSaveState() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	DebugUtils::logInfo("========Starting scan from saved state========");
	DebugUtils::logInfo("\n\tcurrent pose: " + std::to_string(curr_pose));

	// Create the thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);

	// get previous state STRAIGHT FROM THE CONFIG FILE (instead of global variable stuff)
	int prev_inc = 0;
	// Read previous turntable position from config
	std::ifstream config_file(json_path);
	nlohmann::json config_json;
	if (config_file.is_open()) {
		config_file >> config_json;
		config_file.close();
		if (config_json.contains("prev_state") && config_json["prev_state"].contains("turntable_pos")) {
			prev_inc = config_json["prev_state"]["turntable_pos"].get<int>();
			degree_tracker = prev_inc;
			std::cout << "Loaded previous turntable position: " << prev_inc << std::endl;
		} else {
			std::cout << "No previous turntable position found in config." << std::endl;
		}
	} else {
		std::cerr << "Failed to open config file: " << json_path << std::endl;
	}

	// get the previous number of moves from config
	int num_moves_left = 0;
	if (config_json.contains("prev_state") && config_json["prev_state"].contains("current_move")) {
		num_moves_left = config_json["prev_state"]["current_move"].get<int>();
		std::cout << "Loaded previous number of moves: " << num_moves_left << std::endl;
	} else {
		std::cout << "No previous number of moves found in config." << std::endl;
	}

	// get total number of moves from config
	int num_moves = config.getValue<int>("num_moves");
	// get the degree increment and number of moves from config
	int degree_inc = config.getValue<int>("degree_inc");

	// validity checks
	if (degree_inc <= 0) {
		std::cerr << "Invalid degree increment: " << degree_inc << ". It must be greater than 0." << std::endl;
		return false;
	}
	if (num_moves_left > num_moves) {	
		std::cout << "WARNING: Previous number of moves exceeds total number of moves. Resetting to 0." << std::endl;
		num_moves_left = 0; // Reset to 0 if it exceeds total moves
	}
	if (num_moves <= 0) {
		std::cerr << "Invalid number of moves: " << num_moves << ". It must be greater than 0." << std::endl;
		return false;
	}


	// debug messages
	// std::cout << "Starting scan from saved state..." << std::endl;
	// std::cout << "Previous Turntable Position: " << prev_inc << std::endl;
	// std::cout << "Current Degree Tracker: " << degree_tracker << std::endl;
	// std::cout << "Number of Moves Left: " << num_moves_left << std::endl;
	// std::cout << "Total Number of Moves: " << num_moves << std::endl;


	DebugUtils::logDebug("Starting scan from saved state...");
	DebugUtils::logDebug("Current Degree Tracker: " + std::to_string(degree_tracker));
	DebugUtils::logDebug("Previous Turntable Position: " + std::to_string(prev_inc));
	DebugUtils::logDebug("Number of Moves Left: " + std::to_string(num_moves_left));
	DebugUtils::logDebug("Total Number of Moves: " + std::to_string(num_moves));


	// Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = num_moves_left; rots < num_moves; rots++)
	{
		if(scan(&pool) == false) {
			std::cerr << "\n============================================================" << std::endl;
			std::cerr << "!!! ERROR: scan() failed at move " << rots + 1 << "/" << num_moves << ". Aborting scan. !!!" << std::endl;
			std::cerr << "=======================================================\\pose=====\n" << std::endl;

			DebugUtils::logError("Failed scan at move " + std::to_string(rots + 1) + " of " + std::to_string(num_moves));

			// add delay
			std::this_thread::sleep_for(3000ms);
			break;
		}

		// Rotate the turntable
		rotate_turntable(degree_inc);

		// Update the degree tracker
		degree_tracker += degree_inc;
		// std::cout << "\n==========write to moadfig here===========" << std::endl; // debug msgs delete these
		write_degree_move_to_moadfig(degree_tracker, rots + 1); // NOTE: double check 0 indexing for this +1
		// std::cout << "==========write to moadfig here===========\n" << std::endl;
		
		std::cout << "Image " << rots+1 << "/" << num_moves << " taken. " << std::endl;
		DebugUtils::logDebug("Image " + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + " taken.");
	}


	// Stop the loop timer
	auto end = std::chrono::high_resolution_clock::now();
	// Calculate the elapsed time
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// Convert the duration to minutes, seconds, and milliseconds
	int minutes = duration.count() / 60000;
	int seconds = (duration.count() % 60000) / 1000;
	std::cout << "Scan Time: " << std::setfill('0') << std::setw(2) << minutes << ":" 
		<< std::setfill('0') << std::setw(2) << seconds << std::endl;
	std::cout << "RS Fail Count: " << rshandle.fail_count << std::endl;
	std::this_thread::sleep_for(3000ms);

	DebugUtils::logDebug("Scan Time: " + std::to_string(duration.count()) + " ms");
	DebugUtils::logDebug("RS Fail Count: " + std::to_string(rshandle.fail_count));

	// Save camera configurations in a json file
	if (config.getValue<bool>("dslr.collect_dslr")) {
		saveCameraConfig(scan_folder + PATH_SEP + "pose-" + curr_pose);
		saveScanTime(duration, scan_folder + PATH_SEP + "pose-" + curr_pose);
		if (config.getValue<bool>("transform_generator.enabled"))
			generateTransform(degree_inc, num_moves);
	}

	// Generate transform
	// generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;
	write_pose_to_moadfig(curr_pose); // write pose to moadfig

	MenuHandler::WaitUntilKeypress();

	DebugUtils::logInfo("========Scan from saved state completed========");
	return true;
}



// single scan
bool collectSampleData() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	DebugUtils::logInfo("========Starting single scan========");

	// Create thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);

	// Scan all the cameras
	scan(&pool);

	// I think this only makes sense to call for full scans...
	// run_filecount_check();

	DebugUtils::logInfo("========Single scan completed========");
	return false;
}





/*
Sets the name for the object being scanned / data being collected.
	- creates the necessary output folders
	- sets the global "scan_folder"
	- creates and object_info.json template file
	- determines the last pose
	- updates the moad config file with the new object name / pose
*/
void setObjectName(std::string object_name) {
	ConfigHandler& config = ConfigHandler::getInstance();

	DebugUtils::logInfo("Setting object name to: " + object_name);

	// this does something weird where templated setValue calls an inner "saveConfig" function that was
	//	previously hardcoded and saved the config in the wrong location. should be fixed with fs::canonical
	//	(see comments in main)

	config.setValue<std::string>("object_name", object_name, json_path);

	// Get the scan folder path
	std::string output_dir = config.getValue<std::string>("output_dir");
	scan_folder = output_dir + PATH_SEP + object_name;

	// Create base Object data Folder
	create_folder(scan_folder);

	// Create object info.json (template)
	object_info["Object Name"] = object_name;
	create_obj_info_json(config.getValue<std::string>("output_dir"));
	DebugUtils::logWhitespace();

	// save the object name to the moadfig file
	DebugUtils::logInfo("Updating moad_config.json file...");
	write_obj_to_moadfig(object_name); // write object name to moadfig

	// Change pose
	// POSSIBLE EXTRA POSE CHANGE? - GS 8/7
	curr_pose = get_last_pose();
	object_info["Pose"] = curr_pose;
	write_pose_to_moadfig(curr_pose); // write pose to moadfig







	// NOTE: 10/29 GS
	// - this redundantly calls create_folder for realsense and DSLR when they
	// 	have been previously called in the realsense and DSLR intialize function


	// std::cout << "scan_folder inside setObjectName doesnt exist?: " << scan_folder << std::endl;

	// Update the save directories for RealSense and DSLR
	rshandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "realsense";
	// create_folder(rshandle.save_dir,true);
	canonhandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "DSLR";
	// create_folder(canonhandle.save_dir,true);

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
	char last_pose = get_last_pose();
	std::cout << "Enter Pose (Last pose: '" << last_pose << "'): ";
	std::cin >> curr_pose;
	object_info["Pose"] = curr_pose;
	write_pose_to_moadfig(curr_pose); // write pose to moadfig


	return false;
}

/*
	CAMERA BUTTON CONTROL
*/
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

/*
	CAMERA SETTINGS MENUS
*/
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
		
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
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
		

		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
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
		
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
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
		
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
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
		
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
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

bool changeCamera() {
	int option = -1;
	std::cout << "Change Camera. Select which camera to use (0 = all camera):" << std::endl;
	int i = 1;
	
	// Display all available cameras 
	std::cout << "0. All cameras" << std::endl;
	for (const auto& camera : canonhandle.cameraArray) {
		std::cout << i << ". " << camera_name[camera] << std::endl;
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

/*
	LIVE VIEW MENUS
*/
bool getLiveView() {
	ConfigHandler& config = ConfigHandler::getInstance();

	// setup directory for filestream output (otherwise it creates wherever the
	//	MultiCamCui exec is run out of)
	std::string liveViewDir = moad_dir + PATH_SEP + "live_view_filestream" + PATH_SEP;


	if (!config.getValue<bool>("dslr.collect_dslr")){
		std::cout << "DSLR option is set to false, Liveview unavaliable" << std::endl;
		return false;
	}

	if (!liveview_active) {
		// Start Live View configuration
		DebugUtils::logDebug("Starting Live View...");
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
				DownloadEvfCommand(camera, liveViewDir + camera_name[camera], liveview_thread_id);
			}));
			std::this_thread::sleep_for(.5s);
			i++;
		}
		DebugUtils::logDebug("Live View Threads Created...");
	}
	else {
		std::cout << "The liveview is active" << std::endl;
	}

	return true;
}

bool endLiveView() {
	ConfigHandler& config = ConfigHandler::getInstance();
	// Check if the DSLR option is enabled
	if (!config.getValue<bool>("dslr.collect_dslr")) {
		std::cout << "DSLR option is set to false, Liveview unavaliable" << std::endl;
		return false;
	}
	DebugUtils::logDebug("Ending Live View...");
	std::cout << "Ending Liveview..." << std::endl;
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
	std::cout << "Liveview sucessfully closed" << std::endl;
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

bool turntableControl() {
	std::cout << "\n\n\nTURNTABLE CONTROL\n\n";
	std::string degree_inc;
	while(degree_inc != "r"){
		// Prompt for degrees to move
		std::cout << "Enter degrees to move (r = Return): ";
		std::cin >> degree_inc;

		// Check if the input is "r" to return
		if (degree_inc == "r")
			break;
		
		// Send the degree increment to the turntable
		char *send = &degree_inc[0];
		bool is_sent = Serial->WriteSerialPort(send);
		
		// TODO: Calculate wait time based on degrees entered and motor speed 
		if (is_sent) {
			// Calculate wait time based on the degree increment
			int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
			std::cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
			
			// Check if the message matched the expected format
			std::string incoming = Serial->ReadSerialPort(wait_time, "json");
			std::cout << "Incoming: " << incoming << std::endl;
		} 

		// Update degree tracker
		degree_tracker += std::stoi(degree_inc);
		degree_tracker = degree_tracker % 360;
	}

	// Update the object info with the new turntable position
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	return false;
}

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


/*
	InitializeRealsense()
		args: none
		returns: void

	TODO: this adds a lot of possibly unneeded info to the console upon startup, clear this up
		or add a verbose flag in the config for info toggling.
*/
void initializeRealsense() {
	ConfigHandler& config = ConfigHandler::getInstance();

	if (config.getValue<bool>("realsense.collect_realsense")) {

		// scan_folder is not set when this is first intialized TODO
		rshandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "realsense";
		// create_folder(rshandle.save_dir,true);

		// Get some frames to settle autoexposure.
		// std::cout << "Getting frames..." << std::endl;

	
		rshandle.initialize(json_path);

		// Get some frames to settle autoexposure and verify stream.
		int test_frames = 10;
		DebugUtils::logRS("Getting " + std::to_string(test_frames) + " frames to verify data...");
		rshandle.get_frames(test_frames); // make 30 later
		// rshandle.get_current_frame();

	} else {
		std::cout << "\nSkipping RealSense setup, 'collect_rs=0'.\n";
	}
}


/*
	InitializeCanon()
		args: none
		returns: void
	
	TODO: This should be edited to be more prominent on startup so the user is aware of
			how many cameras are connected.
	COMPLETED: added yellow text on startup that displays number of connected cameras - GS 10/3
	
*/
void initializeCanon() {
	ConfigHandler& config = ConfigHandler::getInstance();

	// get camera ids from config (previously they were hardcoded) - gs 11/7
	std::string cam1 = config.getValue<std::string>("dslr.camera_ids.CAMERA_1");
	std::string cam2 = config.getValue<std::string>("dslr.camera_ids.CAMERA_2");
	std::string cam3 = config.getValue<std::string>("dslr.camera_ids.CAMERA_3");
	std::string cam4 = config.getValue<std::string>("dslr.camera_ids.CAMERA_4");
	std::string cam5 = config.getValue<std::string>("dslr.camera_ids.CAMERA_5");

	
	if (config.getValue<bool>("dslr.collect_dslr")) {

		// CanonHandler canonhandle;
		canonhandle.initialize();

		// scan_folder is not set when this is first intialized TODO
		canonhandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "DSLR";
		// create_folder(canonhandle.save_dir,true);
		PreSetting(canonhandle.cameraArray, canonhandle.bodyID);
		// Naming of the camera
		DebugUtils::logCanon("Remapping Camera Names...");
		EdsChar serial[13];
		EdsError err;
		std::stringstream cam_message;
		int index = 1;
		for (const auto& camera: canonhandle.cameraArray) {
			// Fetch the serial number
			err = EdsGetPropertyData(camera, kEdsPropID_BodyIDEx, 0, sizeof(serial), &serial);
			if (err != 0)  {
				std::cout << "canon initialization err: " << err << std::endl;
			}

			// Convert it into string
			std::string serial_str = "";
			for (size_t i = 0; i < sizeof(serial) - 1; i++){
				serial_str += serial[i];
			}

			cam_message.str("");
			cam_message.clear();
			cam_message << "Camera " << index << " (Serial Number: " << serial_str <<"):";


			if (serial_str == cam1) {
				cam_message << "\n\t\t|- Renamed to Camera 1";
				canonhandle.camera_names[std::to_string(index)] = "1";
				camera_name[camera] = "Camera 1";
			}
			else if (serial_str == cam2) {
				cam_message << "\n\t\t|- Renamed to Camera 2";
				canonhandle.camera_names[std::to_string(index)] = "2";
				camera_name[camera] = "Camera 2";
			}
			else if (serial_str == cam3) {
				cam_message << "\n\t\t|- Renamed to Camera 3";
				canonhandle.camera_names[std::to_string(index)] = "3";
				camera_name[camera] = "Camera 3";
			}
			else if (serial_str == cam4) {
				cam_message << "\n\t\t|- Renamed to Camera 4";
				canonhandle.camera_names[std::to_string(index)] = "4";
				camera_name[camera] = "Camera 4";
			}
			else if (serial_str == cam5) {
				cam_message << "\n\t\t|- Renamed to Camera 5";
				canonhandle.camera_names[std::to_string(index)] = "5";
				camera_name[camera] = "Camera 5";
			}else {
				// If serial number is not matched, set the camera name to it's serial number.
				cam_message << "\n\t\t|- NO MATCHING SERIAL NUMBER FOUND IN CONFIG (dslr/camera_ids), setting name to serial...";
				canonhandle.camera_names[std::to_string(index)] = serial_str;
				camera_name[camera] = "Camera " + serial_str;
			}
			
			DebugUtils::logCanon(cam_message.str());
			index++;
		}
	} else {
		std::cout << "\nSkipping DSLR setup, 'collect_dslr=0'." << "\033[0m" << std::endl;
	}

}



/*
	COMMENTS for run_filecount_check():
		args: none
		returns: void
	(For now no parameters used, hardcoded args for a python script)

	THIS IS THE CONNECTION POINT TO THE PYTHON SCRIPT

	Runs /scripts/filecount_test.py via CLI args with preset args
	Note: args available are --count, --create, --prog-delay, --delay=<seconds> (see comments in filecount_test.py)
	TODO: add function parameters that setup CLI args the script is called with (personally i dont see an immediate need for this)
	- GS 7/15
*/
bool run_filecount_check() {
	
	// Sleep(200); // wait a bit for realsense to save, otherwise jumbled console output

	ConfigHandler& config = ConfigHandler::getInstance();
	if (config.getValue<bool>("filecount_testing.enabled") == false) {
		std::cout << "Filecount testing disabled in config, skipping..." << std::endl;
		return false;
	}
	
	DebugUtils::logWhitespace();
	DebugUtils::logInfo("Running file count checking script...");
	// std::cout << "Running file count checking script: " << std::endl;

	// ex. python3.12.exe .\filecount_test.py --count --create --manual_check

	// Prepare command to execute scripts/filecount_test.py
	std::stringstream command_stream;
	command_stream 
		<< "python3 "
		<< "scripts/filecount_test.py ";
		// << "--count "

	if (config.getValue<bool>("filecount_testing.count") == true) {
		command_stream << "--count ";
	}

	if (config.getValue<bool>("filecount_testing.create") == true) {
		command_stream << "--create ";
	}

	if (config.getValue<bool>("filecount_testing.manual_check") == true) {
		command_stream << "--manual_check ";
	}

	if (config.getValue<bool>("filecount_testing.delay") == true) {
		command_stream << "--prog_delay " << "--delay=0.1 "; // hardcoded delay for now
	}

	// add current object directory:
	command_stream << "--directory=\"" << scan_folder << "\" ";

	if (config.getValue<bool>("filecount_testing.check_single_object") == true) {
		command_stream << "--check_single_object ";
	}

	// Execute command
	// convert to string for output debug message
	std::string command = command_stream.str(); 
	std::cout << "\nExecuting Command: " << command; 
	
	// system requires c string
	const char* c_command = command.c_str();
	system(c_command);

	return true;
}


int main(int argc, char* argv[]) 
{	
	// std::cout << "\033[1;44m" << "[ START OF MAIN ]" << "\033[0m\n";	
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
	
	// This function also handles setting the initial object name, 
	// creating the object folder, and getting the last pose.
	// Also notifies DebugUtils that a config has been loaded.
	// DebugUtils::logDebug("Loading Config...");
	loadJsonConfig(json_path);
	// DebugUtils::notifyConfigReady(); // Flag to DebugUtils that a config has been loaded

	// Provides global config access
	ConfigHandler& config = ConfigHandler::getInstance();
	
	// Initialize Debug Log File
	// NOTE: 'scan_folder' is initialized when the config is loaded, which calls setObjectName()
	// NOTE/TODO: Currently, DebugUtils cannot be used until config is initialized.
	std::string debug_log_dir = scan_folder + PATH_SEP + "debug_log.txt";
	DebugUtils::initLogFile(debug_log_dir);

	// Setup Arduino serial port connection
	std::string com_port;
	com_port = config.getValue<std::string>("serial_com_port");

	typedef unsigned long DWORD;
	DWORD COM_BAUD_RATE = B9600; // Serial baud rate currently hardcoded

	DebugUtils::logInfo("Attempting Serial Port Connection...");
	Serial = new SimpleSerial(com_port.c_str(), COM_BAUD_RATE); // input must be c string

	// Log some additional information at startup
	DebugUtils::logDebug("Current Object Name: " + object_info["Object Name"]);
	DebugUtils::logDebug("Current Pose: " + object_info["Pose"]);
	DebugUtils::logDebug("Current Turntable Position: " + object_info["Turntable Pos"]);
	DebugUtils::logDebug("Current Scan Folder: " + scan_folder);
	DebugUtils::logDebug("Current JSON Path: " + json_path);

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
		{"f", run_filecount_check}
	}, object_info);
	menu_handler.setTitle("MOAD - CLI Menu");
	menu_handler.ClearScreen();
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