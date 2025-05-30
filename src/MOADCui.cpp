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

#include <windows.h>
#include "tabulate.hpp"
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "Download.h"
#include "DownloadEvf.h"
#include "PreSetting.h"
#include "PressShutter.h"
#include "Property.h"
#include "TakePicture.h"
#include "SimpleSerial.h"
#include "CameraException.h"

#define CAMERA_1 "352074022019"
#define CAMERA_2 "352074022024"
#define CAMERA_3 "352074022025"
#define CAMERA_4 "352074022005"
#define CAMERA_5 "352074022021"

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using std::cout;
using std::endl;

std::string control_number = "";
char curr_pose = 'a';
bool keyflag;
int degree_tracker = 0;

std::string scan_folder;

// Liveview Threads
std::vector<std::thread> liveview_th;
bool liveview_active = false;
std::thread::id liveview_thread_id;

// Camera
std::map<EdsCameraRef, std::string> camera_name;
bool all_cameras = true;
EdsCameraRef activeCamera;

RealSenseHandler rshandle;
SimpleSerial* Serial;

// Menu
MenuHandler* curr_menu;

// TXT Config 
std::map<std::string, std::string> object_info;
std::string json_path;

// Initialize some functions
void setObjectName(std::string object_name);
void initializeCanon();
void initializeRealsense();

int get_rs_timeout() {
	ConfigHandler& config = ConfigHandler::getInstance();
	int rs_timeout = config.getValue<int>("realsense.realsense_timeout_sec") * 1000;
	return rs_timeout;
}

int get_dslr_timeout() {
	ConfigHandler& config = ConfigHandler::getInstance();
	int dslr_timeout = config.getValue<int>("dslr.dslr_timeout_sec") * 20;
	return dslr_timeout;
}

void loadJsonConfig(std::string path) {
	ConfigHandler& config = ConfigHandler::getInstance();
	std::optional<bool> previous_DSLR;
	std::optional<bool> previous_RS;

	// Check if config is initialized
	if (!config.emptyConfig()) {
		// Get the previous values of DSLR and RealSense collection settings
		previous_DSLR = config.getValue<bool>("dslr.collect_dslr");
		previous_RS = config.getValue<bool>("realsense.collect_realsense");
	}

	// Load the configuration from the specified path
	config.loadConfig(path);

	// Check if the configuration has changed for DSLR
	if (!previous_DSLR.has_value() || previous_DSLR != config.getValue<bool>("dslr.collect_dslr")) {
		std::cout << "Initializing CanonHandler" << std::endl;
		if (config.getValue<bool>("dslr.collect_dslr") && !canonhandle.isSDKLoaded) {
			initializeCanon();
		}
	}
	
	// Check if the configuration has changed for RealSense
	if (!previous_RS.has_value() || previous_RS != config.getValue<bool>("realsense.collect_realsense")) {
		std::cout << "Initializing RealSenseHandler" << std::endl;
		if (config.getValue<bool>("realsense.collect_realsense")) {
			initializeRealsense();
		}
	}

	// Set the object name from the configuration
	std::string object_name = config.getValue<std::string>("object_name");
	setObjectName(object_name);
}

void saveCameraConfig(std::string path) {
	std::vector<std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>>> propertyIDs = {
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_ISOSpeed, iso_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Tv, tv_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Av, av_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_WhiteBalance, whitebalance_table),
	};

	ConfigHandler& config = ConfigHandler::getInstance();
	nlohmann::json json_data;

	// Get the model and focal length for each camera
	for (auto& camera : canonhandle.cameraArray) {
		std::string cam = camera_name[camera];
		EdsDeviceInfo deviceInfo;
		EdsGetDeviceInfo(camera, &deviceInfo);
		json_data[cam]["Model"] = deviceInfo.szDeviceDescription; 
		json_data[cam]["Focal Length"] = config.getValue<std::string>("transform_generator.calibration_mode");
	}

	// Get the configuration properties for each camera
	for (auto propertyID : propertyIDs) {
		std::string name = std::get<0>(getPropertyString(std::get<0>(propertyID)));
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, std::get<0>(propertyID), std::get<1>(propertyID), out_table);
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
        std::cout << "Camera configuration saved to " << output_file << std::endl;
    } else {
        std::cerr << "Failed to open file for writing: " << output_file << std::endl;
	}
}

void saveScanTime(std::chrono::milliseconds duration, std::string path) {
	// Get the given duration in minutes and seconds
	std::string minutes = std::to_string(duration.count() / 60000);
	std::string seconds = std::to_string((duration.count() % 60000) / 1000);

	// Open the JSON file for writing
	std::ifstream file(path + "/camera_config.json");
	if (!file.is_open()) {
		std::cerr << "Failed to open file for writing: " << path + "/camera_config.json" << std::endl;
		return;
	}
	nlohmann::json json_data;
	file >> json_data;

	// Add the scan time to the JSON data
	json_data["Scan Time"] = minutes + ":" + seconds;

	// Save the updated JSON data back to the file
	std::ofstream output_file(path + "/camera_config.json");
	if (output_file.is_open()) {
		output_file << json_data.dump(4); // Pretty print with 4 spaces indentation
		output_file.close();
		std::cout << "Scan time saved to camera_config.json" << std::endl;
	} else {
		std::cerr << "Failed to open file for writing: " << path + "/camera_config.json" << std::endl;
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

int create_folder(std::string path, bool quiet = false) {
	if (!quiet) cout << endl;
	if (!fs::exists(path)) {
        // Create the folder and any necessary higher level folders
        if (fs::create_directories(path)) {
            cout << "Folder created: " << path << endl;
        } else {
            std::cerr << "Failed to create folder: " << path << endl;
            return 1;
        }
    } else {
		if (!quiet)
			cout << "WARNING!: Folder already exists: " << path << endl
				<< "\tYou may accidentally overwrite data.\n";
    }
	return 0;
}

void create_obj_info_json(std::string path) {
	std::string object_name = object_info["Object Name"];
	std::cout << "Creating object info JSON file: " << object_name << std::endl;

	// Check if the path exists
	if (fs::exists(path)){
		// Generate the command to execute scripts\create_object_info.py
		std::stringstream command_stream;
		command_stream 
			<< "python3 " 
			<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/create_object_info.py "
			<< object_name << " "
			<< "-p " << path << " ";

		// Execute command
		std::string command = command_stream.str(); 
		const char* c_command = command.c_str();
		std::cout << "\nExecuting Command: " << command; 
		system(c_command);
	}
	else {
		std::cout << "WARNING: folder " << path << " does not exist." << std::endl;
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
	std::string path = output_dir + "/" + object_name;
	char last_pose = 'a'; // First Pose
	// Check if the directory exists
	if (!fs::exists(path) || !fs::is_directory(path) || fs::is_empty(path)) {
		return 'a';
	}
	
	// Check for files that start with 'pose-'
	for (const auto& entry : fs::directory_iterator(path)) {
		if (fs::is_directory(entry)) {
			std::string name = entry.path().filename().string();
			if (name.find("pose-") != std::string::npos) {
				int length = name.length();
				char letter = name[length - 1];
				// Check if the folder is empty
				if (fs::is_empty(path + "/pose-" + letter + "/")) {
					return letter; // Current pose
				}
				else {
					// If is not empty, check if the subfolder is empty
					for (const auto& sub_entry : fs::directory_iterator(path + "/pose-" + letter)) {
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
	return last_pose + 1; 
}

void scan(ThreadPool* pool = nullptr) {
	ConfigHandler& config = ConfigHandler::getInstance();
	std::string object_name = config.getValue<std::string>("object_name");
	std::string output_dir = config.getValue<std::string>("output_dir");
	scan_folder = output_dir + "/" + object_name;
	
	// Collect RealSense Data
	if(config.getValue<bool>("realsense.collect_realsense")) {
		// Create RS Scan Folder
		rshandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\realsense";
		create_folder(rshandle.save_dir, true);

		// Get the current frame from RealSense
		int rs_timeout = get_rs_timeout();
		rshandle.turntable_position = degree_tracker;
		rshandle.get_current_frame(degree_tracker, rs_timeout, pool);
		if (rshandle.fail_count > 0) {
			cout << "RS Failure - " << rshandle.fail_count << endl;
		}
	}

	// Collect DSLR Data
	if(config.getValue<bool>("dslr.collect_dslr")) {
		canonhandle.images_downloaded = 0;
		
		// Create DSLR Scan Folder
		canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		
		// Take pictures with DSLR
		cout << "Getting DSLR Data...\n";
		canonhandle.turntable_position = degree_tracker;
		for (auto& camera : canonhandle.cameraArray) {
			std::string cam_name = camera_name[camera];
			EdsError err = EDS_ERR_OK;
			err = TakePicture(camera, cam_name);
		}

		// Collecting images from DSLR
		int c = 0;
		int dslr_timeout = get_dslr_timeout();
		while (canonhandle.images_downloaded < canonhandle.cameras_found && c < dslr_timeout) {
			EdsGetEvent();
			std::this_thread::sleep_for(50ms);
			c++;
		}
		cout << "DSLR Timeout Count: " << c << "/" << dslr_timeout << endl;
	}
}

void rotate_turntable(int degree_inc) {
	// Issue command to move turntable.
	ConfigHandler& config = ConfigHandler::getInstance();
	std::string degree_inc_str = std::to_string(degree_inc);
	char *send = &degree_inc_str[0];
	bool is_sent = Serial->WriteSerialPort(send);

	if (is_sent) {
		int wait_time = std::ceil(((abs(degree_inc)*200)+500)/1000)+5;
		cout << "Message sent, moving: " << degree_inc << " degrees, waiting up to " << wait_time << " seconds.\n";
		std::string incoming = Serial->ReadSerialPort(wait_time, "json");
		cout << "Incoming: " << incoming;// << endl;
		// std::this_thread::sleep_for(250ms);
		
		int turntable_delay_ms = config.getValue<int>("turntable_delay_ms");
		std::this_thread::sleep_for(std::chrono::milliseconds(turntable_delay_ms));
	} else {
		cout << "WARNING: Serial command not sent, something went wrong.\n";
	}
}

bool generateTransform(int degree_inc, int num_moves) {
	ConfigHandler& config = ConfigHandler::getInstance();

	// Collect parameters from config
	bool force = config.getValue<bool>("transform_generator.force");
	bool visualize = config.getValue<bool>("transform_generator.visualize");
	std::string calibration_dir = config.getValue<std::string>("transform_generator.calibration_dir_windows");
	std::string calibration = config.getValue<std::string>("transform_generator.calibration_mode");
	std::string output_dir = config.getValue<std::string>("transform_generator.dir_windows");
	std::string object_name = config.getValue<std::string>("object_name");

	// Prepare command to execute scripts/transform_generator.py
	int range = degree_inc * num_moves;
	std::stringstream command_stream;
	command_stream 
		<< "python3 " 
		<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/transform_generator.py "
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
	std::cout << "\nExecuting Command: " << command; 
	system(c_command);

	return false;
}

bool customScan() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}
	// Create the thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);
	int degree_inc;
	int num_moves = 0;

	// Ask for the user to input the degree increment and number of moves
	cout << "Enter degrees per move: ";
	std::cin >> degree_inc;
	cout << "Enter number of moves: ";
	std::cin >> num_moves;
	
	Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		// Scan all the cameras
		scan(&pool);
		// Rotate the turntable
		rotate_turntable(degree_inc);
		
		// Update the degree tracker 
		degree_tracker += degree_inc;
		cout << "Image " << rots+1 << "/" << num_moves << " taken. " << endl;
	}
	// Stop the loop timer
	auto end = std::chrono::high_resolution_clock::now();
	// Calculate the elapsed time
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// Convert the duration to minutes, seconds, and milliseconds
	int minutes = duration.count() / 60000;
	int seconds = (duration.count() % 60000) / 1000;
	cout << "Scan Time: " << std::setfill('0') << std::setw(2) << minutes << ":" 
		<< std::setfill('0') << std::setw(2) << seconds << endl;
	cout << "RS Fail Count: " << rshandle.fail_count << endl;
	std::this_thread::sleep_for(3000ms);

	// Save camera configurations in a json file
	if (config.getValue<bool>("dslr.collect_dslr")) {
		saveCameraConfig(scan_folder + "\\pose-" + curr_pose);
		saveScanTime(duration, scan_folder + "\\pose-" + curr_pose);
	}

	// Generate transform
	generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;

	MenuHandler::WaitUntilKeypress();

	return true;
}

bool fullScan() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	// Create thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);

	int degree_inc = config.getValue<int>("degree_inc");
	int num_moves = config.getValue<int>("num_moves");
	
	Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		// Scan all the cameras
		scan(&pool);
		// Rotate the turntable
		rotate_turntable(degree_inc);
		
		// Update the degree tracker
		degree_tracker += degree_inc;
		cout << "Image " << rots+1 << "/" << num_moves << " taken. " << endl;
	}
	// Stop the loop timer
	auto end = std::chrono::high_resolution_clock::now();
	// Calculate the elapsed time
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// Convert the duration to minutes, seconds, and milliseconds
	int minutes = duration.count() / 60000;
	int seconds = (duration.count() % 60000) / 1000;
	cout << "Scan Time: " << std::setfill('0') << std::setw(2) << minutes << ":" 
	<< std::setfill('0') << std::setw(2) << seconds << endl;
	cout << "RS Fail Count: " << rshandle.fail_count << endl;
	std::this_thread::sleep_for(3000ms);
	
	// Save camera configurations in a json file
	saveCameraConfig(scan_folder + "\\pose-" + curr_pose);
	saveScanTime(duration, scan_folder + "\\pose-" + curr_pose);

	// Generate transform
	generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;

	MenuHandler::WaitUntilKeypress();

	return true;
}

// TODO: Check Degree Tracker
bool collectSampleData() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	// Create thread pool
	int thread_num = config.getValue<int>("thread_num");
	ThreadPool pool(thread_num);

	// Scan all the cameras
	scan(&pool);

	return false;
}

void setObjectName(std::string object_name) {
	ConfigHandler& config = ConfigHandler::getInstance();
	
	// Set the object name in the config 
	config.setValue<std::string>("object_name", object_name);

	// Get the scan folder path
	std::string output_dir = config.getValue<std::string>("output_dir");
	scan_folder = output_dir + "/" + object_name;

	// Create Main Folder
	create_folder(scan_folder);

	// Create object info.json (template)
	create_obj_info_json(config.getValue<std::string>("output_dir"));
	config.setValue<std::string>("object_name", object_name);
	object_info["Object Name"] = object_name;

	// Change pose
	curr_pose = get_last_pose();
	object_info["Pose"] = curr_pose;

	// Update the save directories for RealSense and DSLR
	rshandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\realsense";
	create_folder(rshandle.save_dir,true);
	canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
	create_folder(canonhandle.save_dir,true);
}

bool setObjectName() {
	ConfigHandler& config = ConfigHandler::getInstance();
	
	// Prompt for object name 
	std::string object_name_input;
	cout << "\n\nEnter Object Name: ";
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
	return false;
}

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
		cout << "input no. (ex. 54 = 1/250)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {	
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
		cout << "input no. (ex. 54 = 1/250)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {
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
		cout << "input Av (ex. 21 = 5.6)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {
			// Set the new value for all cameras
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
		cout << "input Av (ex. 21 = 5.6)" << endl;
		cout << ">";

		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {
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
		cout << "input ISOSpeed > ";

		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {
			// Set the new value for all cameras
			err = SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, out_table);
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
		cout << "input ISOSpeed (ex. 8 = ISO 200)" << endl;
		cout << ">";

		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1) {
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
		cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)	{
			// Set the new value for all cameras
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
		cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)	{
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
		cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)	{
			// Set the new value for all cameras
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
		cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
		cout << ">";
		
		// Get user input for the new value
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)	{
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

bool getLiveView() {
	ConfigHandler& config = ConfigHandler::getInstance();
	if (!config.getValue<bool>("dslr.collect_dslr")){
		std::cout << "DSLR option is set to false, Liveview unavaliable" << std::endl;
		return false;
	}

	if (!liveview_active) {
		// Start Live View configuration
		liveview_active = true;
		liveview_thread_id = std::this_thread::get_id();

		// Change camera configuration to liveview
		StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
		std::this_thread::sleep_for(.5s);
		
		// Download and display Image from camera 
		int i = 0;
		for (auto& camera : canonhandle.cameraArray) {
			// Download the image from the camera
			liveview_th.push_back(std::thread([&]() {
				DownloadEvfCommand(camera, camera_name[camera], liveview_thread_id);
			}));
			std::this_thread::sleep_for(.5s);
			i++;
		}
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

	std::cout << "Ending Liveview..." << std::endl;
	// Join all liveview threads
	liveview_active = false;
	for (auto& th : liveview_th) {
		if (th.joinable()) {
			th.join();
		}
	}

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
	cout << "\n\n\nTURNTABLE CONTROL\n\n";
	std::string degree_inc;
	while(degree_inc != "r"){
		// Prompt for degrees to move
		cout << "Enter degrees to move (r = Return): ";
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
			cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
			
			// Check if the message matched the expected format
			std::string incoming = Serial->ReadSerialPort(wait_time, "json");
			cout << "Incoming: " << incoming;
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
	cout << "\n\nEnter Turntable Position: ";
	std::cin >> degree_tracker;

	// Update the turntable position
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Add a success message to the current menu
	curr_menu->addMessage(MenuMessageStatus::SUCCESS, "Turntable Position updated sucessfully");
	
	return false;
}

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

bool reloadConfig() {
	if (liveview_active) {
		std::cout << "Liveview is active, please stop it before reloading the config." << std::endl;
		return false;
	}

	loadJsonConfig(json_path);
	return false;
}

void initializeRealsense() {
	ConfigHandler& config = ConfigHandler::getInstance();

	if (config.getValue<bool>("realsense.collect_realsense")) {
		cout << "\nAttempting Realsense setup...\n";
		// Create RealSense Handler
		rshandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\realsense";
		create_folder(rshandle.save_dir,true);
		// Get some frames to settle autoexposure.
		std::cout << "Getting frames..." << std::endl;
		rshandle.initialize();
		rshandle.get_frames(10); // make 30 later
		// rshandle.get_current_frame();
	} else {
		cout << "\nSkipping RealSense setup, 'collect_rs=0'.\n";
	}
}

void initializeCanon() {
	ConfigHandler& config = ConfigHandler::getInstance();
	
	if (config.getValue<bool>("dslr.collect_dslr")) {
		cout << "\nEntering DSLR Setup...\n";
		// CanonHandler canonhandle;
		canonhandle.initialize();
		canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		PreSetting(canonhandle.cameraArray, canonhandle.bodyID);
		// Naming of the camera
		EdsChar serial[13];
		EdsError err;
		int index = 1;
		for (const auto& camera: canonhandle.cameraArray) {
			// Fetch the serial number
			err = EdsGetPropertyData(camera, kEdsPropID_BodyIDEx, 0, sizeof(serial), &serial);
			// Convert it into string
			std::string serial_str = "";
			for (size_t i = 0; i < sizeof(serial) - 1; i++){
				serial_str += serial[i];
			}
			std::cout <<  "Renaming Camera " << index << " (Serial Number: " << serial_str <<")"; 
			if (serial_str == CAMERA_1) {
				std::cout << " to 1" << std::endl;
				canonhandle.camera_names[std::to_string(index)] = "1";
				camera_name[camera] = "Camera 1";
			}
			if (serial_str == CAMERA_2) {
				std::cout << " to 2" << std::endl;
				canonhandle.camera_names[std::to_string(index)] = "2";
				camera_name[camera] = "Camera 2";
			}
			if (serial_str == CAMERA_3) {
				std::cout << " to 3" << std::endl;
				canonhandle.camera_names[std::to_string(index)] = "3";
				camera_name[camera] = "Camera 3";
			}
			if (serial_str == CAMERA_4) {
				std::cout << " to 4" << std::endl;
				canonhandle.camera_names[std::to_string(index)] = "4";
				camera_name[camera] = "Camera 4";
			}
			if (serial_str == CAMERA_5) {
				std::cout << " to 5" << std::endl;
				canonhandle.camera_names[std::to_string(index)] = "5";
				camera_name[camera] = "Camera 5";
			}

			index++;
		}
	} else {
		cout << "\nSkipping DSLR setup, 'collect_dslr=0'.\n";
	}
}

int main(int argc, char* argv[])
{	
	// SETUP ----------------------------------------------------------------------------------------------
	degree_tracker = 0;
    std::shared_ptr<std::thread> th = std::shared_ptr<std::thread>();

	// Get the path of the root directory
	fs::path executablePath(argv[0]);
	fs::path projectDir = executablePath.parent_path().parent_path().parent_path();

	// Load the JSON Config file
	json_path = projectDir.string() + "\\moad_config.json";
	loadJsonConfig(json_path);
	ConfigHandler& config = ConfigHandler::getInstance();

	// Create object folder 
	scan_folder = config.getValue<std::string>("output_dir") + "/" + config.getValue<std::string>("object_name");
	create_folder(scan_folder);

	// Create object info template
	create_obj_info_json(config.getValue<std::string>("output_dir"));

	// Get last pose
	std::cout << "Checking Last Pose..." << std::endl;
	curr_pose = get_last_pose();
	std::cout << "Last Pose Checked." << std::endl;

	// Setup Arduino serial port connection
	cout << "\nAttempting Serial Motor Control setup...\n";
	char com_port[] = "\\\\.\\COMX";
	com_port[7] = config.getValue<std::string>("serial_com_port").at(0);
	DWORD COM_BAUD_RATE = CBR_9600;

	Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
	if(Serial->connected_) {
		cout << "Serial connected to " << com_port << endl;
	} else {
		cout << "Error creating serial connection.\n";
	}

	// Change pose
	curr_pose = get_last_pose();
	
	object_info["Object Name"] = config.getValue<std::string>("object_name");
	object_info["Turntable Pos"] = std::to_string(degree_tracker);
	object_info["Pose"] = curr_pose;
	
	// Main Menu initialization
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
		{"0", "Reload Config"}
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
		{"0", reloadConfig},
	}, object_info);
	menu_handler.setTitle("MOAD - CLI Menu");
	menu_handler.ClearScreen();
	menu_handler.initialize(curr_menu);

	return false;
}