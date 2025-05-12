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
#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "MenuHandler.h"

#include <windows.h>
#include "tabulate.hpp"
#include "CanonHandler.h"
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "Download.h"
#include "DownloadEvf.h"
#include "PreSetting.h"
#include "PressShutter.h"
#include "Property.h"
#include "TakePicture.h"
#include "SimpleSerial.h"
#include "RealSenseHandler.h"
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
std::string obj_name;
std::map<std::string, std::string> config;
std::string scan_folder;
int rs_timeout;
int dslr_timeout;
int turntable_delay_ms;

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

std::map<std::string, std::string> loadParameters(const std::string& filename) {
    std::map<std::string, std::string> parameters;
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines or lines starting with #
            if (line.empty() || line[0] == '#')
                continue;
            
            // Split the line into key and value
            size_t delimiterPos = line.find('=');
            if (delimiterPos != std::string::npos) {
                std::string key = line.substr(0, delimiterPos);
                std::string value = line.substr(delimiterPos + 1);
                
                // Remove leading/trailing whitespaces from key and value
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                
                parameters[key] = value;
            }
        }
        file.close();
    }
    return parameters;
}

void saveCameraConfig(std::string path) {
	std::vector<std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>>> propertyIDs = {
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_ISOSpeed, iso_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Tv, tv_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_Av, av_table),
		std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>> (kEdsPropID_WhiteBalance, whitebalance_table),
	};

	nlohmann::json json_data;
	for (auto& camera : canonhandle.cameraArray) {
		std::string cam = camera_name[camera];
		EdsDeviceInfo deviceInfo;
		EdsGetDeviceInfo(camera, &deviceInfo);
		json_data[cam]["Model"] = deviceInfo.szDeviceDescription; 
		json_data[cam]["Focal Length"] = config["tg_calibration_mode"];
	}

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
	// Get the current time
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
	if (fs::exists(path)){
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
	std::string path = config["output_dir"] + "/" + obj_name;
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

void scan() {
	// Collect DSLR Data
	if(bool(stoi(config["collect_dslr"]))) {
		canonhandle.images_downloaded = 0;
		// Change Pose
		scan_folder = config["output_dir"]+"/"+obj_name;
		canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		
		int c = 0;
		// int timeout = stoi(config["dslr_timeout_sec"]) * 20; // 20 = 1s
		EdsError err;
		cout << "Getting DSLR Data...\n";
		canonhandle.turntable_position = degree_tracker;
		err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
		// cout << "Result: " << err << endl;
		while (canonhandle.images_downloaded < canonhandle.cameras_found && c < dslr_timeout) {
			EdsGetEvent();
			std::this_thread::sleep_for(50ms);
			c++;
		}
		cout << "DSLR Timeout Count: " << c << "/" << dslr_timeout << endl;
	}

	// Collect RealSense Data
	if(bool(stoi(config["collect_rs"]))) {
		rshandle.turntable_position = degree_tracker;
		rshandle.get_current_frame(rs_timeout);
		if (rshandle.fail_count > 0) {
			cout << "RS Failure - " << rshandle.fail_count << endl;
		}
	}
}

void rotate_turntable(std::string degree_inc) {
	// Issue command to move turntable.
	char *send = &degree_inc[0];
	bool is_sent = Serial->WriteSerialPort(send);

	if (is_sent) {
		int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
		cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
		std::string incoming = Serial->ReadSerialPort(wait_time, "json");
		cout << "Incoming: " << incoming;// << endl;
		// std::this_thread::sleep_for(250ms);
		
		std::this_thread::sleep_for(std::chrono::milliseconds(turntable_delay_ms));
	} else {
		cout << "WARNING: Serial command not sent, something went wrong.\n";
	}
}

bool generateTransform(std::string degree_inc, int num_moves) {
	// std::string degree_inc;
	// std::regex number_only("^([0-9]+|r)$");
	// validate_input("\n\nEnter Degree Interval (integer): ", degree_inc, number_only);
	// std::string visualize;
	// std::regex confirmation_only("^(y|n|r)$");
	// validate_input("\n\nVisualize after finishing the transform? (y/n): ", visualize, confirmation_only);

	// Generate transform
	bool force = config["tg_force"] == "True";
	bool visualize = config["tg_visualize"] == "True";
	std::string calibration_dir = config["tg_calibration_dir_windows"];
	std::string calibration = config["tg_calibration_mode"];
	std::string output_dir = config["tg_output_dir_windows"];

	int range = std::stoi(degree_inc) * num_moves;
	std::stringstream command_stream;
	command_stream 
		<< "python3 " 
		<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/transform_generator.py "
		<< obj_name << " "
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
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	std::string degree_inc;
	int num_moves = 0;

	cout << "Enter degrees per move: ";
	std::cin >> degree_inc;
	cout << "Enter number of moves: ";
	std::cin >> num_moves;
	// PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
	Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		scan();
		rotate_turntable(degree_inc);
		
		degree_tracker += stoi(degree_inc);
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

bool fullScan() {
	if (liveview_active){
		std::cout << "Liveview is active, please stop it before scanning." << std::endl;
		return false;
	}

	std::string degree_inc = config["degree_inc"];
	int num_moves = stoi(config["num_moves"]);
	
	// PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
	Sleep(200);
	// Start the loop timer
	auto start = std::chrono::high_resolution_clock::now();
	for (int rots = 0; rots < num_moves; rots++)
	{
		scan();
		rotate_turntable(degree_inc);
		
		degree_tracker += stoi(degree_inc);
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
	if (bool(stoi(config["collect_dslr"]))) {
		// Reset download counter to 0
		canonhandle.images_downloaded = 0;
		
		// Change Pose
		scan_folder = config["output_dir"]+"/"+obj_name;
		canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";

		EdsError err;
		cout << "Collecting DSLR images...\n";
		canonhandle.turntable_position = degree_tracker;
		err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
		cout << err << endl;
		// EdsGetEvent();
		int c = 0;
		while (canonhandle.images_downloaded < canonhandle.cameras_found && c < dslr_timeout) {
			EdsGetEvent();
			std::this_thread::sleep_for(50ms);
			c++;
		}
		cout << "DSLR Timeout Count: " << c << "/" << dslr_timeout << endl;
	}
	if (bool(stoi(config["collect_rs"]))) {
		rshandle.turntable_position = degree_tracker;
		rshandle.get_current_frame(rs_timeout);
	}

	return false;
}

bool calibrateCamera() {
	// Get the image stream from the file
	canonhandle.images_downloaded = 0;
	scan_folder = config["output_dir"]+"/"+obj_name;
	canonhandle.save_dir = scan_folder + "\\calibration";

	EdsError err;
	cout << "Collecting DSLR images...\n";
	canonhandle.turntable_position = degree_tracker;
	err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
	// EdsGetEvent();
	int c = 0;
	while (canonhandle.images_downloaded < canonhandle.cameras_found && c < dslr_timeout) {
		EdsGetEvent();
		std::this_thread::sleep_for(50ms);
		c++;
	}
	// Convert the image from the camera to an opencv2::Mat object
	cv::Mat img1 = cv::imread(canonhandle.save_dir + "/cam1_000_img.jpg");
	cv::Mat img2 = cv::imread(canonhandle.save_dir + "/cam2_000_img.jpg");
	cv::Mat img3 = cv::imread(canonhandle.save_dir + "/cam3_000_img.jpg");
	cv::Mat img4 = cv::imread(canonhandle.save_dir + "/cam4_000_img.jpg");
	cv::Mat img5 = cv::imread(canonhandle.save_dir + "/cam5_000_img.jpg");

	cv::Point2i center = cv::Point2i(img1.rows / 2, (img1.cols / 2) + 200); 
	cv::Vec3b color1 = img1.at<cv::Vec3b>(center);
	cv::Vec3b color2 = img2.at<cv::Vec3b>(center);
	cv::Vec3b color3 = img3.at<cv::Vec3b>(center);
	cv::Vec3b color4 = img4.at<cv::Vec3b>(center);
	cv::Vec3b color5 = img5.at<cv::Vec3b>(center);

	std::cout << "Color 1: B = " << (int)color1[0] << " G = " << (int)color1[1] << " R = " << (int)color1[2] << std::endl;
	std::cout << "Color 2: B = " << (int)color2[0] << " G = " << (int)color2[1] << " R = " << (int)color2[2] << std::endl;
	std::cout << "Color 3: B = " << (int)color3[0] << " G = " << (int)color3[1] << " R = " << (int)color3[2] << std::endl;
	std::cout << "Color 4: B = " << (int)color4[0] << " G = " << (int)color4[1] << " R = " << (int)color4[2] << std::endl;
	std::cout << "Color 5: B = " << (int)color5[0] << " G = " << (int)color5[1] << " R = " << (int)color5[2] << std::endl;

	// Compare the image
		//  - IF the color of the image it is within the margin of error
			// Do nothing
		//  - Otherwise
			// Save the difference
			// Change the configuration of one of the camera
			// Repeat
	return true;
}


bool setObjectName() {
	// std::string obj_name;
	cout << "\n\nEnter Object Name: ";
	std::cin >> obj_name; 
	scan_folder = config["output_dir"]+"/"+obj_name;
	// Create Main Folder
	create_folder(scan_folder);
	// Create object info.json (template)
	create_obj_info_json(config["output_dir"]);
	config["object_name"] = obj_name;
	object_info["Object Name"] = obj_name;

	// Change pose
	curr_pose = get_last_pose();
	object_info["Pose"] = curr_pose;

	// Create the .json file
	create_obj_info_json(config["output_dir"]);

	// Do handler objects need a config still?
	rshandle.update_config(config);
	rshandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\realsense";
	create_folder(rshandle.save_dir,true);
	canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
	create_folder(canonhandle.save_dir,true);

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
	for (const auto& name: canonhandle.camera_names) {
		values.push_back(name.first);
	}
	cam_val_table.add_row(values);
	
	values.clear();
	for (const auto& value: value_arr) {
		values.push_back(value);
	}
	cam_val_table.add_row(values);
	table.add_row({cam_val_table});

	std::cout << table << std::endl; 
}

bool setTV() {
	if (all_cameras) {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table, out_table);
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, out_table, value_arr);
		
		_getCurrCameraValue(value_arr);
		
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		cout << "input no. (ex. 54 = 1/250)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, canonhandle.data, out_table);
		}
	}
	else {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, tv_table, out_table);
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, out_table, value);
		
		// _getCurrCameraValue(value);
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		cout << "input no. (ex. 54 = 1/250)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Tv, canonhandle.data, out_table);
		}
		
	}

	return false;
}

bool setAV() {
	if(all_cameras) {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table, out_table);
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, out_table, value_arr);
		
		_getCurrCameraValue(value_arr);
		
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		cout << "input Av (ex. 21 = 5.6)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, canonhandle.data, out_table);
		}
	
		return false;
	}
	else {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_Av, av_table, out_table);
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_Av, out_table, value);
		
		// _getCurrCameraValue(value_arr);
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		cout << "input Av (ex. 21 = 5.6)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
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
		
		_getCurrCameraValue(value_arr);

		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		cout << "input ISOSpeed > ";
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			err = SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, out_table);
		}
		
		return false;
	}
	else {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, iso_table, out_table);
		
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, out_table, value);
		
		// _getCurrCameraValue(value);
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		cout << "input ISOSpeed (ex. 8 = ISO 200)" << endl;
		cout << ">";
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_ISOSpeed, canonhandle.data, out_table);
		}
		
		return false;
	}
}

bool setWhiteBalance() {
	if (all_cameras) {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table, out_table);
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, out_table, value_arr);
		
		_getCurrCameraValue(value_arr);
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, canonhandle.data, out_table);
		}
		
		return false;
	}
	else {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, whitebalance_table, out_table);
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, out_table, value);
		
		// _getCurrCameraValue(value);
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_WhiteBalance, canonhandle.data, out_table);
		}
		
		return false;
	}
}

bool setDriveMode() {
	if (all_cameras) {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table, out_table);
		std::vector<std::string> value_arr;
		GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, out_table, value_arr);
		
		_getCurrCameraValue(value_arr);
		std::cout << "WARNING: The modification will be applied to all cameras" << std::endl;
		cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, canonhandle.data, out_table);
		}
		
		return false;
	}
	else {
		std::map<EdsUInt32, const char*> out_table;
		GetPropertyDesc(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, drivemode_table, out_table);
		std::string value;
		GetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, out_table, value);
		
		// _getCurrCameraValue(value);
		std::cout << "Modifing " << camera_name[activeCamera] << std::endl;
		std::cout << "Current Value: " << value << std::endl;
		cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
		cout << ">";
		
		canonhandle.data = getvalue();
		if (canonhandle.data != -1)
		{
			SetProperty(activeCamera, canonhandle.bodyID[0], kEdsPropID_DriveMode, canonhandle.data, out_table);
		}
		
		return false;
	}
}

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
	std::cout << "0. All cameras" << std::endl;
	for (const auto& camera : canonhandle.cameraArray) {
		std::cout << i << ". " << camera_name[camera] << std::endl;
		i++; 
	}
	option = getvalue();

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

void _liveView(int retry = 0) {
	// Start Live View configuration
	// Download and display Image from camera
	if (retry >= 3) {
		std::cout << "Maximum tries reached!" << std::endl;
		return;
	}

	try {
		std::cout << "Starting Liveview: Try #" << retry + 1 << std::endl;
		StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
		std::this_thread::sleep_for(.5s);
		for (auto& camera : canonhandle.cameraArray) {
			// Download the image from the camera
			// liveview_th =
			DownloadEvfCommand(camera, camera_name[camera], liveview_thread_id);
		}
		DownloadEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
		EndEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
		std::cout << "Liveview sucessfully closed" << std::endl;
	}
	catch (const CameraObjectNotReadyException& err) {
		std::cout << "Error trying to open liveview: \n" << err.what() << std::endl;
		_liveView(++retry);
	}
	catch (const CameraException& err) {
		// curr_menu->addMessage("Cannot open the liveview");
		std::cout << "Error trying to open liveview: \n" << err.what() << std::endl;
	}
	// End Live View Configuration
	liveview_active = false;
}

bool getLiveView() {
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
	std::cout << "Ending Liveview..." << std::endl;
	liveview_active = false;
	for (auto& th : liveview_th) {
		if (th.joinable()) {
			std::cout << "Joining thread..." << std::endl;
			th.join();
		}
	}
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
		cout << "Enter degrees to move (r = Return): ";
		std::cin >> degree_inc;
		if (degree_inc == "r")
			break;
		else {
			char *send = &degree_inc[0];
			bool is_sent = Serial->WriteSerialPort(send);
			// TODO: Calculate wait time based on degrees entered and motor speed ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
			if (is_sent) {
				int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
				cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
				std::string incoming = Serial->ReadSerialPort(wait_time, "json");
				cout << "Incoming: " << incoming;// << endl;
				//Sleep(4000);
			} 
		}
		// Update degree tracker
		degree_tracker += std::stoi(degree_inc);
		degree_tracker = degree_tracker % 360;
	}

	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	return false;
}

bool turntablePosition() {
	cout << "\n\nEnter Turntable Position: ";
	std::cin >> degree_tracker;

	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	curr_menu->addMessage(MenuMessageStatus::SUCCESS, "Turntable Position updated sucessfully");
	
	return false;
}

bool CalibrationSubMenu(){
	MenuHandler calibration_menu_handler({
		{"1", "Press Halfway"},
		{"2", "Press Completely"},
		{"3", "Press Off"},
		{"4", "Calibrate Camera"},
	},{
		{"1", pressHalfway},
		{"2", pressCompletely},
		{"3", pressOff},
		{"4", calibrateCamera},
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

bool downloadImages(){
	DownloadImageAll(canonhandle.cameraArray, canonhandle.bodyID);
	MenuHandler::WaitUntilKeypress();

	return false;
}

int main(int argc, char* argv[])
{	
	// SETUP ----------------------------------------------------------------------------------------------
	degree_tracker = 0;
	std::smatch match_results;
    std::shared_ptr<std::thread> th = std::shared_ptr<std::thread>();
	bool loop = true;

	// Read in Parameters from config file
	fs::path executablePath(argv[0]);
	fs::path projectDir = executablePath.parent_path().parent_path().parent_path();
	std::string config_file = projectDir.string() + "\\moad_config.txt";
	cout << "Loading Config File: " << config_file << endl;
	config = loadParameters(config_file);
	
	// Print the config map
	cout << "Initial Settings Loaded:\n";
    for (auto it = config.begin(); it != config.end(); ++it) {
        cout << it->first << " = " << it->second << endl;
    }

	// Create object folder 
	scan_folder = config["output_dir"]+"/"+config["object_name"];
	obj_name = config["object_name"];
	object_info["Object Name"] = obj_name;

    create_folder(scan_folder);

	// Create object info template
	create_obj_info_json(config["output_dir"]);
	
	// Set initial Object name
	int turntable_delay_ms = stoi(config["turntable_delay_ms"]);

	// Get last pose
	std::cout << "Checking Last Pose..." << std::endl;
	curr_pose = get_last_pose();
	std::cout << "Last Pose Checked." << std::endl;


	// Setup Realsense Cameras
	rs_timeout = stoi(config["rs_timeout_sec"]) * 1000;
	if (bool(stoi(config["collect_rs"]))) {
		cout << "\nAttempting Realsense setup...\n";
		// Create RealSense Handler
		rshandle.initialize();
		rshandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\realsense";
		create_folder(rshandle.save_dir,true);
		rshandle.update_config(config);
		// Get some frames to settle autoexposure.
		rshandle.get_frames(10); // make 30 later
		// rshandle.get_current_frame();
	} else {
		cout << "\nSkipping RealSense setup, 'collect_rs=0'.\n";
	}
	

	// Setup Arduino serial port connection
	cout << "\nAttempting Serial Motor Control setup...\n";
	char com_port[] = "\\\\.\\COMX";
	com_port[7] = config["serial_com_port"].at(0);
	DWORD COM_BAUD_RATE = CBR_9600;

	Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
	if(Serial->connected_) {
		cout << "Serial connected to " << com_port << endl;
	} else {
		cout << "Error creating serial connection.\n";
	}

	// Initialization of Canon SDK
	dslr_timeout = stoi(config["dslr_timeout_sec"]) * 20;
	if (bool(stoi(config["collect_dslr"]))) {
		cout << "\nEntering DSLR Setup...\n";
		// CanonHandler canonhandle;
		canonhandle.initialize();
		canonhandle.save_dir = scan_folder + "\\pose-" + curr_pose + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		PreSetting(canonhandle.cameraArray, canonhandle.bodyID);
		// Naming of the camera
		canonhandle.rename_cameras = bool(stoi(config["dslr_name_override"]));
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

	// Change pose
	curr_pose = get_last_pose();

	object_info["Object Name"] = obj_name;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);
	object_info["Pose"] = curr_pose;
	// RUNNING MENU LOOP -----------------------------------------------------------------------------
	MenuHandler menu_handler({
		{"1", "Full Scan"},
		{"2", "Custom Scan"},
		{"3", "Collect Single Data"},
		{"4", "Set Object Name"},
		{"5", "Set Pose"},
		{"6", "Camera Calibration..."},
		{"7", "Camera Options..."},
		{"8", "Turntable Options..."},
		{"9", "Live View..."}
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
		{"9", liveViewMenu}
	}, object_info);
	menu_handler.setTitle("MOAD - CLI Menu");
	menu_handler.ClearScreen();
	menu_handler.initialize(curr_menu);

	return false;
}