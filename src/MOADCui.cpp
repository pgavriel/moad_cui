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

std::thread* liveview_th;
bool liveview_active = false;

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

void CheckKey()// After key is entered, _ endthread is automatically called.
{
	std::cin >> control_number;
	std::cin.ignore();// ignore "\n"
	keyflag = true;
}

void clr_screen()
{
	// cout << "\033[2J"; // screen clr
	// cout << "\033[0;0H"; // move low=0, column=0
	// cout << "\n\n\n\n\n";
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

void pause_return()
{
	//	system("pause");
	cout << "\n" << "Press the RETURN key." << endl;
	getvalue();
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
	std::string filename = object_info["Object Name"];
	if (fs::exists(path)){
		std::stringstream command_stream;
		command_stream 
			<< "python3 " 
			<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/create_object_info.py "
			<< filename << " "
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
				if (fs::is_empty(path + "/pose-" + letter + "/DSLR")) {
					return letter; // Current pose
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
	std::string degree_inc;
	int num_moves = 0;
	// 
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

	// Generate transform
	generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;

	pause_return();
	clr_screen();

	return true;
}

bool fullScan() {
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

	// Generate transform
	generateTransform(degree_inc, num_moves);

	// Recalculate angle
	degree_tracker = degree_tracker % 360;
	object_info["Turntable Pos"] = std::to_string(degree_tracker);

	// Change Pose
	curr_pose++;
	object_info["Pose"] = curr_pose;

	pause_return();
	clr_screen();

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
	clr_screen();

	return false;
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
	clr_screen();

	return false;
}

bool pressCompletely() {
	PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Completely);
	clr_screen();
	
	return false;
}

bool pressOff() {
	PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_OFF);
	clr_screen();

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
		liveview_active = true;
		liveview_th = new std::thread(_liveView, 0);
	}
	else {
		std::cout << "The liveview is active" << std::endl;
	}

	return true;
}

bool endLiveView() {
	if(liveview_th->joinable()) {
		liveview_th->join();
	}
	else {
		std::cout << "There is no liveview active" << std::endl;
	}

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
		{"3", "Press Off"}
	},{
		{"1", pressHalfway},
		{"2", pressCompletely},
		{"3", pressOff},
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
	pause_return();
	clr_screen();

	return false;
}

bool option8(){
	std::cout << "Change config options" << std::endl;
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
    create_folder(scan_folder);
	// Create object info template
	create_obj_info_json(config["output_dir"]);
	// Set initial Object name
	obj_name = config["object_name"];
	int turntable_delay_ms = stoi(config["turntable_delay_ms"]);

	// Get last pose
	curr_pose = get_last_pose();

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

	// Retrieve possible configuration (Probably gotta do that once, then save it in a file or something)


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
	// while (true)
	// {
	// 	//control menu
	// 	if (loop)
	// 	{
	// 		keyflag = false; // initialize keyflag
	// 		//_beginthread(CheckKey, 0, NULL); // Start waiting for keystrokes
	// 		th = std::make_shared<std::thread>(CheckKey);

	// 		cout << "--------------------------------" << endl;
	// 		cout
	// 			<< "[ 1] Collect & Download Data \n"
	// 			<< "[ 2] Press Halfway \n"
	// 			<< "[ 3] Press Completely \n"
	// 			<< "[ 4] Press Off \n"
	// 			<< "[ 5] TV \n"
	// 			<< "[ 6] AV \n"
	// 			<< "[ 7] ISO \n"
	// 			<< "[ 8] White Balance \n"
	// 			<< "[ 9] Drive Mode \n"
	// 			<< "[10] AE Mode (read only) \n"
	// 			<< "[11] Get Live View \n"
	// 			<< "[12] File Download \n"
	// 			<< "[13] Run Object Scan \n" 
	// 			<< "[14] Turntable Control \n"
	// 			<< "[15] Set Object Name \n"
	// 			<< "[16] Set Turntable Position \n"
	// 			<< "[17] Run Transform Generator \n";
	// 		cout << "--------------------------------" << endl;
	// 		cout << "Object:\t\t" << obj_name << "\nPosition:\t" << degree_tracker << endl;
	// 		cout << "--------------------------------" << endl;
	// 		cout << "Enter the number of the control.\n"
	// 			<< "\tor 'r' (=Return)" << endl;
	// 		cout << "> ";
	// 		loop = false;
	// 	}

	// 	if (keyflag == true) // Within this scope, the CheckKey thread is terminated
	// 	{

	// 		th->join();
			
	// 		if (std::regex_search(control_number, match_results, std::regex("[0-9]")))
	// 		{
	// 			// Take Picture
	// 			if (control_number == "1")
	// 			{	
	// 				if (bool(stoi(config["collect_dslr"]))) {
	// 					EdsError err;
	// 					cout << "Collecting DSLR images...\n";
	// 					canonhandle.turntable_position = degree_tracker;
	// 					err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
	// 					cout << err << endl;
	// 					EdsGetEvent();
	// 				}
	// 				if (bool(stoi(config["collect_rs"]))) {
	// 					rshandle.turntable_position = degree_tracker;
	// 					rshandle.get_current_frame(rs_timeout);
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Press halfway
	// 			else if (control_number == "2")
	// 			{
	// 				PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Press Completely
	// 			else if (control_number == "3")
	// 			{
	// 				PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Completely);
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Press Completely
	// 			else if (control_number == "4")
	// 			{
	// 				PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_OFF);
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// TV
	// 			else if (control_number == "5")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);

	// 				cout << "input no. (ex. 54 = 1/250)" << endl;
	// 				cout << ">";

	// 				canonhandle.data = getvalue();
	// 				if (canonhandle.data != -1)
	// 				{
	// 					SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, canonhandle.data, tv_table);
	// 					pause_return();
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Av
	// 			else if (control_number == "6")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
	// 				cout << "input Av (ex. 21 = 5.6)" << endl;
	// 				cout << ">";

	// 				canonhandle.data = getvalue();
	// 				if (canonhandle.data != -1)
	// 				{
	// 					SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, canonhandle.data, av_table);
	// 					pause_return();
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// ISO
	// 			else if (control_number == "7")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
	// 				cout << "input ISOSpeed (ex. 8 = ISO 200)" << endl;
	// 				cout << ">";

	// 				canonhandle.data = getvalue();
	// 				if (canonhandle.data != -1)
	// 				{
	// 					SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, iso_table);
	// 					pause_return();
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// White Balanse
	// 			else if (control_number == "8")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
	// 				cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
	// 				cout << ">";

	// 				canonhandle.data = getvalue();
	// 				if (canonhandle.data != -1)
	// 				{
	// 					SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, canonhandle.data, whitebalance_table);
	// 					pause_return();
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Drive Mode
	// 			else if (control_number == "9")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
	// 				cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
	// 				cout << ">";

	// 				canonhandle.data = getvalue();
	// 				if (canonhandle.data != -1)
	// 				{
	// 					SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, canonhandle.data, drivemode_table);
	// 					pause_return();
	// 				}
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// AE Mode
	// 			else if (control_number == "10")
	// 			{
	// 				GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
	// 				GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
	// 				pause_return();
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Download EVF image
	// 			else if (control_number == "11")
	// 			{
	// 				StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	// 				DownloadEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	// 				EndEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	// 				pause_return();
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Download All Image
	// 			else if (control_number == "12")
	// 			{
	// 				DownloadImageAll(canonhandle.cameraArray, canonhandle.bodyID);
	// 				pause_return();
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			//Run Object Scan
	// 			else if (control_number == "13")
	// 			{	
	// 				std::string degree_inc;
	// 				int num_moves = 0;
	// 				// 
	// 				cout << "Enter degrees per move: ";
	// 				std::cin >> degree_inc;
	// 				cout << "Enter number of moves: ";
	// 				std::cin >> num_moves;
	// 				// PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
	// 				Sleep(200);
	// 				// Start the loop timer
    // 				auto start = std::chrono::high_resolution_clock::now();
	// 				for (int rots = 0; rots < num_moves; rots++)
	// 				{
	// 					// Collect DSLR Data
	// 					if(bool(stoi(config["collect_dslr"]))) {
	// 						canonhandle.images_downloaded = 0;
	// 						int c = 0;
	// 						// int timeout = stoi(config["dslr_timeout_sec"]) * 20; // 20 = 1s
	// 						EdsError err;
	// 						cout << "Getting DSLR Data...\n";
	// 						canonhandle.turntable_position = degree_tracker;
	// 						err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
	// 						// cout << "Result: " << err << endl;
	// 						while (canonhandle.images_downloaded < canonhandle.cameras_found && c < dslr_timeout) {
	// 							EdsGetEvent();
	// 							std::this_thread::sleep_for(50ms);
	// 							c++;
	// 						}
	// 						cout << "DSLR Timeout Count: " << c << "/" << dslr_timeout << endl;
	// 					}

	// 					// Collect RealSense Data
	// 					if(bool(stoi(config["collect_rs"]))) {
	// 						rshandle.turntable_position = degree_tracker;
	// 						rshandle.get_current_frame(rs_timeout);
	// 						if (rshandle.fail_count > 0) {
	// 							cout << "RS Failure - " << rshandle.fail_count << endl;
	// 							break;
	// 						}
	// 					}
						
	// 					// Issue command to move turntable.
	// 					char *send = &degree_inc[0];
	// 					bool is_sent = Serial->WriteSerialPort(send);

	// 					if (is_sent) {
	// 						int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
	// 						cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
	// 						std::string incoming = Serial->ReadSerialPort(wait_time, "json");
	// 						cout << "Incoming: " << incoming;// << endl;
	// 						// std::this_thread::sleep_for(250ms);
							
	// 						std::this_thread::sleep_for(std::chrono::milliseconds(turntable_delay_ms));
	// 					} else {
	// 						cout << "WARNING: Serial command not sent, something went wrong.\n";
	// 					}

	// 					degree_tracker += stoi(degree_inc);
	// 					cout << "Image " << rots+1 << "/" << num_moves << " taken. " << endl;
	// 				}
	// 				// Stop the loop timer
    // 				auto end = std::chrono::high_resolution_clock::now();
	// 				// Calculate the elapsed time
	// 				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// 				// Convert the duration to minutes, seconds, and milliseconds
	// 				int minutes = duration.count() / 60000;
	// 				int seconds = (duration.count() % 60000) / 1000;
	// 				cout << "Scan Time: " << std::setfill('0') << std::setw(2) << minutes << ":" 
	// 					<< std::setfill('0') << std::setw(2) << seconds << endl;
	// 				cout << "RS Fail Count: " << rshandle.fail_count << endl;
	// 				std::this_thread::sleep_for(3000ms);

	// 				// Generate transform
	// 				bool force = config["tg_force"] == "True";
	// 				bool visualize = config["tg_visualize"] == "True";
	// 				std::string calibration_dir = config["tg_calibration_dir_windows"];
	// 				std::string calibration = config["tg_calibration_mode"];
	// 				std::string output_dir = config["tg_output_dir_windows"];

	// 				int range = std::stoi(degree_inc) * num_moves;
	// 				std::stringstream command_stream;
	// 				command_stream 
	// 					<< "python3 " 
	// 					<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/transform_generator.py "
	// 					<< obj_name << " "
	// 					<< "-d " << degree_inc << " "
	// 					<< "-r " << range << " "
	// 					<< "-c " << calibration << " "
	// 					<< "--calibration_dir " << calibration_dir << " "
	// 					<< "-p " << output_dir;

	// 				if (visualize) {
	// 					command_stream << " -v";
	// 				}

	// 				if (force) {
	// 					command_stream << " -f";
	// 				}

	// 				// Execute command
	// 				std::string command = command_stream.str(); 
	// 				const char* c_command = command.c_str();
	// 				std::cout << "\nExecuting Command: " << command; 
	// 				system(c_command);

	// 				// Recalculate angle
	// 				degree_tracker = degree_tracker % 360;

	// 				pause_return();
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 			// Turntable control
	// 			else if (control_number == "14")
	// 			{	
	// 				cout << "\n\n\nTURNTABLE CONTROL\n\n";
	// 				std::string degree_inc;
	// 				while(degree_inc != "r"){
	// 					cout << "Enter degrees to move (r = Return): ";
	// 					std::cin >> degree_inc;
	// 					if (degree_inc == "r")
	// 						break;
	// 					else {
	// 						char *send = &degree_inc[0];
	// 						bool is_sent = Serial->WriteSerialPort(send);
	// 						// TODO: Calculate wait time based on degrees entered and motor speed ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// 						if (is_sent) {
	// 							int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
	// 							cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
	// 							std::string incoming = Serial->ReadSerialPort(wait_time, "json");
	// 							cout << "Incoming: " << incoming;// << endl;
	// 							//Sleep(4000);
	// 						} 
	// 					}
	// 					// Update degree tracker
	// 					degree_tracker += std::stoi(degree_inc);
	// 					degree_tracker = degree_tracker % 360;
	// 				}
					

	// 				// pause_return();
	// 				clr_screen();
	// 				loop = true;
	// 			}
	// 			// Set Object Name
	// 			else if (control_number == "15")
	// 			{	
	// 				// std::string obj_name;
	// 				cout << "\n\nEnter Object Name: ";
	// 				std::cin >> obj_name; 
	// 				scan_folder = config["output_dir"]+"/"+obj_name;
	// 				create_folder(scan_folder);
	// 				config["object_name"] = obj_name;
	// 				// Do handler objects need a config still?
	// 				rshandle.update_config(config);
	// 				rshandle.save_dir = scan_folder + "\\realsense";
	// 				create_folder(rshandle.save_dir,true);
	// 				canonhandle.save_dir = scan_folder + "\\DSLR";
	// 				create_folder(canonhandle.save_dir,true);

	// 				loop = true;
	// 			}
	// 			// Set Turntable Position
	// 			else if (control_number == "16")
	// 			{	
	// 				cout << "\n\nEnter Turntable Position: ";
	// 				std::cin >> degree_tracker; 
					
	// 				loop = true;
	// 			}
	// 			// Call the transform generator
	// 			else if (control_number == "17"){
	// 				std::string degree_inc;
	// 				std::regex number_only("^([0-9]+|r)$");
	// 				validate_input("\n\nEnter Degree Interval (integer): ", degree_inc, number_only);
	// 				std::string visualize;
	// 				std::regex confirmation_only("^(y|n|r)$");
	// 				validate_input("\n\nVisualize after finishing the transform? (y/n): ", visualize, confirmation_only);

	// 				if (degree_inc != "r" && visualize != "r") {
	// 					std::stringstream command_stream;
	// 					command_stream 
	// 						<< "python3 " 
	// 						<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/transform_generator.py "
	// 						<< obj_name << " "
	// 						<< "-d " << degree_inc << " ";

	// 					if (visualize == "y") {
	// 						command_stream << "-v";
	// 					}

	// 					// Execute command
	// 					std::string command = command_stream.str(); 
	// 					const char* c_command = command.c_str();
	// 					std::cout << "\nExecuting Command: " << command; 
	// 					system(c_command);
	// 				}

					
	// 				// Clean screen
	// 				clr_screen();
	// 				loop = true;
	// 				continue; 
	// 			}
	// 			else
	// 			{
	// 				clr_screen();
	// 				loop = true;
	// 				continue;
	// 			}
	// 		}
	// 		// Return
	// 		else if (std::regex_search(control_number, match_results, std::regex("r", std::regex_constants::icase)))
	// 		{
	// 			// clr_screen();
	// 			keyflag = false;
	// 			loop = false;
	// 			break;
	// 		}
	// 		else
	// 		{
	// 			clr_screen();
	// 			loop = true;
	// 			continue;
	// 		}
	// 	}
	// 	// send getevent periodically
	// 	// cout << "Event check.\n";
	// 	EdsGetEvent();
	// 	std::this_thread::sleep_for(200ms);
	// }

	//Release camera list
	// if (canonhandle.cameraList != NULL)
	// {
	// 	EdsRelease(canonhandle.cameraList);
	// }

	// // Release Camera
	// for (i = 0; i < canonhandle.cameraArray.size(); i++)
	// {
	// 	if (canonhandle.cameraArray[i] != NULL)
	// 	{
	// 		EdsRelease(canonhandle.cameraArray[i]);
	// 		canonhandle.cameraArray[i] = NULL;
	// 	}
	// }
	// //Remove elements before looping. Memory is automatically freed at the destructor of the vector when it leaves the scope.
	// canonhandle.cameraArray.clear();
	// canonhandle.bodyID.clear();
	// clr_screen();

	
	// if (isSDKLoaded)
	// {
	// 	EdsTerminateSDK();
	// }

	return false;
}