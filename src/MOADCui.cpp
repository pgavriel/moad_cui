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


using namespace std::chrono_literals;
namespace fs = std::filesystem;
using std::cout;
using std::endl;

std::string control_number = "";
bool keyflag;
int degree_tracker = 0;
std::string obj_name;
std::map<std::string, std::string> config;
std::string scan_folder;
int rs_timeout;
int dslr_timeout;


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
	cout << "\n\n\n\n\n";
}

EdsInt32 getvalue()
{
	std::string input;
	std::smatch  match_results;
	std::getline(std::cin, input); // Pass if only return key is input.
	if (input != "\n")
	{
		if (std::regex_search(input, match_results, std::regex("[0-9]")))
		{
			return( stoi(input) );
		}
	}
	return(-1);
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

// TODO: Check Degree Tracker
bool fullScan() {
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
		// Collect DSLR Data
		if(bool(stoi(config["collect_dslr"]))) {
			canonhandle.images_downloaded = 0;
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
				break;
			}
		}
		
		// Issue command to move turntable.
		char *send = &degree_inc[0];
		bool is_sent = Serial.WriteSerialPort(send);

		if (is_sent) {
			int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
			cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
			std::string incoming = Serial.ReadSerialPort(wait_time, "json");
			cout << "Incoming: " << incoming;// << endl;
			// std::this_thread::sleep_for(250ms);
			
			std::this_thread::sleep_for(std::chrono::milliseconds(turntable_delay_ms));
		} else {
			cout << "WARNING: Serial command not sent, something went wrong.\n";
		}

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
		<< "-p " << output_dir;

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

	// Recalculate angle
	degree_tracker = degree_tracker % 360;

	pause_return();
	clr_screen();

	return false;
}

// TODO: Check Degree Tracker
bool collectSampleData() {
	if (bool(stoi(config["collect_dslr"]))) {
		EdsError err;
		cout << "Collecting DSLR images...\n";
		canonhandle.turntable_position = degree_tracker;
		err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
		cout << err << endl;
		EdsGetEvent();
	}
	if (bool(stoi(config["collect_rs"]))) {
		rshandle.turntable_position = degree_tracker;
		rshandle.get_current_frame(rs_timeout);
	}
	clr_screen();

	return false;
}

// TODO: Deal with local variables
bool setObjectName() {
	// std::string obj_name;
	cout << "\n\nEnter Object Name: ";
	std::cin >> obj_name; 
	scan_folder = config["output_dir"]+"/"+obj_name;
	create_folder(scan_folder);
	config["object_name"] = obj_name;
	// Do handler objects need a config still?
	rshandle.update_config(config);
	rshandle.save_dir = scan_folder + "\\realsense";
	create_folder(rshandle.save_dir,true);
	canonhandle.save_dir = scan_folder + "\\DSLR";
	create_folder(canonhandle.save_dir,true);

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

bool setTV() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);

	cout << "input no. (ex. 54 = 1/250)" << endl;
	cout << ">";

	canonhandle.data = getvalue();
	if (canonhandle.data != -1)
	{
		SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, canonhandle.data, tv_table);
		pause_return();
	}
	clr_screen();

	return false;
}

bool setAV() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
	cout << "input Av (ex. 21 = 5.6)" << endl;
	cout << ">";

	canonhandle.data = getvalue();
	if (canonhandle.data != -1)
	{
		SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, canonhandle.data, av_table);
		pause_return();
	}
	clr_screen();

	return false;
}

bool setISO() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
	cout << "input ISOSpeed (ex. 8 = ISO 200)" << endl;
	cout << ">";

	canonhandle.data = getvalue();
	if (canonhandle.data != -1)
	{
		SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, iso_table);
		pause_return();
	}
	clr_screen();

	return false;
}

bool setWhiteBalance() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
	cout << "input WhiteBalance (ex. 0 = Auto)" << endl;
	cout << ">";

	canonhandle.data = getvalue();
	if (canonhandle.data != -1)
	{
		SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, canonhandle.data, whitebalance_table);
		pause_return();
	}
	clr_screen();

	return false;
}

bool setDriveMode() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
	cout << "input Drive Mode (ex. 0 = Single shooting)" << endl;
	cout << ">";

	canonhandle.data = getvalue();
	if (canonhandle.data != -1)
	{
		SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, canonhandle.data, drivemode_table);
		pause_return();
	}
	clr_screen();

	return false;
}


bool setAEMode() {
	GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
	GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
	pause_return();
	clr_screen();

	return false;
}

bool getLiveView() {
	StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	DownloadEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	EndEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
	pause_return();
	clr_screen();

	return false;
}

// TODO: Check for unused variables
bool turntableControl () {
	cout << "\n\n\nTURNTABLE CONTROL\n\n";
	std::string degree_inc;
	while(degree_inc != "r"){
		cout << "Enter degrees to move (r = Return): ";
		std::cin >> degree_inc;
		if (degree_inc == "r")
			break;
		else {
			char *send = &degree_inc[0];
			bool is_sent = Serial.WriteSerialPort(send);
			// TODO: Calculate wait time based on degrees entered and motor speed ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
			if (is_sent) {
				int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
				cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
				std::string incoming = Serial.ReadSerialPort(wait_time, "json");
				cout << "Incoming: " << incoming;// << endl;
				//Sleep(4000);
			} 
		}
		// Update degree tracker
		degree_tracker += std::stoi(degree_inc);
		degree_tracker = degree_tracker % 360;
	}
}

// TODO: Check for unused variables
bool turntablePosition() {
	cout << "\n\nEnter Turntable Position: ";
	std::cin >> degree_tracker;
}

// TODO: Check for unused variables
bool generateTransform() {
	std::string degree_inc;
	std::regex number_only("^([0-9]+|r)$");
	validate_input("\n\nEnter Degree Interval (integer): ", degree_inc, number_only);
	std::string visualize;
	std::regex confirmation_only("^(y|n|r)$");
	validate_input("\n\nVisualize after finishing the transform? (y/n): ", visualize, confirmation_only);

	if (degree_inc != "r" && visualize != "r") {
		std::stringstream command_stream;
		command_stream 
			<< "python3 " 
			<< "C:/Users/csrobot/Documents/Version13.16.01/moad_cui/scripts/transform_generator.py "
			<< obj_name << " "
			<< "-d " << degree_inc << " ";

		if (visualize == "y") {
			command_stream << "-v";
		}

		// Execute command
		std::string command = command_stream.str(); 
		const char* c_command = command.c_str();
		std::cout << "\nExecuting Command: " << command; 
		system(c_command);
	}

	
	// Clean screen
	clr_screen();

	return false;
}

bool option1(){
	std::cout << "Option 1" << std::endl;
	return false;
}
bool option2(){
	std::cout << "Option 2" << std::endl;
	return false;
}
bool option3(){
	std::cout << "Option 3" << std::endl;
	return false;
}
bool option4(){
	MenuHandler calibration_menu_handler({
		{"1", "Press Halfway"},
		{"2", "Press Completely"},
		{"3", "Press Off"}
	},{
		{"1", option1},
		{"2", option1},
		{"3", option1},
	});
	calibration_menu_handler.setTitle("Calibration Menu");
	calibration_menu_handler.initialize();
	return true;
}
bool option5(){
	MenuHandler camera_menu_handler({
		{"1", "TV"},
		{"2", "AV"},
		{"3", "ISO"},
		{"4", "White Balance"},
		{"5", "Drive Mode"},
		{"6", "AE Mode"},
	},{
		{"1", option1},
		{"2", option1},
		{"3", option1},
		{"4", option1},
		{"5", option1},
		{"6", option1},
	});
	camera_menu_handler.setTitle("Camera Options");
	camera_menu_handler.initialize();
	return true;
}
bool option6(){
	MenuHandler turntable_handler({
		{"1", "Turntable Control"},
		{"2", "Turntable Position"},
	},{
		{"1", option1},
		{"2", option1},
	});
	turntable_handler.setTitle("Turntable Options");
	turntable_handler.initialize();
	return true;
}
bool option7(){
	std::cout << "Option 7" << std::endl;
	return false;
}
bool option8(){
	std::cout << "Option 8" << std::endl;
	return false;
}

int main(int argc, char* argv[])
{	
	// SETUP ----------------------------------------------------------------------------------------------
	MenuHandler menu_handler({
		{"1", "Full Scan"},
		{"2", "Collect Single Data"},
		{"3", "Set Object Name"},
		{"4", "Camera Calibration"},
		{"5", "Camera Options"},
		{"6", "Turntable Options"},
		{"7", "Live View"},
		{"8", "Configuration"},
	},
	{
		{"1", option1},
		{"2", option2},
		{"3", option3},
		{"4", option4},
		{"5", option5},
		{"6", option6},
		{"7", option7},
		{"8", option8}
	});
	menu_handler.setTitle("MOAD - CLI Menu");
	menu_handler.initialize();

	degree_tracker = 0;
	obj_name;
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
	// Set initial Object name
	obj_name = config["object_name"];
	int turntable_delay_ms = stoi(config["turntable_delay_ms"]);


	// Setup Realsense Cameras
	RealSenseHandler rshandle;
	rs_timeout = stoi(config["rs_timeout_sec"]) * 1000;
	if (bool(stoi(config["collect_rs"]))) {
		cout << "\nAttempting Realsense setup...\n";
		// Create RealSense Handler
		rshandle.initialize();
		rshandle.save_dir = scan_folder + "\\realsense";
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

	SimpleSerial Serial(com_port, COM_BAUD_RATE);
	if(Serial.connected_) {
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
		canonhandle.save_dir = scan_folder + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		PreSetting(canonhandle.cameraArray, canonhandle.bodyID);
		canonhandle.rename_cameras = bool(stoi(config["dslr_name_override"]));
		canonhandle.camera_names["1"] = config["cam1_rename"];
		canonhandle.camera_names["2"] = config["cam2_rename"];
		canonhandle.camera_names["3"] = config["cam3_rename"];
		canonhandle.camera_names["4"] = config["cam4_rename"];
		canonhandle.camera_names["5"] = config["cam5_rename"];
	} else {
		cout << "\nSkipping DSLR setup, 'collect_dslr=0'.\n";
	}


	// RUNNING MENU LOOP -----------------------------------------------------------------------------
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
	// 					bool is_sent = Serial.WriteSerialPort(send);

	// 					if (is_sent) {
	// 						int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
	// 						cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
	// 						std::string incoming = Serial.ReadSerialPort(wait_time, "json");
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
	// 						bool is_sent = Serial.WriteSerialPort(send);
	// 						// TODO: Calculate wait time based on degrees entered and motor speed ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
	// 						if (is_sent) {
	// 							int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
	// 							cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
	// 							std::string incoming = Serial.ReadSerialPort(wait_time, "json");
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
	if (canonhandle.cameraList != NULL)
	{
		EdsRelease(canonhandle.cameraList);
	}

	// Release Camera
	for (i = 0; i < canonhandle.cameraArray.size(); i++)
	{
		if (canonhandle.cameraArray[i] != NULL)
		{
			EdsRelease(canonhandle.cameraArray[i]);
			canonhandle.cameraArray[i] = NULL;
		}
	}
	//Remove elements before looping. Memory is automatically freed at the destructor of the vector when it leaves the scope.
	canonhandle.cameraArray.clear();
	canonhandle.bodyID.clear();
	clr_screen();

	
	if (isSDKLoaded)
	{
		EdsTerminateSDK();
	}

	return false;
}