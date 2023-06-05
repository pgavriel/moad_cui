#include <iostream>
#include <windows.h>
#include <filesystem>
#include <iterator>
#include <list>
#include <regex>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <memory>

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

std::string control_number = "";
bool keyflag;

void CheckKey()// After key is entered, _ endthread is automatically called.
{
	std::cin >> control_number;
	std::cin.ignore();// ignore "\n"
	keyflag = true;
}

void clr_screen()
{
	// std::cout << "\033[2J"; // screen clr
	// std::cout << "\033[0;0H"; // move low=0, column=0
	std::cout << "\n\n\n\n\n";
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
	std::cout << "\n" << "Press the RETURN key." << std::endl;
	getvalue();
}

int create_folder(std::string path, bool quiet = false) {
	if (!quiet) std::cout << std::endl;
	if (!fs::exists(path)) {
        // Create the folder and any necessary higher level folders
        if (fs::create_directories(path)) {
            std::cout << "Folder created: " << path << std::endl;
        } else {
            std::cerr << "Failed to create folder: " << path << std::endl;
            return 1;
        }
    } else {
		if (!quiet)
			std::cout << "WARNING!: Folder already exists: " << path << std::endl
				<< "\tYou may accidentally overwrite data.\n";
    }
	return 0;
}

int main(int argc, char* argv[])
{	
	// Read in Parameters from config file
	fs::path executablePath(argv[0]);
	fs::path projectDir = executablePath.parent_path().parent_path().parent_path();
	std::string config_file = projectDir.string() + "\\moad_config.txt";
	std::cout << "Loading Config File: " << config_file << std::endl;
	std::map<std::string, std::string> config = loadParameters(config_file);
	// Print the config map
	std::cout << "Initial Settings Loaded:\n";
    for (auto it = config.begin(); it != config.end(); ++it) {
        std::cout << it->first << " = " << it->second << std::endl;
    }
	// Create object folder 
	std::string scan_folder = config["output_dir"]+"/"+config["object_name"];
    create_folder(scan_folder);

	// Setup Realsense Cameras
	RealSenseHandler rshandle;
	if (bool(stoi(config["collect_rs"]))) {
		std::cout << "\nAttempting Realsense setup...\n";
		// Create RealSense Handler
		rshandle.initialize();
		rshandle.save_dir = scan_folder + "\\realsense";
		create_folder(rshandle.save_dir,true);
		rshandle.update_config(config);
		// Get some frames to settle autoexposure.
		rshandle.get_frames(10); // make 30 later
		// rshandle.get_current_frame();
	} else {
		std::cout << "Skipping RealSense setup, 'collect_rs=0'.\n";
	}
	

	// Setup Arduino serial port connection
	std::cout << "\nAttempting Serial Motor Control setup...\n";
	char com_port[] = "\\\\.\\COMX";
	com_port[7] = config["serial_com_port"].at(0);
	DWORD COM_BAUD_RATE = CBR_9600;

	SimpleSerial Serial(com_port, COM_BAUD_RATE);
	if(Serial.connected_) {
		std::cout << "Serial connected to " << com_port << std::endl;
	} else {
		std::cout << "Error creating serial connection.\n";
	}


	// Initialization of Canon SDK
	if (bool(stoi(config["collect_dslr"]))) {
		std::cout << "\nEntering DSLR Setup...\n";
		// CanonHandler canonhandle;
		canonhandle.initialize();
		canonhandle.save_dir = scan_folder + "\\DSLR";
		create_folder(canonhandle.save_dir,true);
		PreSetting(canonhandle.cameraArray, canonhandle.bodyID);
	} else {
		std::cout << "Skipping DSLR setup, 'collect_dslr=0'.\n";
	}
    

	// pause_return();

	// int i;
	int degree_tracker = 0;
	std::smatch  match_results;
    std::shared_ptr<std::thread> th = std::shared_ptr<std::thread>();
	bool loop = true;

	

	clr_screen();

	while (true)
	{
		//control menu
		if (loop)
		{
			keyflag = false; // initialize keyflag
			//_beginthread(CheckKey, 0, NULL); // Start waiting for keystrokes
			th = std::make_shared<std::thread>(CheckKey);

			std::cout << "--------------------------------" << std::endl;
			std::cout
				<< "[ 1] Collect & Download Data \n"
				<< "[ 2] Press Halfway \n"
				<< "[ 3] Press Completely \n"
				<< "[ 4] Press Off \n"
				<< "[ 5] TV \n"
				<< "[ 6] AV \n"
				<< "[ 7] ISO \n"
				<< "[ 8] White Balance \n"
				<< "[ 9] Drive Mode \n"
				<< "[10] AE Mode (read only) \n"
				<< "[11] Get Live View \n"
				<< "[12] File Download \n"
				<< "[13] Run Photo Loop \n" 
				<< "[14] Turntable Control \n"
				<< "[15] Set Object Name \n" << std::endl;
			std::cout << "--------------------------------" << std::endl;
			std::cout << "Enter the number of the control.\n"
				<< "\tor 'r' (=Return)" << std::endl;
			std::cout << "> ";
			loop = false;
		}

		if (keyflag == true) // Within this scope, the CheckKey thread is terminated
		{

			th->join();
			
			if (std::regex_search(control_number, match_results, std::regex("[0-9]")))
			{
				// Take Picture
				if (control_number == "1")
				{	
					if (bool(stoi(config["collect_dslr"]))) {
						EdsError err;
						std::cout << "Collecting DSLR images...\n";
						canonhandle.turntable_position = degree_tracker;
						err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
						std::cout << err << std::endl;
						EdsGetEvent();
					}
					if (bool(stoi(config["collect_rs"]))) {
						rshandle.turntable_position = degree_tracker;
						rshandle.get_current_frame(15000);
					}
					clr_screen();
					loop = true;
					continue;
				}
				// Press halfway
				else if (control_number == "2")
				{
					PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
					clr_screen();
					loop = true;
					continue;
				}
				// Press Completely
				else if (control_number == "3")
				{
					PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Completely);
					clr_screen();
					loop = true;
					continue;
				}
				// Press Completely
				else if (control_number == "4")
				{
					PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_OFF);
					clr_screen();
					loop = true;
					continue;
				}
				// TV
				else if (control_number == "5")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, tv_table);

					std::cout << "input no. (ex. 54 = 1/250)" << std::endl;
					std::cout << ">";

					canonhandle.data = getvalue();
					if (canonhandle.data != -1)
					{
						SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Tv, canonhandle.data, tv_table);
						pause_return();
					}
					clr_screen();
					loop = true;
					continue;
				}
				// Av
				else if (control_number == "6")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, av_table);
					std::cout << "input Av (ex. 21 = 5.6)" << std::endl;
					std::cout << ">";

					canonhandle.data = getvalue();
					if (canonhandle.data != -1)
					{
						SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_Av, canonhandle.data, av_table);
						pause_return();
					}
					clr_screen();
					loop = true;
					continue;
				}
				// ISO
				else if (control_number == "7")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, iso_table);
					std::cout << "input ISOSpeed (ex. 8 = ISO 200)" << std::endl;
					std::cout << ">";

					canonhandle.data = getvalue();
					if (canonhandle.data != -1)
					{
						SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_ISOSpeed, canonhandle.data, iso_table);
						pause_return();
					}
					clr_screen();
					loop = true;
					continue;
				}
				// White Balanse
				else if (control_number == "8")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
					std::cout << "input WhiteBalance (ex. 0 = Auto)" << std::endl;
					std::cout << ">";

					canonhandle.data = getvalue();
					if (canonhandle.data != -1)
					{
						SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_WhiteBalance, canonhandle.data, whitebalance_table);
						pause_return();
					}
					clr_screen();
					loop = true;
					continue;
				}
				// Drive Mode
				else if (control_number == "9")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, drivemode_table);
					std::cout << "input Drive Mode (ex. 0 = Single shooting)" << std::endl;
					std::cout << ">";

					canonhandle.data = getvalue();
					if (canonhandle.data != -1)
					{
						SetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_DriveMode, canonhandle.data, drivemode_table);
						pause_return();
					}
					clr_screen();
					loop = true;
					continue;
				}
				// AE Mode
				else if (control_number == "10")
				{
					GetProperty(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
					GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, kEdsPropID_AEMode, AEmode_table);
					pause_return();
					clr_screen();
					loop = true;
					continue;
				}
				// Download EVF image
				else if (control_number == "11")
				{
					StartEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
					DownloadEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
					EndEvfCommand(canonhandle.cameraArray, canonhandle.bodyID);
					pause_return();
					clr_screen();
					loop = true;
					continue;
				}
				// Download All Image
				else if (control_number == "12")
				{
					DownloadImageAll(canonhandle.cameraArray, canonhandle.bodyID);
					pause_return();
					clr_screen();
					loop = true;
					continue;
				}
				//Run photo loop
				else if (control_number == "13")
				{	
					std::string degree_inc;
					int num_moves = 0;
					// 
					std::cout << "Enter degrees per move: ";
					std::cin >> degree_inc;
					std::cout << "Enter number of moves: ";
					std::cin >> num_moves;
					PressShutter(canonhandle.cameraArray, canonhandle.bodyID, kEdsCameraCommand_ShutterButton_Halfway);
					Sleep(200);
					for (int rots = 0; rots < num_moves; rots++)
					{
						char *send = &degree_inc[0];
						bool is_sent = Serial.WriteSerialPort(send);

						if (is_sent) {
							int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
							std::cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
							std::string incoming = Serial.ReadSerialPort(wait_time, "json");
							std::cout << "Incoming: " << incoming;// << std::endl;
							std::this_thread::sleep_for(250ms);
						} 

						if(bool(stoi(config["collect_dslr"]))) {
							canonhandle.images_downloaded = 0;
							int c = 0;
							EdsError err;
							std::cout << "Collecting DSLR images...\n";
							canonhandle.turntable_position = degree_tracker;
							err = TakePicture(canonhandle.cameraArray, canonhandle.bodyID);
							std::cout << "Result: " << err << std::endl;
							while (canonhandle.images_downloaded < canonhandle.cameras_found && c < 50) {
								EdsGetEvent();
								std::this_thread::sleep_for(100ms);
								c++;
							}
							std::cout << "C: " << c << std::endl;
							
							EdsGetEvent();
							std::this_thread::sleep_for(50ms);
							EdsGetEvent();
							std::this_thread::sleep_for(50ms);
							EdsGetEvent();
							std::this_thread::sleep_for(50ms);
							EdsGetEvent();
							std::this_thread::sleep_for(50ms);
						}
						if(bool(stoi(config["collect_rs"]))) {
							rshandle.turntable_position = degree_tracker;
							rshandle.get_current_frame(15000);
						}
						

						degree_tracker += stoi(degree_inc);
						std::cout << "Image " << rots+1 << "/" << num_moves << " taken. " << std::endl;
					}
					pause_return();
					clr_screen();
					loop = true;
					continue;
				}
				// Turntable control
				else if (control_number == "14")
				{	
					std::cout << "\n\n\nTURNTABLE CONTROL\n\n";
					std::string degree_inc;
					while(degree_inc != "r"){
						std::cout << "Enter degrees to move (r = Return): ";
						std::cin >> degree_inc;
						if (degree_inc == "r")
							break;
						else {
							char *send = &degree_inc[0];
							bool is_sent = Serial.WriteSerialPort(send);
							// TODO: Calculate wait time based on degrees entered and motor speed ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
							if (is_sent) {
								int wait_time = std::ceil(((abs(stoi(degree_inc))*200)+500)/1000)+5;
								std::cout << "Message sent, waiting up to " << wait_time << " seconds.\n";
								std::string incoming = Serial.ReadSerialPort(wait_time, "json");
								std::cout << "Incoming: " << incoming;// << std::endl;
								//Sleep(4000);
							} 
						}
					}
					

					pause_return();
					clr_screen();
					loop = true;
				}
				// Set Object Name
				else if (control_number == "15")
				{	
					std::string obj_name;
					std::cout << "\n\nEnter Object Name: ";
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

					loop = true;
				}
				else
				{
					clr_screen();
					loop = true;
					continue;
				}
			}
			// Return
			else if (std::regex_search(control_number, match_results, std::regex("r", std::regex_constants::icase)))
			{
				// clr_screen();
				keyflag = false;
				loop = false;
				break;
			}
			else
			{
				clr_screen();
				loop = true;
				continue;
			}
		}
		// send getevent periodically
		// std::cout << "Event check.\n";
		EdsGetEvent();
		std::this_thread::sleep_for(200ms);
	}

	// //Release camera list
	// if (canonhandle.cameraList != NULL)
	// {
	// 	EdsRelease(canonhandle.cameraList);
	// }

	//Release Camera

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

	// 
	// if (isSDKLoaded)
	// {
	// 	EdsTerminateSDK();
	// }

	return FALSE;
}