#include <iostream>
#include <windows.h>
#include <iterator>
#include <list>
#include <regex>
#include <string>
#include <vector>
#include <thread>
#include <memory>
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
	std::cout << "\033[2J"; // screen clr
	std::cout << "\033[0;0H"; // move low=0, column=0
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

int main()
{	
	// Setup Realsense Cameras
	// try {
	std::cout << "\nAttempting Realsense setup...\n";
	// Create RealSense Handler
	RealSenseHandler rshandle;
	// Get some frames to settle autoexposure.
	rshandle.get_frames(10); // make 30 later
	rshandle.get_current_frame();

	// Setup Arduino serial port connection
	std::cout << "\nAttempting Serial Motor Control setup...\n";
	char com_port[] = "\\\\.\\COM5";
	DWORD COM_BAUD_RATE = CBR_9600;

	SimpleSerial Serial(com_port, COM_BAUD_RATE);

	if(Serial.connected_) {
		std::cout << "Serial connected to " << com_port << std::endl;
	} else {
		std::cout << "Error creating serial connection.\n";
	}

	EdsError	 err = EDS_ERR_OK;
	EdsCameraListRef cameraList = NULL;
	EdsUInt32	 count = 0;
	EdsUInt32 data;
	bool	isSDKLoaded = false;
	
	// Initialization of Canon SDK
	err = EdsInitializeSDK();
	if (err == EDS_ERR_OK)
	{
		isSDKLoaded = true;
	}

	std::vector<EdsCameraRef> cameraArray;
	std::vector<EdsUInt64> bodyID;
	EdsCameraRef camera;
	EdsUInt32 min = 0, max = 0, i = 0;
	std::smatch  match_results;
	std::shared_ptr<std::thread> th = std::shared_ptr<std::thread>();


	
	
	std::cout << "\nEntering DSLR Setup...\n";
	while (true)
	{
		bool loop = true;

		while (true)
		{
			if (loop)
			{
				//Acquisition of camera list
				if (err == EDS_ERR_OK)
				{
					err = EdsGetCameraList(&cameraList);
				}

				//Acquisition of number of Cameras
				if (err == EDS_ERR_OK)
				{
					err = EdsGetChildCount(cameraList, &count);
					if (count == 0)
					{
						std::cout << "Cannot detect any camera" << std::endl;
						pause_return();
						exit(EXIT_FAILURE);
					}
					else if (count > 30)
					{
						std::cout << "Too many cameras detected" << std::endl;
						pause_return();
						exit(EXIT_FAILURE);
					}
					std::cout << count << "cameras detected." << std::endl;
				}

				std::cout << "number\t" << "Device Description" << std::endl;
				std::cout << "------+-------------------------" << std::endl;

				//Acquisition of camera at the head of the list
				for (i = 0; i < count; i++)
				{
					if (err == EDS_ERR_OK)
					{
						err = EdsGetChildAtIndex(cameraList, i, &camera);
						EdsDeviceInfo deviceInfo;
						err = EdsGetDeviceInfo(camera, &deviceInfo);
						if (err == EDS_ERR_OK && camera == NULL)
						{
							std::cout << "Camera is not found." << std::endl;
							pause_return();
							exit(EXIT_FAILURE);
						}
						std::cout << "[" << i + 1 << "]\t" << deviceInfo.szDeviceDescription << std::endl;
					}
				}
				std::cout << "--------------------------------" << std::endl;
				std::cout << "Enter the number of the camera to connect [1-" << count << "]" << std::endl;
				std::cout << "\tor 'a' (=All)" << std::endl;
				std::cout << "\tor 'x' (=eXit)" << std::endl;
				std::cout << "> ";

				// Wait for input on another thread to send the command "getevent" periodically.
//				_beginthread(CheckKey, 0, NULL); // Start waiting for keystrokes
				th = std::make_shared<std::thread>(CheckKey);

				loop = false;
				keyflag = false; // initialize keyflag
			}
			if (keyflag == true) // Within this scope, the CheckKey thread is terminated
			{
				th->join();
				// min～max
				if (std::regex_match(control_number, match_results, std::regex(R"((\d+)-(\d+))")))
				{

					min = stoi(match_results[1].str());
					max = stoi(match_results[2].str());
					std::cout << "connecting to [" << min << "-" << max << "] ..." << std::endl;
					if (min > count || max > count)
					{
						std::cout << "invalid number" << std::endl;
						pause_return();
						exit(EXIT_FAILURE);
					}
					for (i = (min - 1); i < max; i++)
					{
						err = EdsGetChildAtIndex(cameraList, i, &camera);
						cameraArray.push_back(camera);
						bodyID.push_back(i + 1);
					}
					break;
				}

				// single camera
				else if (std::regex_search(control_number, match_results, std::regex("[0-9]")))
				{
					i = stoi(control_number) - 1;
					if (i > count-1)
					{
						std::cout << "invalid number" << std::endl;
						pause_return();
						exit(EXIT_FAILURE);
					}
					std::cout << "connecting to [" << control_number << "] ..." << std::endl;
					err = EdsGetChildAtIndex(cameraList, i, &camera);
					cameraArray.push_back(camera);
					bodyID.push_back(i + 1);
					break;
				}

				// all
				else if (std::regex_search(control_number, match_results, std::regex("a", std::regex_constants::icase)))
				{
					std::cout << "connecting to all cameras..." << std::endl;
					for (i = 0; i < count; i++)
					{
						err = EdsGetChildAtIndex(cameraList, i, &camera);
						cameraArray.push_back(camera);
						bodyID.push_back(i + 1);
					}
					break;
				}

				// exit
				else if (std::regex_search(control_number, match_results, std::regex("x", std::regex_constants::icase)))
				{
					std::cout << "closing app..." << std::endl;
					pause_return();
					exit(EXIT_SUCCESS);
				}

				// retry
				else
				{
					keyflag = false;
					loop = true;
					pause_return();
					clr_screen();

					continue;
				}
			}
			// send getevent periodically
			EdsGetEvent();
			std::this_thread::sleep_for(200ms);
		}

		PreSetting(cameraArray, bodyID);

		clr_screen();

		loop = true;

		while (true)
		{
			//control menu
			if (loop)
			{
				keyflag = false; // initialize keyflag
//				_beginthread(CheckKey, 0, NULL); // Start waiting for keystrokes
				th = std::make_shared<std::thread>(CheckKey);

				std::cout << "--------------------------------" << std::endl;
				std::cout
					<< "[ 1] Take Picture and download \n"
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
					<< "[14] Turntable Control \n" << std::endl;
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
						TakePicture(cameraArray, bodyID);
						clr_screen();
						loop = true;
						continue;
					}
					// Press halfway
					else if (control_number == "2")
					{
						PressShutter(cameraArray, bodyID, kEdsCameraCommand_ShutterButton_Halfway);
						clr_screen();
						loop = true;
						continue;
					}
					// Press Completely
					else if (control_number == "3")
					{
						PressShutter(cameraArray, bodyID, kEdsCameraCommand_ShutterButton_Completely);
						clr_screen();
						loop = true;
						continue;
					}
					// Press Completely
					else if (control_number == "4")
					{
						PressShutter(cameraArray, bodyID, kEdsCameraCommand_ShutterButton_OFF);
						clr_screen();
						loop = true;
						continue;
					}
					// TV
					else if (control_number == "5")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_Tv, tv_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_Tv, tv_table);

						std::cout << "input no. (ex. 54 = 1/250)" << std::endl;
						std::cout << ">";

						data = getvalue();
						if (data != -1)
						{
							SetProperty(cameraArray, bodyID, kEdsPropID_Tv, data, tv_table);
							pause_return();
						}
						clr_screen();
						loop = true;
						continue;
					}
					// Av
					else if (control_number == "6")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_Av, av_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_Av, av_table);
						std::cout << "input Av (ex. 21 = 5.6)" << std::endl;
						std::cout << ">";

						data = getvalue();
						if (data != -1)
						{
							SetProperty(cameraArray, bodyID, kEdsPropID_Av, data, av_table);
							pause_return();
						}
						clr_screen();
						loop = true;
						continue;
					}
					// ISO
					else if (control_number == "7")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_ISOSpeed, iso_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_ISOSpeed, iso_table);
						std::cout << "input ISOSpeed (ex. 8 = ISO 200)" << std::endl;
						std::cout << ">";

						data = getvalue();
						if (data != -1)
						{
							SetProperty(cameraArray, bodyID, kEdsPropID_ISOSpeed, data, iso_table);
							pause_return();
						}
						clr_screen();
						loop = true;
						continue;
					}
					// White Balanse
					else if (control_number == "8")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_WhiteBalance, whitebalance_table);
						std::cout << "input WhiteBalance (ex. 0 = Auto)" << std::endl;
						std::cout << ">";

						data = getvalue();
						if (data != -1)
						{
							SetProperty(cameraArray, bodyID, kEdsPropID_WhiteBalance, data, whitebalance_table);
							pause_return();
						}
						clr_screen();
						loop = true;
						continue;
					}
					// Drive Mode
					else if (control_number == "9")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_DriveMode, drivemode_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_DriveMode, drivemode_table);
						std::cout << "input Drive Mode (ex. 0 = Single shooting)" << std::endl;
						std::cout << ">";

						data = getvalue();
						if (data != -1)
						{
							SetProperty(cameraArray, bodyID, kEdsPropID_DriveMode, data, drivemode_table);
							pause_return();
						}
						clr_screen();
						loop = true;
						continue;
					}
					// AE Mode
					else if (control_number == "10")
					{
						GetProperty(cameraArray, bodyID, kEdsPropID_AEMode, AEmode_table);
						GetPropertyDesc(cameraArray, bodyID, kEdsPropID_AEMode, AEmode_table);
						pause_return();
						clr_screen();
						loop = true;
						continue;
					}
					// Download EVF image
					else if (control_number == "11")
					{
						StartEvfCommand(cameraArray, bodyID);
						DownloadEvfCommand(cameraArray, bodyID);
						EndEvfCommand(cameraArray, bodyID);
						pause_return();
						clr_screen();
						loop = true;
						continue;
					}
					// Download All Image
					else if (control_number == "12")
					{
						DownloadImageAll(cameraArray, bodyID);
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
						int degree_tracker = 0;
						std::cout << "Enter degrees per move: ";
						std::cin >> degree_inc;
						std::cout << "Enter number of moves: ";
						std::cin >> num_moves;
						PressShutter(cameraArray, bodyID, kEdsCameraCommand_ShutterButton_Halfway);
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
								//Sleep(4000);
							} 
							TakePicture(cameraArray, bodyID);
							//DownloadImageAll(cameraArray, bodyID);
							EdsGetEvent();
							rshandle.turntable_position = degree_tracker;
							degree_tracker += stoi(degree_inc);
							rshandle.get_current_frame(10000);
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
					clr_screen();
					keyflag = false;
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
			EdsGetEvent();
			std::this_thread::sleep_for(200ms);
		}

		//Release camera list
		if (cameraList != NULL)
		{
			EdsRelease(cameraList);
		}

		//Release Camera

		for (i = 0; i < cameraArray.size(); i++)
		{
			if (cameraArray[i] != NULL)
			{
				EdsRelease(cameraArray[i]);
				cameraArray[i] = NULL;
			}
		}
		//Remove elements before looping. Memory is automatically freed at the destructor of the vector when it leaves the scope.
		cameraArray.clear();
		bodyID.clear();
		clr_screen();
	}

	//Termination of SDK
	if (isSDKLoaded)
	{
		EdsTerminateSDK();
	}

	return FALSE;
}