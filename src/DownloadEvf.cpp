#include <thread>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "CameraException.h"

namespace fs = std::filesystem;
using namespace std::chrono_literals;


typedef struct _EVF_DATASET
{
	EdsStreamRef	stream; // JPEG stream.
	EdsUInt32		zoom;
	EdsRect			zoomRect;
	EdsPoint		imagePosition;
	EdsUInt32		histogram[256 * 4]; //(YRGB) YRGBYRGBYRGBYRGB....
	EdsSize			sizeJpegLarge;
}EVF_DATASET;

void throwCameraException(EdsError err, const char* message = "") {
	switch (err)
	{
		case EDS_ERR_DEVICE_BUSY:
			throw CameraBusyException(message);

		case EDS_ERR_OBJECT_NOTREADY:
			throw CameraObjectNotReadyException(message);
		
		default:
			throw CameraException(message);
	}
}

void ReleaseStream(EdsStreamRef& stream, EdsEvfImageRef& image) {
	if (stream != NULL)
	{
		EdsRelease(stream);
		stream = NULL;
	}

	if (image != NULL)
	{
		EdsRelease(image);
		image = NULL;
	}
}

EdsError StartEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& bodyID) {
	EdsError err = EDS_ERR_OK;
	EdsUInt32 evfMode = 0;
	
	// Get EVF Mode Properties 
	err = EdsGetPropertyData(camera, kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);

	// Set EVF Mode Property to 1 if is 0
	if (evfMode == 0) {
		evfMode = 1;
		err = EdsSetPropertyData(camera, kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);
	}

	if (err == EDS_ERR_OK)
	{
		// Get the current output device.
		EdsUInt32 device = 0;
		err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);

		// Set the PC as the current output device.
		device |= kEdsEvfOutputDevice_PC;

		// Set the output device
		if (err == EDS_ERR_OK)
		{
			err = EdsSetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
		}
	}

	//Notification of error
	if (err != EDS_ERR_OK) {
		std::cout << "First" << std::endl;
		throwCameraException(err);
	}

	return true;
}

EdsError DownloadEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& _bodyID)
{
	EdsError err = EDS_ERR_OK;

	EdsEvfImageRef evfImage = NULL;
	EdsStreamRef stream = NULL;
	EdsUInt32 device = 0;
	EdsUInt32 retry = 0;

	// Get the device property
	err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
	// Exit unless during live view.
	if ((device & kEdsEvfOutputDevice_PC) == 0) {	
		return true;
	}

	// create folder  ex) cam1
	// TODO: This part should be deleted 
	EdsUInt32 camid;
	camid = (EdsUInt32)_bodyID;
	std::string directory_tree = "cam" + std::to_string(camid);
	if (fs::exists(directory_tree) == FALSE) {
		std::filesystem::create_directories(directory_tree);
	}

	std::string tmp;
	tmp = directory_tree + "\\evf.jpg";
	char* filename = new char[tmp.size() + 1];
	strcpy(filename, tmp.c_str());

	// When creating to a file.
	err = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);

	// Create EvfImageRef.
	if (err == EDS_ERR_OK) {
		err = EdsCreateEvfImageRef(stream, &evfImage);
	}

	//Notification of error
	if (err != EDS_ERR_OK) {
		std::cout << "Second" << std::endl;
		throwCameraException(err);
	}

	if (retry >= 3) {
		ReleaseStream(stream, evfImage);
		throw std::runtime_error("The camera is not ready. Try again");
	}

	std::this_thread::sleep_for(100ms);

	// Show Liveview Image
	while (cv::waitKey(1) != 'r') {
		EdsUInt32 camid;
		camid = (EdsUInt32)_bodyID;
		std::string directory_tree = "cam" + std::to_string(camid);
		// Download live view image data.
		// TODO: This should be converted into cv::Mat instead
		if (err == EDS_ERR_OK)
		{
			err = EdsDownloadEvfImage(camera, evfImage);
			std::this_thread::sleep_for(50ms);
		}

		// Get meta data for live view image data.
		if (err == EDS_ERR_OK)
		{
			EVF_DATASET dataSet = { 0 };
			dataSet.stream = stream;

			// Get magnification ratio (x1, x5, or x10).
			EdsGetPropertyData(evfImage, kEdsPropID_Evf_Zoom, 0, sizeof(dataSet.zoom), &dataSet.zoom);

			// Get position of image data. (when enlarging)
			// Upper left coordinate using JPEG Large size as a reference.
			EdsGetPropertyData(evfImage, kEdsPropID_Evf_ImagePosition, 0, sizeof(dataSet.imagePosition), &dataSet.imagePosition);

			// Get histogram (RGBY).
			EdsGetPropertyData(evfImage, kEdsPropID_Evf_Histogram, 0, sizeof(dataSet.histogram), dataSet.histogram);

			// Get rectangle of the focus border.
			EdsGetPropertyData(evfImage, kEdsPropID_Evf_ZoomRect, 0, sizeof(dataSet.zoomRect), &dataSet.zoomRect);

			// Get the size as a reference of the coordinates of rectangle of the focus border.
			EdsGetPropertyData(evfImage, kEdsPropID_Evf_CoordinateSystem, 0, sizeof(dataSet.sizeJpegLarge), &dataSet.sizeJpegLarge);
		}

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			ReleaseStream(stream, evfImage);
			std::cout << "Third" << std::endl;
			throwCameraException(err);
		}
		// Display Image
		cv::Mat frame;
		frame = cv::imread(directory_tree + "\\evf.jpg");
		if (frame.empty()) {
			break;
		}
	
		cv::namedWindow(directory_tree, cv::WINDOW_NORMAL);
		cv::setWindowProperty(directory_tree, cv::WND_PROP_TOPMOST, 1);
		cv::resizeWindow(directory_tree, 600, 400);
		cv::imshow(directory_tree, frame);
		// cv::moveWindow(directory_tree, 600 * (i % 3), 430 * (i / 3));
	}

	cv::waitKey(0);
	cv::destroyAllWindows();

	// Close the Liveview Stream
	ReleaseStream(stream, evfImage);

	return true;
}

EdsError EndEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& bodyID)
{
	EdsError err = EDS_ERR_OK;
	// Get the current output device.
	EdsUInt32 device = 0;
	err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);

		// Do nothing if the remote live view has already ended.
		if ((device & kEdsEvfOutputDevice_PC) == 0)
		{
			return true;
		}

		// Get depth of field status.
		EdsUInt32 depthOfFieldPreview = 0;
		err = EdsGetPropertyData(camera, kEdsPropID_Evf_DepthOfFieldPreview, 0, sizeof(depthOfFieldPreview), &depthOfFieldPreview);

		// Release depth of field in case of depth of field status.
		if (depthOfFieldPreview != 0)
		{
			depthOfFieldPreview = 0;
			err = EdsSetPropertyData(camera, kEdsPropID_Evf_DepthOfFieldPreview, 0, sizeof(depthOfFieldPreview), &depthOfFieldPreview);

			// Standby because commands are not accepted for awhile when the depth of field has been released.
			if (err == EDS_ERR_OK)
			{
				std::this_thread::sleep_for(500ms);
			}
		}


		// Change the output device.
		if (err == EDS_ERR_OK)
		{
			device &= ~kEdsEvfOutputDevice_PC;
			err = EdsSetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
		}


		EdsUInt32 evfMode = 0;
		err = EdsGetPropertyData(camera,
			kEdsPropID_Evf_Mode,
			0,
			sizeof(evfMode),
			&evfMode);


		if (evfMode == 1)
		{
			evfMode = 0;

			// Set to the camera.
			err = EdsSetPropertyData(camera, kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);
		}

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			std::cout << "Fourth" << std::endl;
			throwCameraException(err);
		}
	return true;
}

EdsError StartEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	int i;
	bool result;
	for (i = 0; i < cameraArray.size(); i++)
	{
		result = StartEvfCommand(cameraArray[i], bodyID[i]);
		if (!result) {
			return result;
		}
	}
	return true;
}

EdsError DownloadEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& _bodyID)
{

	EdsError err = EDS_ERR_OK;

	std::vector<EdsEvfImageRef> evfImage(5, NULL);
	std::vector<EdsStreamRef> stream(5, NULL);
	EdsUInt32 device = 0;
	int i;

	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsGetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
		// Exit unless during live view.
		if ((device & kEdsEvfOutputDevice_PC) == 0)
		{	
			return true;
		}

		// create folder  ex) cam1
		EdsUInt32 camid;
		camid = (EdsUInt32)_bodyID[i];
		std::string directory_tree = "cam" + std::to_string(camid);
		if (fs::exists(directory_tree) == FALSE)
		{
			std::filesystem::create_directories(directory_tree);
		}

		std::string tmp;
		tmp = directory_tree + "\\evf.jpg";
		char* filename = new char[tmp.size() + 1];
		strcpy(filename, tmp.c_str());

		// When creating to a file.
		err = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream[i]);

		// Create EvfImageRef.
		if (err == EDS_ERR_OK)
		{
			err = EdsCreateEvfImageRef(stream[i], &evfImage[i]);
		}

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			std::cout << "Error: " << err << std::endl;
			ReleaseStream(stream[i], evfImage[i]);
			std::cout << "Fifth" << std::endl;
			throwCameraException(err);
		}
	}

	while (cv::waitKey(1) != 'r') {
		for (i = 0; i < cameraArray.size(); i++) {
			EdsUInt32 camid;
			camid = (EdsUInt32)_bodyID[i];
			std::string directory_tree = "cam" + std::to_string(camid);
			// Download live view image data.
			if (err == EDS_ERR_OK)
			{
				err = EdsDownloadEvfImage(cameraArray[i], evfImage[i]);
				std::this_thread::sleep_for(50ms);
			}
	
			// Get meta data for live view image data.
			if (err == EDS_ERR_OK)
			{
				EVF_DATASET dataSet = { 0 };
	
				dataSet.stream = stream[i];
	
				// Get magnification ratio (x1, x5, or x10).
				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_Zoom, 0, sizeof(dataSet.zoom), &dataSet.zoom);
	
				// Get position of image data. (when enlarging)
				// Upper left coordinate using JPEG Large size as a reference.
				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_ImagePosition, 0, sizeof(dataSet.imagePosition), &dataSet.imagePosition);
	
				// Get histogram (RGBY).
				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_Histogram, 0, sizeof(dataSet.histogram), dataSet.histogram);
	
				// Get rectangle of the focus border.
				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_ZoomRect, 0, sizeof(dataSet.zoomRect), &dataSet.zoomRect);
	
				// Get the size as a reference of the coordinates of rectangle of the focus border.
				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_CoordinateSystem, 0, sizeof(dataSet.sizeJpegLarge), &dataSet.sizeJpegLarge);
			}

			//Notification of error
			if (err != EDS_ERR_OK)
			{
				std::cout << "Error: " << err << std::endl;
				ReleaseStream(stream[i], evfImage[i]);
				std::cout << "Sixth" << std::endl;
				throwCameraException(err, "Something 1");
			}

			// Display Image
			cv::Mat frame;
			frame = cv::imread(directory_tree + "\\evf.jpg");
			if (frame.empty()) {
				break;
			}
		
			cv::namedWindow(directory_tree, cv::WINDOW_NORMAL);
			cv::setWindowProperty(directory_tree, cv::WND_PROP_TOPMOST, 1);
			cv::resizeWindow(directory_tree, 600, 400);
			cv::imshow(directory_tree, frame);
			cv::moveWindow(directory_tree, 600 * (i % 3), 430 * (i / 3));
		}
	}

	cv::waitKey(0);
	cv::destroyAllWindows();

	std::cout << "End" << std::endl;
	for (i = 0; i < cameraArray.size(); i++) {
		ReleaseStream(stream[i], evfImage[i]);
	}

	return true;
}

EdsError EndEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	bool result;
	for (int i = 0; i < cameraArray.size(); i++) {
		result = EndEvfCommand(cameraArray[i], bodyID[i]);
		if (!result) {
			return false;
		}
	}
	return true;
}
