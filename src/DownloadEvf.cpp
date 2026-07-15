/*
11/7 GS:
	this file is meant to enable live viewing of the DSLR cameras
	it uses filestreaming with opencv to give almost fully realtime rendering

	in the past couple weeks, i have been porting this entire codebase to linux. for this file, the only thing
		needed was a waitKey() call in liveview loop which causes the OS to query the buffer where the images
		are waiting everytime the loop runs INSTEAD of not this. the windows version automatically does this
		consistent buffer check under the hood which is why it works there.

	the opencv window would sometimes get stuck open for one of the cameras. the fix i did was just to reorder
		the closures so the stream ends THEN the window attempts closing AND i added an extra waitKey(1) 
		afterwards to attempt a final read to remove the empty buffer. otherwise the state of the dslr cams
		gets messed up and breaks the next time live view is triggered

	also useful:
		this file creates folders with the camera names inside wherever ./MultiCamCui (build) occurs. this 
		is the location of the file stream

*/
/*

5/8/26 PG:
	Apparently EVF stands for Electronic View Finder, it's Canon's terminology for the live view feed.
	The more you know.

*/



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
#include "DebugUtils.h" 
#include "ConfigHandler.h"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

extern bool liveview_active;

typedef struct _EVF_DATASET
{
	EdsStreamRef	stream; // JPEG stream.
	EdsUInt32		zoom;
	EdsRect			zoomRect;
	EdsPoint		imagePosition;
	EdsUInt32		histogram[256 * 4]; //(YRGB) YRGBYRGBYRGBYRGB....
	EdsSize			sizeJpegLarge;
}EVF_DATASET;

// Function to throw camera exceptions based on the error code
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

// Function to release the stream and image references
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

// Function to start the EVF command, setting the EVF mode and output device
EdsError StartEvfCommand(EdsCameraRef const& camera, EdsUInt64 const& bodyID) {
    const int    MAX_RETRIES  = 5;
    const auto   RETRY_DELAY  = 1000ms;

    EdsError     err          = EDS_ERR_OK;
    EdsUInt32    evfMode      = 0;

    // Retry loop — camera may not be ready immediately on first connect
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        err = EdsGetPropertyData(camera, kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);

        if (err == EDS_ERR_OBJECT_NOTREADY) {
            std::cout << "[StartEvf] Camera not ready, attempt " << attempt
                      << "/" << MAX_RETRIES << ", retrying in "
                      << RETRY_DELAY.count() << "ms..." << std::endl;
            std::this_thread::sleep_for(RETRY_DELAY);
            continue;
        }

        if (err != EDS_ERR_OK) {
            throwCameraException(err, "StartEvfCommand: EdsGetPropertyData failed");
        }

        // Camera responded — proceed with EVF setup
        if (evfMode == 0) {
            evfMode = 1;
            err = EdsSetPropertyData(camera, kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);
            if (err != EDS_ERR_OK) {
                throwCameraException(err, "StartEvfCommand: failed to set EVF mode");
            }
        }

        EdsUInt32 device = 0;
        err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
        if (err != EDS_ERR_OK) {
            throwCameraException(err, "StartEvfCommand: failed to get output device");
        }

        device |= kEdsEvfOutputDevice_PC;
        err = EdsSetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
        if (err != EDS_ERR_OK) {
            throwCameraException(err, "StartEvfCommand: failed to set output device");
        }

        std::cout << "[StartEvf] Camera ready after attempt " << attempt << std::endl;
        return EDS_ERR_OK;
    }

    // Exhausted retries
    throwCameraException(EDS_ERR_OBJECT_NOTREADY,
        "StartEvfCommand: camera not ready after max retries");
    return EDS_ERR_OBJECT_NOTREADY;  // unreachable, but satisfies the compiler
}


// Function to download the EVF image from the camera and display it using OpenCV
EdsError DownloadEvfCommand(EdsCameraRef const& camera, std::string const& cameraName, std::thread::id const& parentThreadID)
{
    std::string cam_name_short = fs::path(cameraName).filename().string();
	const std::string TAG = "[DownloadEvf] (" + cam_name_short + ") ";
    DebugUtils::logDebug(TAG + "Thread started.");

    EdsError      err      = EDS_ERR_OK;
    EdsEvfImageRef evfImage = NULL;
    EdsStreamRef   stream   = NULL;
    EdsUInt32      device   = 0;

	std::string sdk_path  = cameraName + "/evf.jpg";         // SDK writes here
	std::string temp_path = cameraName + "/evf_tmp.jpg";     // our intermediate copy
	std::string live_path = cameraName + "/evf_live.jpg";    // Safe readable version for Python script
	std::string lock_path = cameraName + "/evf.lock";        // write sentinel

	ConfigHandler& config = ConfigHandler::getInstance();	// MOAD CONFIG

    // ── Check output device ───────────────────────────────────────────────
    err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
    if (err != EDS_ERR_OK) {
        DebugUtils::logError(TAG + "Failed to get output device. err=" + std::to_string(err));
        throwCameraException(err, (TAG + "EdsGetPropertyData failed").c_str());
    }
    if ((device & kEdsEvfOutputDevice_PC) == 0) {
        DebugUtils::logWarning(TAG + "PC is not the output device — exiting early.");
        return EDS_ERR_OK;
    }
    DebugUtils::logDebug(TAG + "Output device confirmed as PC.");

    // ── Create directory ──────────────────────────────────────────────────
    if (!fs::exists(cameraName)) {
        DebugUtils::logDebug(TAG + "Creating directory: " + cameraName);
        fs::create_directories(cameraName);
    }

    // std::string filepath = cameraName + "/evf.jpg";
    DebugUtils::logDebug(TAG + "File stream path: " + sdk_path);

    // ── Create file stream ────────────────────────────────────────────────
    err = EdsCreateFileStream(sdk_path.c_str(), kEdsFileCreateDisposition_CreateAlways,
                              kEdsAccess_ReadWrite, &stream);
    if (err != EDS_ERR_OK) {
        DebugUtils::logError(TAG + "EdsCreateFileStream failed. err=" + std::to_string(err));
        throwCameraException(err, (TAG + "EdsCreateFileStream failed").c_str());
    }
    DebugUtils::logDebug(TAG + "File stream created.");

    // ── Create EvfImageRef ────────────────────────────────────────────────
    err = EdsCreateEvfImageRef(stream, &evfImage);
    if (err != EDS_ERR_OK) {
        DebugUtils::logError(TAG + "EdsCreateEvfImageRef failed. err=" + std::to_string(err));
        ReleaseStream(stream, evfImage);
        throwCameraException(err, (TAG + "EdsCreateEvfImageRef failed").c_str());
    }
    DebugUtils::logDebug(TAG + "EvfImageRef created. Entering liveview loop.");

    std::this_thread::sleep_for(100ms);

    // ── Liveview loop ─────────────────────────────────────────────────────
    int frame_count = 0;
    while (liveview_active) {

        // Download with retry
        const int  MAX_DL_RETRIES = 5;
        const auto DL_RETRY_DELAY = 300ms;
        for (int attempt = 1; attempt <= MAX_DL_RETRIES; attempt++) {
            err = EdsDownloadEvfImage(camera, evfImage);
            if (err == EDS_ERR_OK) {
                break;
            } else if (err == EDS_ERR_OBJECT_NOTREADY) {
                DebugUtils::logWarning(TAG + "Image not ready, attempt "
                    + std::to_string(attempt) + "/" + std::to_string(MAX_DL_RETRIES)
                    + ", retrying in " + std::to_string(DL_RETRY_DELAY.count()) + "ms...");
                DebugUtils::logTimestamp(TAG + "Not ready.");
                
                std::this_thread::sleep_for(DL_RETRY_DELAY);
            } else {
                // Any other error is not worth retrying
                DebugUtils::logError(TAG + "EdsDownloadEvfImage failed with unexpected err="
                    + std::to_string(err) + " on attempt " + std::to_string(attempt));
                break;
            }
        }

        if (err != EDS_ERR_OK) {
            DebugUtils::logError(TAG + "EdsDownloadEvfImage failed after all retries. err="
                + std::to_string(err) + " — exiting liveview loop.");
            DebugUtils::logTimestamp("Livestream error.");
            ReleaseStream(stream, evfImage);
            throwCameraException(err, (TAG + "EdsDownloadEvfImage failed").c_str());
        }

        std::this_thread::sleep_for(50ms);

        // ── Metadata ──────────────────────────────────────────────────────
        EVF_DATASET dataSet = { 0 };
        dataSet.stream = stream;
        EdsGetPropertyData(evfImage, kEdsPropID_Evf_Zoom, 0, sizeof(dataSet.zoom), &dataSet.zoom);
        EdsGetPropertyData(evfImage, kEdsPropID_Evf_ImagePosition, 0, sizeof(dataSet.imagePosition), &dataSet.imagePosition);
        EdsGetPropertyData(evfImage, kEdsPropID_Evf_Histogram, 0, sizeof(dataSet.histogram), dataSet.histogram);
        EdsGetPropertyData(evfImage, kEdsPropID_Evf_ZoomRect, 0, sizeof(dataSet.zoomRect), &dataSet.zoomRect);
        EdsGetPropertyData(evfImage, kEdsPropID_Evf_CoordinateSystem, 0, sizeof(dataSet.sizeJpegLarge), &dataSet.sizeJpegLarge);
		
		// ── Safe file publish ─────────────────────────────────────────────
        // SDK has written to sdk_path. Copy → temp, then atomic rename → live.
        // Lock file signals to Python that a write is in progress.
        {
            // 1. Acquire lock
            std::ofstream lock(lock_path);
        }

        // 2. Copy SDK output to temp path
        //    (can't rename sdk_path itself — the SDK still holds it open)
        try {
            fs::copy_file(sdk_path, temp_path, fs::copy_options::overwrite_existing);
        } catch (const fs::filesystem_error& e) {
            DebugUtils::logWarning(TAG + "copy_file failed: " + e.what());
            fs::remove(lock_path);
            continue;
        }

        // 3. Atomic rename temp → live
        fs::rename(temp_path, live_path);

        // 4. Release lock
        fs::remove(lock_path);

        // ── Display ───────────────────────────────────────────────────────
		if(config.getValue<bool>("dslr.display_liveview_windows")) {
			cv::Mat frame = cv::imread(live_path);
			if (frame.empty()) {
				DebugUtils::logWarning(TAG + "cv::imread returned empty frame — skipping display.");
				continue;
			}

			if (frame_count == 0) {
				DebugUtils::logDebug(TAG + "First frame displayed successfully.");
			}
			frame_count++;

			cv::namedWindow(cam_name_short, cv::WINDOW_NORMAL);
			cv::setWindowProperty(cam_name_short, cv::WND_PROP_TOPMOST, 1);
			cv::resizeWindow(cam_name_short, 600, 400);
			cv::imshow(cam_name_short, frame);

			int i = cameraName[cameraName.size() - 1] - '0' - 1;
			cv::moveWindow(cam_name_short, 600 * (i % 3), 430 * (i / 3));

			if (cv::waitKey(1) == 27)
				break;
		}      
    }

    DebugUtils::logDebug(TAG + "Liveview loop exited. Frames displayed: "
        + std::to_string(frame_count) + ". Releasing resources.");

    // ── Cleanup ───────────────────────────────────────────────────────────
    ReleaseStream(stream, evfImage);
    DebugUtils::logDebug(TAG + "Stream released.");
	
	if(config.getValue<bool>("dslr.display_liveview_windows")) {
		cv::destroyWindow(cam_name_short);
		DebugUtils::logDebug(TAG + "CV Window Destroyed.");
		cv::waitKey(1);
	}

    DebugUtils::logDebug(TAG + "Thread exiting cleanly.");
    return EDS_ERR_OK;
}


// Function to end the EVF command, stopping the live view and releasing resources
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

// Function to start the EVF command for multiple cameras
EdsError StartEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
    for (size_t i = 0; i < cameraArray.size(); i++)
    {
        EdsError err = StartEvfCommand(cameraArray[i], bodyID[i]);
        if (err != EDS_ERR_OK) {
            DebugUtils::logError("[StartEvf] Failed on camera index " + std::to_string(i)
                + " err=" + std::to_string(err));
            return err;
        }
        DebugUtils::logDebug("[StartEvf] Camera index " + std::to_string(i) + " configured successfully.");
    }
    return EDS_ERR_OK;
}

// (DEPRICATED) Function to download the image of each camera and then display it using OpenCV.
// EdsError DownloadEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& _bodyID)
// {

// 	EdsError err = EDS_ERR_OK;

// 	std::vector<EdsEvfImageRef> evfImage(5, NULL);
// 	std::vector<EdsStreamRef> stream(5, NULL);
// 	EdsUInt32 device = 0;
// 	int i;

// 	for (i = 0; i < cameraArray.size(); i++)
// 	{
// 		err = EdsGetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
// 		// Exit unless during live view.
// 		if ((device & kEdsEvfOutputDevice_PC) == 0)
// 		{	
// 			return true;
// 		}

// 		// create folder  ex) cam1
// 		EdsUInt32 camid;
// 		camid = (EdsUInt32)_bodyID[i];
// 		std::string directory_tree = "cam" + std::to_string(camid);
// 		if (fs::exists(directory_tree) == FALSE)
// 		{
// 			std::filesystem::create_directories(directory_tree);
// 		}

// 		std::string tmp;
// 		tmp = directory_tree + "/evf.jpg";
// 		char* filename = new char[tmp.size() + 1];
// 		strcpy(filename, tmp.c_str());

// 		// When creating to a file.
// 		err = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream[i]);

// 		// Create EvfImageRef.
// 		if (err == EDS_ERR_OK)
// 		{
// 			err = EdsCreateEvfImageRef(stream[i], &evfImage[i]);
// 		}

// 		//Notification of error
// 		if (err != EDS_ERR_OK)
// 		{
// 			std::cout << "Error: " << err << std::endl;
// 			ReleaseStream(stream[i], evfImage[i]);
// 			std::cout << "Fifth" << std::endl;
// 			throwCameraException(err);
// 		}
// 	}

// 	while (cv::waitKey(1) != 'r') {
// 		for (i = 0; i < cameraArray.size(); i++) {
// 			EdsUInt32 camid;
// 			camid = (EdsUInt32)_bodyID[i];
// 			std::string directory_tree = "cam" + std::to_string(camid);
// 			// Download live view image data.
// 			if (err == EDS_ERR_OK)
// 			{
// 				err = EdsDownloadEvfImage(cameraArray[i], evfImage[i]);
// 				std::this_thread::sleep_for(50ms);
// 			}
	
// 			// Get meta data for live view image data.
// 			if (err == EDS_ERR_OK)
// 			{
// 				EVF_DATASET dataSet = { 0 };
	
// 				dataSet.stream = stream[i];
	
// 				// Get magnification ratio (x1, x5, or x10).
// 				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_Zoom, 0, sizeof(dataSet.zoom), &dataSet.zoom);
	
// 				// Get position of image data. (when enlarging)
// 				// Upper left coordinate using JPEG Large size as a reference.
// 				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_ImagePosition, 0, sizeof(dataSet.imagePosition), &dataSet.imagePosition);
	
// 				// Get histogram (RGBY).
// 				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_Histogram, 0, sizeof(dataSet.histogram), dataSet.histogram);
	
// 				// Get rectangle of the focus border.
// 				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_ZoomRect, 0, sizeof(dataSet.zoomRect), &dataSet.zoomRect);
	
// 				// Get the size as a reference of the coordinates of rectangle of the focus border.
// 				EdsGetPropertyData(evfImage[i], kEdsPropID_Evf_CoordinateSystem, 0, sizeof(dataSet.sizeJpegLarge), &dataSet.sizeJpegLarge);
// 			}

// 			//Notification of error
// 			if (err != EDS_ERR_OK)
// 			{
// 				std::cout << "Error: " << err << std::endl;
// 				ReleaseStream(stream[i], evfImage[i]);
// 				std::cout << "Sixth" << std::endl;
// 				throwCameraException(err, "Something 1");
// 			}

// 			// Display Image
// 			cv::Mat frame;
// 			frame = cv::imread(directory_tree + "/evf.jpg");
// 			if (frame.empty()) {
// 				break;
// 			}
		
// 			cv::namedWindow(directory_tree, cv::WINDOW_NORMAL);
// 			cv::setWindowProperty(directory_tree, cv::WND_PROP_TOPMOST, 1);
// 			cv::resizeWindow(directory_tree, 600, 400);
// 			cv::imshow(directory_tree, frame);
// 			cv::moveWindow(directory_tree, 600 * (i % 3), 430 * (i / 3));
// 		}
// 	}

// 	cv::waitKey(0);
// 	cv::destroyAllWindows();

// 	std::cout << "End" << std::endl;
// 	for (i = 0; i < cameraArray.size(); i++) {
// 		ReleaseStream(stream[i], evfImage[i]);
// 	}

// 	return true;
// }

// Function to end the EVF command for multiple cameras
EdsError EndEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	bool result;
	for (size_t i = 0; i < cameraArray.size(); i++) {
		result = EndEvfCommand(cameraArray[i], bodyID[i]);
		if (!result) {
			return false;
		}
	}
	return true;
}
