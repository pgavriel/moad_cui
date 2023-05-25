#include <thread>
#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <filesystem>
#include "EDSDK.h"
#include "EDSDKTypes.h"

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

EdsError StartEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	EdsError	 err = EDS_ERR_OK;
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{

		/// Change settings because live view cannot be started
		/// when camera settings are set to ?gdo not perform live view.?h
		EdsUInt32 evfMode = 0;

		//Acquisition of the property
		err = EdsGetPropertyData(cameraArray[i],
			kEdsPropID_Evf_Mode,
			0,
			sizeof(evfMode),
			&evfMode);


		if (evfMode == 0)
		{
			evfMode = 1;

			// Set to the camera.
			err = EdsSetPropertyData(cameraArray[i], kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);
		}

		if (err == EDS_ERR_OK)
		{
			// Get the current output device.
			EdsUInt32 device = 0;
			err = EdsGetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);

			// Set the PC as the current output device.
			device |= kEdsEvfOutputDevice_PC;

			// Set to the camera.
			if (err == EDS_ERR_OK)
			{
				err = EdsSetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
			}
		}

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			// It doesn't retry it at device busy
			if (err == EDS_ERR_DEVICE_BUSY)
			{
				std::cout << "DeviceBusy" << std::endl;
			}
			return false;
		}
	}
	return true;
}

EdsError DownloadEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& _bodyID)
{

	EdsError err = EDS_ERR_OK;

	EdsEvfImageRef evfImage = NULL;
	EdsStreamRef stream = NULL;
	EdsUInt32 device = 0;
	EdsUInt32 retry = 0;

	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		for (retry = 0; retry < 3; retry++)
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
			err = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);


			// Create EvfImageRef.
			if (err == EDS_ERR_OK)
			{
				err = EdsCreateEvfImageRef(stream, &evfImage);
			}

			// Download live view image data.
			if (err == EDS_ERR_OK)
			{
				err = EdsDownloadEvfImage(cameraArray[i], evfImage);
				std::this_thread::sleep_for(200ms);
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

				/*
							// Live view image transfer complete notification.
							if (err == EDS_ERR_OK)
							{
								CameraEvent e("EvfDataChanged", &dataSet);
								_model->notifyObservers(&e);
							}
				*/
			}


			if (stream != NULL)
			{
				EdsRelease(stream);
				stream = NULL;
			}

			if (evfImage != NULL)
			{
				EdsRelease(evfImage);
				evfImage = NULL;
			}

			//Notification of error
			if (err != EDS_ERR_OK)
			{
				// Retry getting image data if EDS_ERR_OBJECT_NOTREADY is returned
				// when the image data is not ready yet.
				if (err == EDS_ERR_OBJECT_NOTREADY)
				{
					std::cout << "Object not ready" << std::endl;
					std::this_thread::sleep_for(500ms);
					continue;
				}
				// It doesn't retry it at device busy
				if (err == EDS_ERR_DEVICE_BUSY)
				{
					std::cout << "DeviceBusy" << std::endl;
					break;
				}
			}

			break;
		}
	}
	return true;
}

EdsError EndEvfCommand(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	EdsError err = EDS_ERR_OK;


	// Get the current output device.
	EdsUInt32 device = 0;
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsGetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);

		// Do nothing if the remote live view has already ended.
		if ((device & kEdsEvfOutputDevice_PC) == 0)
		{
			return true;
		}

		// Get depth of field status.
		EdsUInt32 depthOfFieldPreview = 0;
		err = EdsGetPropertyData(cameraArray[i], kEdsPropID_Evf_DepthOfFieldPreview, 0, sizeof(depthOfFieldPreview), &depthOfFieldPreview);

		// Release depth of field in case of depth of field status.
		if (depthOfFieldPreview != 0)
		{
			depthOfFieldPreview = 0;
			err = EdsSetPropertyData(cameraArray[i], kEdsPropID_Evf_DepthOfFieldPreview, 0, sizeof(depthOfFieldPreview), &depthOfFieldPreview);

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
			err = EdsSetPropertyData(cameraArray[i], kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
		}


		EdsUInt32 evfMode = 0;
		err = EdsGetPropertyData(cameraArray[i],
			kEdsPropID_Evf_Mode,
			0,
			sizeof(evfMode),
			&evfMode);


		if (evfMode == 1)
		{
			evfMode = 0;

			// Set to the camera.
			err = EdsSetPropertyData(cameraArray[i], kEdsPropID_Evf_Mode, 0, sizeof(evfMode), &evfMode);
		}

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			// It doesn't retry it at device busy
			if (err == EDS_ERR_DEVICE_BUSY)
			{
				std::cout << "DeviceBusy" << std::endl;
			}
			return false;
		}
	}
	return true;
}
