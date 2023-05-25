#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError GetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> iso_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsGetPropertySize(cameraArray[i],
			kEdsPropID_Tv,
			0,
			&dataType,
			&dataSize);
		if (err == EDS_ERR_OK)
		{
			if (dataType == EdsDataType::kEdsDataType_UInt32)
			{
				EdsUInt32 data = 0;

				//Acquisition of the property
				err = EdsGetPropertyData(cameraArray[i],
					propertyID,
					0,
					dataSize,
					&data);

				std::map<EdsUInt32, const char*>::iterator itr = iso_table.find(data);
				if (itr != iso_table.end())
				{
					// Set String combo box
					std::cout << "camera" << bodyID[i] << " : current setting is ";
					std::cout << itr->second << "\n" << std::endl;
					//					std::cout << "distance=" << std::distance(iso_table.begin(), itr) << std::endl;
				}

				/* hex display
				std::stringstream ss;
				ss << std::showbase << std::hex << data;
				std::cout << "camera" << bodyID[i] << " : current setting is " << ss.str() << "\n" << std::endl;
				*/
			}

			if (dataType == EdsDataType::kEdsDataType_String)
			{

				EdsChar str[EDS_MAX_NAME] = {};
				//Acquisition of the property
				err = EdsGetPropertyData(cameraArray[i],
					propertyID,
					0,
					dataSize,
					str);

				//Acquired property value is set
				if (err == EDS_ERR_OK)
				{
					std::cout << "camera" << bodyID[i] << " : current setting is " << str << "\n" << std::endl;
				}
			}
		}
	}
	return err;
}

EdsError GetPropertyDesc(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsPropertyDesc	 propertyDesc = { 0 };
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		//Get property
		if (err == EDS_ERR_OK)
		{
			err = EdsGetPropertyDesc(cameraArray[i],
				propertyID,
				&propertyDesc);
		}
		if (err == EDS_ERR_OK)
		{
			std::cout << "camera" << bodyID[i] << "'s \t available values are...";
			for (int propDescNum = 0; propDescNum < propertyDesc.numElements; propDescNum++)
			{
				if ((propDescNum % 4) == 0)
				{
					std::cout << std::endl;
				}
				std::map<EdsUInt32, const char*>::iterator itr = prop_table.find(propertyDesc.propDesc[propDescNum]);
				if (itr != prop_table.end())
				{
					std::cout << std::setw(4) << std::right << std::distance(prop_table.begin(), itr) << ":";
					std::cout << std::setw(7) << std::right << itr->second << "      ";
					std::cout << std::left;
				}

				/* hex display
				std::stringstream ss;
				ss << std::showbase << std::hex << propertyDesc.propDesc[propDescNum] << "\t";
				std::cout << ss.str();
				*/
			}
			std::cout << "\n" << std::endl;
		}
	}
	return err;
}

/*
EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt32> const& bodyID, EdsPropertyID propertyID, EdsVoid* data)
{
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;	// Set property
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsGetPropertySize(cameraArray[i],
			kEdsPropID_Tv,
			0,
			&dataType,
			&dataSize);

		err = EdsSetPropertyData(cameraArray[i],
			propertyID,
			0,
			dataSize,
			(EdsVoid*)data);

		//Notification of error
		if (err != EDS_ERR_OK)
		{
			// It retries it at device busy
			if (err == EDS_ERR_DEVICE_BUSY)
			{
				std::cout << "DeviceBusy";
			}
			return err;
		}

		std::cout << "camera" << bodyID[i] << " : property changed." << std::endl;

	}
	return err;
}
*/

EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;	// Set property
	EdsPropertyDesc	 propertyDesc = { 0 };
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		//Get property
		if (err == EDS_ERR_OK)
		{
			err = EdsGetPropertyDesc(cameraArray[i],
				propertyID,
				&propertyDesc);
			EdsInt32 input_prop;
			if (err == EDS_ERR_OK)
			{
				auto iter = prop_table.begin();
				std::advance(iter, (EdsInt32)data); // Advance the iterator to the datath map
				input_prop = iter->first;
				bool exists = std::find(propertyDesc.propDesc, propertyDesc.propDesc + propertyDesc.numElements, input_prop) != propertyDesc.propDesc + propertyDesc.numElements;
				if (exists)
				{
					err = EdsGetPropertySize(cameraArray[i],
						kEdsPropID_Tv,
						0,
						&dataType,
						&dataSize);
					if (err == EDS_ERR_OK)
					{
						err = EdsSetPropertyData(cameraArray[i],
							propertyID,
							0,
							dataSize,
							&input_prop);
					}
					//Notification of error
					if (err != EDS_ERR_OK)
					{
						// It retries it at device busy
						if (err == EDS_ERR_DEVICE_BUSY)
						{
							std::cout << "DeviceBusy";
						}
					}
					std::cout << "camera" << bodyID[i] << " : property changed." << std::endl;
				}
				else
				{
					std::cout << "error invalid setting." << std::endl;
				}
			}
		}
	}
	return err;
}