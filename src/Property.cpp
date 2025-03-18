#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "tabulate.hpp"
#include "EDSDK.h"
#include "EDSDKTypes.h"

EdsError GetProperty(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> iso_table, std::string& output){
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;
	int i;
	err = EdsGetPropertySize(camera,
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
			err = EdsGetPropertyData(camera,
				propertyID,
				0,
				dataSize,
				&data);

			// TODO: Should be outside of here
			std::map<EdsUInt32, const char*>::iterator itr = iso_table.find(data);
			if (itr != iso_table.end())
			{
				std::stringstream ss;
				ss << "(" << std::distance(iso_table.begin(), itr) << "): " << itr->second;
				output = ss.str();
				// std::cout << "camera" << bodyID << " : current setting is ";
				// std::cout << itr->second << "\n" << std::endl;
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
			err = EdsGetPropertyData(camera,
				propertyID,
				0,
				dataSize,
				str);
				
			std::stringstream ss;
			ss << str;
			output = ss.str();
			// output = ss.str();
			//Acquired property value is set
			// if (err == EDS_ERR_OK)
			// {
			// 	std::cout << "camera" << bodyID << " : current setting is " << str << "\n" << std::endl;
			// }
		}
	}
	
	return err;
}

EdsError GetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> iso_table, std::vector<std::string>& output)
{
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;
	std::vector<std::string> output_arr;
	
	for (int i = 0; i < cameraArray.size(); i++) {
		std::string output_data;
		err = GetProperty(cameraArray[i], bodyID[i], propertyID, iso_table, output_data);
		std::cout << output_data << std::endl;
		output_arr.push_back(output_data);
	}

	output = output_arr;

	return err;
}

EdsError GetPropertyDesc(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsPropertyDesc	 propertyDesc = { 0 };

	//Get property
	if (err == EDS_ERR_OK)
	{
		err = EdsGetPropertyDesc(camera,
			propertyID,
			&propertyDesc);
	}
	if (err == EDS_ERR_OK)
	{
		// ## Unnecesary printing
		tabulate::Table table;
		table.add_row({"[Type] Possible Values: "});

		int k = 12;
		switch (propertyID)
		{
			case kEdsPropID_DriveMode:
			case kEdsPropID_WhiteBalance:
				k = 3;
				break;
			case kEdsPropID_AEMode:
				k = 5;
				break;
			
			default:
				break;
		}
		int i = 0;
		tabulate::Table optionRow;
		tabulate::Table::Row_t options;
		for (const auto& pair : prop_table) {
			if (i % k == 0 && i >= k) {
				optionRow.add_row(options);
				// Create a new row
				options.clear();
			}
			std::stringstream ss; 

			int first = pair.first;
			std::string second = pair.second;

			ss << i << ": (" << second << ")";
			options.push_back(ss.str());
			i++;
		}

		table.add_row({optionRow});

		std::cout << table << std::endl;
		// Use Tabulate to print values (Dont need to print it 5 times)
		// std::cout << "camera" << bodyID << "'s \t available values are...";
		// for (int propDescNum = 0; propDescNum < propertyDesc.numElements; propDescNum++)
		// {
		// 	if ((propDescNum % 4) == 0)
		// 	{
		// 		std::cout << std::endl;
		// 	}
		// 	std::map<EdsUInt32, const char*>::iterator itr = prop_table.find(propertyDesc.propDesc[propDescNum]);
		// 	if (itr != prop_table.end())
		// 	{
		// 		std::cout << std::setw(4) << std::right << std::distance(prop_table.begin(), itr) << ":";
		// 		std::cout << std::setw(7) << std::right << itr->second << "      ";
		// 		std::cout << std::left;
		// 	}

		// 	/* hex display
		// 	std::stringstream ss;
		// 	ss << std::showbase << std::hex << propertyDesc.propDesc[propDescNum] << "\t";
		// 	std::cout << ss.str();
		// 	*/
		// }
		// std::cout << "\n" << std::endl;
	}
	return err;
}

EdsError GetPropertyDesc(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsPropertyDesc	 propertyDesc = { 0 };
	int i = 0;
	err = GetPropertyDesc(cameraArray[i], bodyID[i], propertyID, prop_table);
	// for (i = 0; i < cameraArray.size(); i++)
	// {
	// 	err = GetPropertyDesc(cameraArray[i], bodyID[i], propertyID, prop_table);
	// }
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

EdsError SetProperty(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table) {
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;	// Set property
	EdsPropertyDesc	 propertyDesc = { 0 };
	
	//Get property
	if (err == EDS_ERR_OK)
	{
		err = EdsGetPropertyDesc(camera,
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
				err = EdsGetPropertySize(camera,
					kEdsPropID_Tv,
					0,
					&dataType,
					&dataSize);
				if (err == EDS_ERR_OK)
				{
					err = EdsSetPropertyData(camera,
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
			}
			else
			{
				std::cout << "error invalid setting." << std::endl;
			}
		}
	}

	return err;
}

EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsDataType	dataType = EdsDataType::kEdsDataType_Unknown;
	EdsUInt32   dataSize = 0;	// Set property
	EdsPropertyDesc	 propertyDesc = { 0 };
	int i;
	for (i = 0; i < cameraArray.size(); i++)
	{
		err = SetProperty(cameraArray[i], bodyID[i], propertyID, data, prop_table);
	}
	return err;
}