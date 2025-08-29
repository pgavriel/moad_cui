#include <map>
#include <vector>
#include <tuple>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "tabulate.hpp"
#include "EDSDK.h"
#include "EDSDKTypes.h"

// Returns a tuple with the property name and description based on the property ID, Used for displaying property information when updating properties.
std::tuple<std::string, std::string> getPropertyString(EdsPropertyID propertyID) {
	switch (propertyID) {
		case kEdsPropID_AEMode:
			return std::tuple<std::string, std::string>(
				"AE Mode", 
				"Some Description"
			);
		case kEdsPropID_DriveMode:
			return std::tuple<std::string, std::string>(
				"Drive Mode", 
				"Affects the shooting buttom, can modify the shotting mode to be either one or multiple picture when holding the shooting buttom. "
			);
		case kEdsPropID_ISOSpeed:
			return std::tuple<std::string, std::string>(
				"ISO", 
				"Affects the Brightness of the picture by changing the sensibility of the light sensor of the camera."
			);
		case kEdsPropID_Av:
			return std::tuple<std::string, std::string>(
				"AV", 
				"Affects the amount of light that goes into the lense, increasing the blur of the image."
			);
		case kEdsPropID_Tv:
			return std::tuple<std::string, std::string>(
				"TV", 
				"Affects how sharp a object is when is moving. When the value is lower, the picture is sharper"
			);
		case kEdsPropID_WhiteBalance:
			return std::tuple<std::string, std::string>(
				"White Balance", 
				"Post-Processing configuration, Create a light filter based on the ambient setting taken by the picture."
			); 
		default:
			return std::tuple<std::string, std::string>(
				"", 
				""
			);
	}
}

// Function to get the property value from the camera and return it as a string.
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
			}
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
		}
	}
	
	return err;
}

// Function to get the property value from multiple cameras and return it as a vector of strings.
EdsError GetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> iso_table, std::vector<std::string>& output)
{
	EdsError	 err = EDS_ERR_OK;
	std::vector<std::string> output_arr;
	
	for (int i = 0; i < cameraArray.size(); i++) {
		std::string output_data;
		err = GetProperty(cameraArray[i], bodyID[i], propertyID, iso_table, output_data);
		output_arr.push_back(output_data);
	}

	output = output_arr;

	return err;
}

// Function to get the property description and create a table with the possible values for the given property ID.
EdsError GetPropertyDesc(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::map<EdsUInt32, const char*>& out_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsPropertyDesc	 propertyDesc = { 0 };
	std::tuple<std::string, std::string> propertyType = getPropertyString(propertyID);
	//Get property
	err = EdsGetPropertyDesc(camera,
		propertyID,
		&propertyDesc);

	if (err == EDS_ERR_OK) {
		// Creation of table
		tabulate::Table table;
		table.add_row({std::get<0>(propertyType) + " Possible Values: "});
		table.add_row({std::get<1>(propertyType)});

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
			// Filtering
			bool found = std::find(propertyDesc.propDesc, propertyDesc.propDesc + propertyDesc.numElements, pair.first) != propertyDesc.propDesc + propertyDesc.numElements;
			if (!found) {
				continue;
			}
			out_table.insert(pair);
			// Adding data to the table
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

		if (options.size() < k) {
			while (options.size() < k) {
				options.push_back(" ***** ");
			}
		}

		optionRow.add_row(options);
		table.add_row({optionRow});

		std::cout << table << std::endl;
	}
	return err;
}

// Function to get the property description and create a table with the possible values for the given property ID from multiple cameras.
EdsError GetPropertyDesc(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::map<EdsUInt32, const char*>& out_table)
{
	EdsError	 err = EDS_ERR_OK;
	EdsPropertyDesc	 propertyDesc = { 0 };
	int i = 0;
	err = GetPropertyDesc(cameraArray[i], bodyID[i], propertyID, prop_table, out_table);
	return err;
}

// Function to set a property on a camera, given the camera reference, body ID, property ID, and data to set.
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
			// Look for the table to find the given property
			auto iter = prop_table.begin();
			std::advance(iter, (EdsInt32)data); // Advance the iterator to the datath map
			// Use the Hex value to see if the data exists
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

// Function to set a property on multiple cameras, given the camera references, body IDs, property ID, and data to set.
EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table)
{
	EdsError	 err = EDS_ERR_OK;

	EdsPropertyDesc	 propertyDesc = { 0 };
	int i;
	for (i = 0; i < cameraArray.size(); i++) {
		err = SetProperty(cameraArray[i], bodyID[i], propertyID, data, prop_table);
	}
	return err;
}