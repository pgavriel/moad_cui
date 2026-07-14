#pragma once

#include <map>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>
#include "EDSDK.h"
#include "EDSDKTypes.h"

std::tuple<std::string, std::string> getPropertyString(EdsPropertyID propertyID);
//EdsError GetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt32> const& bodyID, EdsPropertyID propertyID);
EdsError GetProperty(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::string& output);
EdsError GetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::vector<std::string>& output);
//EdsError GetPropertyDesc(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt32> const& bodyID, EdsPropertyID propertyID);
EdsError GetPropertyDesc(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::map<EdsUInt32, const char*>& out_table, bool print_table = true);
EdsError GetPropertyDesc(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, std::map<EdsUInt32, const char*> prop_table, std::map<EdsUInt32, const char*>& out_table, bool print_table = true);
//EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt32> const& bodyID, EdsPropertyID propertyID, EdsVoid* data);
EdsError SetProperty(EdsCameraRef const& camera, EdsUInt64 const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table);
EdsError SetProperty(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID, EdsPropertyID propertyID, EdsInt32 data, std::map<EdsUInt32, const char*> prop_table);


extern std::map<EdsUInt32, const char*> iso_table;
extern std::map<EdsUInt32, const char*> tv_table;
extern std::map<EdsUInt32, const char*> av_table;
extern std::map<EdsUInt32, const char*> whitebalance_table;
extern std::map<EdsUInt32, const char*> drivemode_table;
extern std::map<EdsUInt32, const char*> AEmode_table;
