#include <vector>
#include <cstring>
#include <sstream>
#include <iostream>
#include <filesystem>
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "CanonHandler.h"

namespace fs = std::filesystem;

struct FileNumber
{
	EdsDirectoryItemRef DcimItem;
	int dirnum;
	int* filenum;

	FileNumber(int p1)
	{
		DcimItem = NULL;
		dirnum = p1;
		filenum = new int[dirnum];
	}
};

EdsError downloadImage(EdsDirectoryItemRef  directoryItem, EdsVoid* _bodyID);
EdsError CountImages(EdsDirectoryItemRef directoryItem, EdsUInt32* directory_count, int* fileCount, FileNumber const& fileNumber, std::vector<EdsDirectoryItemRef> & imageItems);
EdsError CountImagesByDirectory(EdsDirectoryItemRef directoryItem, int directoryNo, int* image_count, std::vector<EdsDirectoryItemRef> & imageItems);
EdsError CountDirectory(EdsDirectoryItemRef directoryItem, EdsUInt32* directory_count);
EdsError DownloadImageAll(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID);
EdsError DownloadImageByDirectory(EdsUInt64 _bodyID, int volume_num);

static int _currentFileNum = 0;
static int _fileCount = 0;
static std::vector<EdsDirectoryItemRef> _imageItems;

EdsError downloadImage(EdsDirectoryItemRef  directoryItem, EdsVoid* _bodyID)
{
	EdsError err = EDS_ERR_OK;
	EdsStreamRef stream = NULL;     // Get directory item information  
	EdsDirectoryItemInfo  dirItemInfo;

	err = EdsGetDirectoryItemInfo(directoryItem, &dirItemInfo);

	// create folder  ex) cam1
	EdsUInt32 camid;
	camid = (EdsUInt64)_bodyID;
	// std::cout << "Download " << camid << std::endl;
	std::string directory_tree = canonhandle.save_dir;// + "\\cam" + std::to_string(camid);
	if (fs::exists(directory_tree) == FALSE)
	{
		std::filesystem::create_directories(directory_tree);
	}

	// Create filename
	std::string tmp;
	std::stringstream out_file;
	out_file.str("");
	out_file << "\\cam" <<  std::to_string(camid) << "_" 
		<< std::setfill('0') << std::setw(3) << canonhandle.turntable_position << "_img.jpg";
	tmp = directory_tree + out_file.str();
	std::cout << "Saving: " << out_file.str() << std::endl;
	char* filename = new char[tmp.size() + 1];
	strcpy(filename, tmp.c_str());


	// Create file stream for transfer destination  
	if (err == EDS_ERR_OK)
	{
		err = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);
	}

	// Download image  
	if (err == EDS_ERR_OK)
	{
		err = EdsDownload(directoryItem, dirItemInfo.size, stream);
	}
	// Issue notification that download is complete  
	if (err == EDS_ERR_OK) {
		err = EdsDownloadComplete(directoryItem);
	}

	// Release stream  
	if (stream != NULL) {
		EdsRelease(stream);   stream = NULL;
	}

	canonhandle.images_downloaded++;

	return err;
}

EdsError CountImages(EdsDirectoryItemRef* directoryItem, EdsUInt32* directory_count, int* fileCount, const FileNumber & fileNumber, std::vector<EdsDirectoryItemRef> & imageItems)
{
	EdsError err = EDS_ERR_OK;

	// Get the number of camera volumes
	fileCount = 0;

	// Get retrieved item information

	for (EdsUInt32 i = 0; i < *directory_count; ++i)
	{
		int count = 0;
		err = CountImagesByDirectory(*directoryItem, i, &count, imageItems);
		if (err != EDS_ERR_OK)
		{
			return err;
		}
		fileCount += count;
		fileNumber.filenum[i] = count;
	}
	return EDS_ERR_OK;
}

EdsError CountImagesByDirectory(EdsDirectoryItemRef directoryItem, int directoryNo, int* image_count, std::vector<EdsDirectoryItemRef> & imageItems)
{
	EdsUInt32 item_count = 0;

	EdsDirectoryItemRef directoryfiles;
	EdsDirectoryItemRef fileitem;
	EdsDirectoryItemInfo dirItemInfo;

	EdsError err = EdsGetChildAtIndex(directoryItem, directoryNo, &directoryfiles);
	if (err != EDS_ERR_OK)
	{
		return err;
	}

	// Get retrieved item information
	// Get item name
	err = EdsGetDirectoryItemInfo(directoryfiles, &dirItemInfo);
	if (err != EDS_ERR_OK)
	{
		return err;
	}

	EdsUInt32 index = 0, filecount = 0;
	err = EdsGetChildCount(directoryfiles, &item_count);
	if (err != EDS_ERR_OK)
	{
		return err;
	}
	for (index = 0; index < item_count; ++index)
	{
		err = EdsGetChildAtIndex(directoryfiles, index, &fileitem);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		// Get retrieved item information
		err = EdsGetDirectoryItemInfo(fileitem, &dirItemInfo);
		if (err != EDS_ERR_OK)
		{
			return err;
		}
		if (dirItemInfo.isFolder == 0)
		{
			imageItems.push_back(fileitem);
			filecount += 1;
		}

	}
	*image_count = filecount;

	return EDS_ERR_OK;
}

EdsError CountDirectory(EdsBaseRef _volume, EdsDirectoryItemRef* directoryItem, EdsUInt32* directory_count)
{
	EdsError err = EDS_ERR_OK;
	EdsUInt32 item_count = 0;
	*directory_count = 0;

	// Get DCIM folder
	EdsDirectoryItemRef dirItem;
	EdsDirectoryItemInfo dirItemInfo;
	dirItemInfo.szFileName[EDS_MAX_NAME - 1] = {  };
	dirItemInfo.size = 0;

	err = EdsGetChildCount(_volume, &item_count);
	if (err != EDS_ERR_OK)
	{
		return err;
	}
	for (EdsUInt32 i = 0; i < item_count; ++i)
	{
		// Get the ith item under the specifed volume
		err = EdsGetChildAtIndex(_volume, i, &dirItem);
		if (err != EDS_ERR_OK)
		{
			continue;
		}

		// Get retrieved item information
		err = EdsGetDirectoryItemInfo(dirItem, &dirItemInfo);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		std::string filename;
		filename = dirItemInfo.szFileName;
		// Indicates whether or not the retrieved item is a DCIM folder.
		if (filename == "DCIM" && dirItemInfo.isFolder == 1)
		{
			*directoryItem = dirItem;
			break;
		}

		// Release retrieved item
		if (dirItem != NULL)
		{
			EdsRelease(dirItem);
		}
	}

	// Get number of directory in DCIM.
	return err = EdsGetChildCount(*directoryItem, directory_count);

}

EdsError DownloadImageAll(std::vector<EdsCameraRef> const& cameraArray, std::vector<EdsUInt64> const& bodyID)
{
	int i;
	EdsUInt32 volume_count = 0;
	EdsBaseRef volumes[2] = {};
	EdsError				err = EDS_ERR_OK;
	EdsVolumeInfo outVolumeInfo;
	EdsDirectoryItemRef  directoryItem = nullptr;

	for (i = 0; i < cameraArray.size(); i++)
	{
		err = EdsGetChildCount(cameraArray[i], &volume_count);
		if (volume_count > 0) {
			for (EdsUInt32 j = 0; j < volume_count; j++)
			{
				err = EdsGetChildAtIndex(cameraArray[i], j, &volumes[j]);
				err = EdsGetVolumeInfo(volumes[j], &outVolumeInfo);
				if (outVolumeInfo.storageType != kEdsStorageType_Non)
				{
					std::cout << "Memory Card " << j+1 << " (" << outVolumeInfo.szVolumeLabel << ")" << std::endl;
				}
				// download all
				_currentFileNum = 0;

				EdsUInt32 directoryCount;
				err = CountDirectory(volumes[j], &directoryItem, &directoryCount);

				FileNumber _fileNumber(directoryCount);
				_fileNumber.DcimItem = directoryItem;
				err = CountImages(&directoryItem, &directoryCount, &_fileCount, _fileNumber, _imageItems);
				if (err != EDS_ERR_OK)
				{
					return false;
				}

				// Download file.
				err = DownloadImageByDirectory(bodyID[i], j);
				if (err != EDS_ERR_OK)
				{
					return false;
				}
			}
		}
	}
	return true;
}

EdsError DownloadImageByDirectory(EdsUInt64 _bodyID, int volume_num)
{
	EdsDirectoryItemRef fileitem;
	EdsDirectoryItemInfo dirItemInfo;
	dirItemInfo.szFileName[EDS_MAX_NAME - 1] = {  };
	dirItemInfo.size = 0;

	EdsUInt64 index = 0, filenum = _imageItems.size();

	for (index = 0; index < filenum; ++index)
	{

		fileitem = _imageItems[index];
		EdsError err = EdsGetDirectoryItemInfo(fileitem, &dirItemInfo);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		// create folder  ex) cam1
		EdsUInt32 camid;
		camid = _bodyID;
		std::string directory_tree = "cam" + std::to_string(camid) + "\\card" + std::to_string(volume_num+1);
		if (fs::exists(directory_tree) == FALSE)
		{
			std::filesystem::create_directories(directory_tree);
		}

		std::string tmp;
		tmp = directory_tree + "\\" + dirItemInfo.szFileName;
		char* szDstFileName = new char[tmp.size() + 1];
		strcpy(szDstFileName, tmp.c_str());

		// Create file stream for transfer destination
		EdsStreamRef stream;

		err = EdsCreateFileStream(szDstFileName,
			kEdsFileCreateDisposition_CreateAlways,
			kEdsAccess_ReadWrite, &stream);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		//Set Progress
		//err = EdsSetProgressCallback(stream, ProgressFunc, EDSDKLib.EDSDK.kEdsProgressOption_Periodically, this);
		// Download image
		err = EdsDownload(fileitem, dirItemInfo.size, stream);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		// Issue notification that download is complete
		err = EdsDownloadComplete(fileitem);
		if (err != EDS_ERR_OK)
		{
			return err;
		}

		//Release Item
		if (fileitem != NULL)
		{
			err = EdsRelease(fileitem);
			fileitem = NULL;
		}

		//Release stream
		if (stream != NULL)
		{
			err = EdsRelease(stream);
			stream = NULL;
		}

		_currentFileNum += 1;
		std::cout << "Downloading files " << _currentFileNum << "/" << filenum << "\r";
	}

		std::cout << "\n";
	_currentFileNum = 0;
	_imageItems.clear();
	return EDS_ERR_OK;
}
