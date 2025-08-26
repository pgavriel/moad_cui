#include <vector>
#include <map>
#include <iostream>
#include <thread>
#include "EDSDK.h"
#include "EDSDKTypes.h"
#include "EDSDKErrors.h"
#include "DebugUtils.h"

#include <mutex>
#include <thread> // for thread debugging


static std::mutex dslr_mutex;

// bool WaitForCameraReady1(EdsCameraRef camera, int timeoutMs) {
//     // std::cout << "inside wait function..." << std::endl;
//     // std::cout << "timeout is " << timeoutMs << " sec" << std::endl;
//     int busy_count = 0;
//     auto start = std::chrono::steady_clock::now();
//     while (true) {
//         EdsError err = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UILock, 0);
//         if (err == EDS_ERR_OK) {
//             // if ok then early return
//             EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UIUnLock, 0);
//             return true; // Ready
//         }
//         else if (err != EDS_ERR_DEVICE_BUSY) {
//             // if cam not busy but still err, return err
//             std::cerr << "Camera returned error: " << err << std::endl;
//             return false;
//         } 
//         else {
//             std::cerr << "camera " << camera << " is busy, count: " << busy_count << std::endl;
//             busy_count++;
//         }    
//         if (std::chrono::duration_cast<std::chrono::seconds>(
//                 std::chrono::steady_clock::now() - start).count() >= timeoutMs) {
//             std::cerr << "Camera still busy after timeout." << std::endl;
//             return false;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
// }

bool WaitForCameraReadyStrict(EdsCameraRef camera, int timeoutMs) {

    // attempt to light reset the state of the camera
    EdsError reset1 = EDS_ERR_OK;
    reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_ExitDirectTransfer, 0);
    std::cout << "[RETRY] exit DT, err: " << reset1 << std::endl;
    reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UIUnLock, 0);
    std::cout << "[RETRY] ui unlock, err: " << reset1 << std::endl;

    DebugUtils::logInfo("exit DT, err: " + std::to_string(reset1));
    DebugUtils::logInfo("ui unlock, err: " + std::to_string(reset1));
    

    int busy_count = 0;
    auto start = std::chrono::steady_clock::now();
    std::cout << "Waiting for camera " << camera << " to be ready..." << std::endl;
    // DebugUtils::logInfo("Waiting for camera " + std::to_string(camera) + " to be ready...");


    while (true) {
        // Check multiple camera status commands and prosperties
        EdsError errUI = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UILock, 0); // lock ui
        // std::cout << "[WAIT] Camera " << camera << " UI Lock status: " << errUI << std::endl;

        // Check if camera is ready
        if (errUI == EDS_ERR_OK) {
            EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UIUnLock, 0);
            return true;
        } else if (errUI != EDS_ERR_DEVICE_BUSY) {
            std::cerr << "[ERROR] Camera returned: " << errUI << std::endl;
            return false;
        } else {
            // std::cerr << "[BUSY] Camera " << camera << " count: " << busy_count++ << std::endl;
            busy_count++;
        }

        // Retry shutter press/release if stuck too long
        if (busy_count > 72) {

            std::cout << "[RETRY] Camera " << camera << " is still busy after 72 attempts." << std::endl;
            DebugUtils::logError("\tCamera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " is still busy after 72 attempts.");

            reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_ExitDirectTransfer, 0);
            std::cout << "[RETRY] exit DT, err: " << reset1 << std::endl;
            reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UIUnLock, 0);
            std::cout << "[RETRY] ui unlock, err: " << reset1 << std::endl;



            std::cout << "[RETRY] Attempting to free camera..." << std::endl;
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout << "\tTimestamp: " << std::ctime(&now);

            DebugUtils::logInfo("\tAttempting to free camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + "...");
            DebugUtils::logInfo("\tTimestamp: " + std::to_string(now));

            // attempt shutter press/release if stuck too long

            // EdsError pressErr = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely_NonAF);

            // EdsError releaseErr = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF);
            // std::cout << "[RETRY] Press error: " << pressErr << ", Release error: " << releaseErr << std::endl;





            // attempt full download to try and reset camera state 

            // EdsError downloadErr = EDS_ERR_OK;

            // // Dummy filename and stream for testing
            // const char* filename = "bad.jpg";
            // EdsStreamRef stream = nullptr;
            // EdsDirectoryItemRef directoryItem = nullptr;
            // EdsDirectoryItemInfo dirItemInfo = {0};

            // // Set stream to current directory
            // EdsError dirErr = EdsGetChildAtIndex(camera, 0, &directoryItem);
            // if (dirErr == EDS_ERR_OK && directoryItem != nullptr) {
            //     EdsGetDirectoryItemInfo(directoryItem, &dirItemInfo);
            // } else {
            //     std::cout << "Failed to get current directory item, err: " << dirErr << std::endl;
            // }

            // // Create file stream for transfer destination  
            // if (downloadErr == EDS_ERR_OK) {
            //     downloadErr = EdsCreateFileStream(filename, kEdsFileCreateDisposition_CreateAlways, kEdsAccess_ReadWrite, &stream);
            //     std::cout << "eds create filestream, err: " << downloadErr << std::endl;
            // } else {
            //     std::cout << "eds create filestream, err: " << downloadErr << std::endl;
            // }

            // // Download image  
            // if (downloadErr == EDS_ERR_OK) {
            //     downloadErr = EdsDownload(directoryItem, dirItemInfo.size, stream);
            // } else {
            //     std::cout << "eds download, err: " << downloadErr << std::endl;
            // }

            // // Issue notification that download is complete  
            // if (downloadErr == EDS_ERR_OK) {
            //     downloadErr = EdsDownloadComplete(directoryItem);
            // } else {
            //     std::cout << "eds download complete, err: " << downloadErr << std::endl;
            // }

            // // Release stream  
            // if (stream != NULL) {
            //     downloadErr = EdsRelease(stream);   stream = NULL;
            // } else {
            //     std::cout << "eds release filestream, err: " << downloadErr << std::endl;
            // }



            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
            busy_count = 0;
        }



        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() >= timeoutMs) {
            std::cerr << "[TIMEOUT] Camera still busy after " << timeoutMs << " sec." << std::endl;
            DebugUtils::logError("\tCamera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " is still busy after " + std::to_string(timeoutMs) + " sec.");
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

EdsError TakePicture(EdsCameraRef const& camera, std::string const& bodyID) {

    std::cout << "====\ntaking picture " << bodyID << std::endl;

    std::lock_guard<std::mutex> lock(dslr_mutex);
    std::cout << "[Thread " << std::this_thread::get_id() << "] Shooting cam " << bodyID << std::endl;
    DebugUtils::logThread("thread in use, id " + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));

    EdsError anyErr = EdsGetEvent();
    std::cout << "[TAKEPIC] any err init: " << anyErr << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " initial event: " + std::to_string(anyErr));

    // 1. Ensure camera is ready BEFORE shutter press
    std::cout << "[CHECK] Waiting for camera ready before shutter press..." << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " waiting for ready state...");
    if (!WaitForCameraReadyStrict(camera, 60)) {
        std::cerr << "[FAIL] Camera " << bodyID << " not ready before press." << std::endl;
        DebugUtils::logError("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " not ready before shutter press.");
        return EDS_ERR_DEVICE_BUSY;
    }

    anyErr = EdsGetEvent();
    std::cout << "[TAKEPIC] any err before shutter press down: " << anyErr << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " ready before shutter press: " + std::to_string(anyErr));




    // 2. Press shutter
    EdsError err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely_NonAF);
    std::cout << "[CMD] Press shutter -> error: " << err << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " press shutter command: " + std::to_string(err));
    if (err != EDS_ERR_OK) return err;

    anyErr = EdsGetEvent();
    std::cout << "[TAKEPIC] any err after shutter press down: " << anyErr << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " after shutter press: " + std::to_string(anyErr));
    // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // give shutter mechanism time




    // attempt to set the camera to ready
    EdsError reset1 = EDS_ERR_OK;
    reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_ExitDirectTransfer, 0);
    std::cout << "[RETRY] exit DT, err: " << reset1 << std::endl;
    reset1 = EdsSendStatusCommand(camera, kEdsCameraStatusCommand_UIUnLock, 0);
    std::cout << "[RETRY] ui unlock, err: " << reset1 << std::endl;

    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " exit DT: " + std::to_string(reset1));
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " UI unlock: " + std::to_string(reset1));

    // 3. Immediately release shutter
    err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF);
    std::cout << "[CMD] Release shutter -> error: " << err << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " release shutter command: " + std::to_string(err));
    if (err != EDS_ERR_OK) return err;

    anyErr = EdsGetEvent();
    std::cout << "[TAKEPIC] any err after shutter release: " << anyErr << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " after shutter release: " + std::to_string(anyErr));




    // 4. Wait until the camera finishes saving the image before allowing next capture
    std::cout << "[CHECK] Waiting for camera to finish saving image..." << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " waiting for save to complete...");
    if (!WaitForCameraReadyStrict(camera, 60)) {
        std::cerr << "[FAIL] Camera " << bodyID << " stuck busy after save." << std::endl;
        DebugUtils::logError("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " stuck busy after save.");
        return EDS_ERR_DEVICE_BUSY;
    }

    anyErr = EdsGetEvent();
    std::cout << "[TAKEPIC] any err after camera save: " << anyErr << std::endl;
    DebugUtils::logDebug("Camera " + std::to_string(reinterpret_cast<uintptr_t>(camera)) + " after camera save: " + std::to_string(anyErr));

    std::cout << "[Thread " << std::this_thread::get_id() << "] Finished shooting cam " << bodyID << std::endl;
    DebugUtils::logThread("thread finished, id " + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    return err;
}




// static std::mutex dslr_mutex;

// EdsError SendCommandWithWait(EdsCameraRef const& camera, std::string const& bodyID, EdsUInt32 command, EdsUInt32 param) {
//     EdsError err;
//     int retries = 0;
//     const int max_retries = 900; // e.g., 5 seconds if we sleep 100ms each time

//     do {
//         err = EdsSendCommand(camera, command, param);
//         DebugUtils::logDebug("inside send command for cam " + bodyID + ", err " + std::to_string(err));
//         if (err == EDS_ERR_DEVICE_BUSY) {
//             DebugUtils::logDebug("camera " + bodyID + " is busy, retrying..." + std::to_string(retries) + " out of " + std::to_string(max_retries));
//             std::this_thread::sleep_for(std::chrono::seconds(2));
//             retries++;
//         } else {
//             break;
//         }
//     } while (retries < max_retries);

//     return err;
// }


EdsError TakePictureNoWait(EdsCameraRef const& camera, std::string const& bodyID) {
    std::lock_guard<std::mutex> lock(dslr_mutex); // ensures exclusive access for this function

    EdsError err = EDS_ERR_OK;
    std::cout << "[Thread " << std::this_thread::get_id() << "] Started shooting cam " << bodyID << std::endl;

    DebugUtils::logThread("thread in use, id " + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    DebugUtils::logCam("started taking picture with camera: " + bodyID);

    // Press shutter completely (Non-AF)
    if(err == EDS_ERR_OK) {

	    err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_Completely_NonAF); // kEdsCameraCommand_ShutterButton_Completely

        // err = SendCommandWithWait(camera, bodyID, kEdsCameraCommand_PressShutterButton,
        //                         kEdsCameraCommand_ShutterButton_Completely_NonAF);
        
        DebugUtils::logDebug("shutter pressed successfully, " + std::to_string(err));
    } else {
        DebugUtils::logWarning("shutter press error, " + std::to_string(err) + " instead of EDS_ERR_OK");
    }

    // Release shutter
    if (err == EDS_ERR_OK) {

        err = EdsSendCommand(camera, kEdsCameraCommand_PressShutterButton, kEdsCameraCommand_ShutterButton_OFF); // kEdsCameraCommand_ShutterButton_OFF

        // err = SendCommandWithWait(camera, bodyID, kEdsCameraCommand_PressShutterButton,
        //                           kEdsCameraCommand_ShutterButton_OFF);

        DebugUtils::logDebug("shutter released successfully, " + std::to_string(err));
    } else {
        DebugUtils::logWarning("shutter release error, " + std::to_string(err) + " instead of EDS_ERR_OK");
    }
    
    // #define EDS_ERR_COMM_DISCONNECTED 0x000000C1L
    std::cout << "[Thread " << std::this_thread::get_id() << "] Finished shooting cam " << bodyID << std::endl;

    DebugUtils::logThread("thread finished, id " + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    DebugUtils::logCam("finished taking picture with camera: " + bodyID);

    return err;
}



EdsError TakePicture(std::vector<EdsCameraRef> const& cameraArray, std::map<EdsCameraRef, std::string> const& bodyID) {
	
	std::cout << "multi cam shooting" << std::endl;
	
	EdsError err = EDS_ERR_OK;
	
	// For each camera in the array, take a picture
	for (auto& camera : cameraArray) {
		TakePicture(camera, bodyID.at(camera));
	}

	return err;
}



