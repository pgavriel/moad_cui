/*
    ScanManager.cpp
    See ScanManager.h for documentation.
*/
#include "ScanManager.h"
#include "MOADGlobals.h"
#include "ConfigHandler.h"
#include "DebugUtils.h"
#include "ScriptRunner.h"
#include "ThreadPool.h"

// Canon SDK
#include "EDSDK.h"
#include "TakePicture.h"
#include "PreSetting.h"
#include "Property.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

#ifdef _WIN32
    static const char PATH_SEP = '\\';
#else
    static const char PATH_SEP = '/';
#endif


/* ---------------------------------------------------------------------------
Creates all necessary output folders for data collection.
--------------------------------------------------------------------------- */
void prepare_scan_output_folders() {
    DebugUtils::logInfo("Preparing for data collection...");
    ConfigHandler& config = ConfigHandler::getInstance();

    if (config.getValue<bool>("realsense.enable_collection")) {
        rshandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "realsense";
        create_folder(rshandle.save_dir, true);
    }

    if (config.getValue<bool>("dslr.enable_collection")) {
        canonhandle.save_dir = scan_folder + PATH_SEP + "pose-" + curr_pose + PATH_SEP + "DSLR";
        create_folder(canonhandle.save_dir, true);
    }

    DebugUtils::logWhitespace();
}


/* ---------------------------------------------------------------------------
Collects a single data frame from all enabled sensors
--------------------------------------------------------------------------- */
bool scan(ThreadPool* pool) {
    ConfigHandler& config = ConfigHandler::getInstance();
    std::string object_name = config.getValue<std::string>("object_name");
    std::string output_dir  = config.getValue<std::string>("output_dir");
    scan_folder = output_dir + PATH_SEP + object_name;

    bool safe_take_picture = config.getValue<bool>("dslr.safe_take_picture");

    DebugUtils::logDebug("Collecting data at position: " + std::to_string(degree_tracker) + "°");

    // ── RealSense ────────────────────────────────────────────────────────────
    if (config.getValue<bool>("realsense.enable_collection")) {
        DebugUtils::logDebug("Getting RealSense data...");
        int rs_timeout = config.getValue<int>("realsense.realsense_timeout_sec") * 1000;
        rshandle.turntable_position = degree_tracker;
        rshandle.get_current_frame(degree_tracker, rs_timeout, pool);

        if (rshandle.fail_count > 0) {
            DebugUtils::logError("RealSense failed to get frame.");
            return false;
        }
        DebugUtils::logDebug("RealSense frame captured.");
    }

    // ── DSLR ─────────────────────────────────────────────────────────────────
    if (config.getValue<bool>("dslr.enable_collection")) {
        DebugUtils::logDebug("Getting DSLR data...");
        canonhandle.images_downloaded = 0;
        canonhandle.turntable_position = degree_tracker;

        std::vector<std::thread> threads;
        for (auto& camera : canonhandle.cameraArray) {
            std::string cam_name = canonhandle.camera_name[camera];
            threads.emplace_back([&, camera, cam_name]() {
                EdsError err = safe_take_picture
                    ? TakePicture(camera, cam_name)
                    : TakePictureNoWait(camera, cam_name);
                if (err != EDS_ERR_OK)
                    DebugUtils::logError("Error taking picture: " + cam_name);
            });
        }

        DebugUtils::logDebug("Waiting for DSLR threads...");
        for (auto& t : threads) t.join();

        // Wait for all downloads to complete
        while (canonhandle.images_downloaded != canonhandle.cameras_found) {
            EdsGetEvent();
            DebugUtils::logDebug("Waiting for downloads: "
                + std::to_string(canonhandle.images_downloaded) + "/"
                + std::to_string(canonhandle.cameras_found));
            std::this_thread::sleep_for(100ms);
        }
        DebugUtils::logDebug("All DSLR images downloaded.");
    }

    return true;
}


/*
Writes the proper command over serial connection to move the turntable a specified number of degrees.
Accepts negative and positive numbers for different directions.
*/
void rotate_turntable(int degree_inc) {
    ConfigHandler& config = ConfigHandler::getInstance();
    std::string degree_str = std::to_string(degree_inc);
    char* send = &degree_str[0];

    if (Serial->WriteSerialPort(send)) {
        int wait_time = config.getValue<int>("turntable_control.command_timeout_s");
        DebugUtils::logTurntable("Moving " + std::to_string(degree_inc)
            + "°, waiting up to " + std::to_string(wait_time) + "s...");

        std::string incoming = Serial->ReadSerialPort(wait_time, "json");
        DebugUtils::logTurntable("Turntable response: " + incoming);

        int delay_ms = config.getValue<int>("turntable_control.delay_after_move_ms");
        if (delay_ms > 0) {
            DebugUtils::logTurntable("Post-move delay: " + std::to_string(delay_ms) + "ms", 2);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    } else {
        DebugUtils::logError("Serial command not sent to turntable.");
    }
}

/* ---------------------------------------------------------------------------
Called after scanning. Saves the current camera settings in a json file within
the scan output directory.
--------------------------------------------------------------------------- */
void saveCameraConfig(std::string path) {
    DebugUtils::logFileSys("Saving camera_config.json to: " + path);

    std::vector<std::tuple<EdsPropertyID, std::map<EdsUInt32, const char*>>> propertyIDs = {
        { kEdsPropID_ISOSpeed,    iso_table        },
        { kEdsPropID_Tv,          tv_table         },
        { kEdsPropID_Av,          av_table         },
        { kEdsPropID_WhiteBalance, whitebalance_table },
    };

    ConfigHandler& config = ConfigHandler::getInstance();
    nlohmann::json json_data;

    for (auto& camera : canonhandle.cameraArray) {
        std::string cam = canonhandle.camera_name[camera];
        EdsDeviceInfo deviceInfo;
        EdsGetDeviceInfo(camera, &deviceInfo);
        json_data[cam]["Model"]        = deviceInfo.szDeviceDescription;
        json_data[cam]["Focal Length"] = config.getValue<std::string>("transform_generator.calibration_mode");
    }

    for (auto& [propID, propTable] : propertyIDs) {
        std::string name = std::get<0>(getPropertyString(propID));
        std::map<EdsUInt32, const char*> out_table;
        GetPropertyDesc(canonhandle.cameraArray, canonhandle.bodyID, propID, propTable, out_table, false);
        std::vector<std::string> value_arr;
        GetProperty(canonhandle.cameraArray, canonhandle.bodyID, propID, out_table, value_arr);
        for (auto& camera : canonhandle.cameraArray)
            json_data[canonhandle.camera_name[camera]][name] = value_arr[0];
    }

    std::ofstream file(path + "/camera_config.json");
    if (file.is_open()) {
        file << json_data.dump(4);
        DebugUtils::logDebug("Camera config saved.");
    } else {
        DebugUtils::logError("Failed to write camera_config.json to: " + path);
    }
}

/* ---------------------------------------------------------------------------
Saves the total scan time into the camera_config.json (created by saveCameraConfig)
--------------------------------------------------------------------------- */
void saveScanTime(std::chrono::milliseconds duration, std::string path) {
    std::string full_path = path + PATH_SEP + "camera_config.json";
    std::ifstream file(full_path);
    if (!file.is_open()) {
        DebugUtils::logError("Failed to open camera_config.json: " + full_path);
        return;
    }
    nlohmann::json json_data;
    file >> json_data;
    file.close();

    int minutes = duration.count() / 60000;
    int seconds = (duration.count() % 60000) / 1000;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds;
    json_data["Scan Time"] = oss.str();

    std::ofstream out(full_path);
    if (out.is_open()) {
        out << json_data.dump(4);
        DebugUtils::logDebug("Scan time saved to camera_config.json");
    } else {
        DebugUtils::logError("Failed to write scan time to: " + full_path);
    }
}


/* ---------------------------------------------------------------------------
Called post-scan. Logs the scan duration.
--------------------------------------------------------------------------- */
static void log_scan_time(std::chrono::milliseconds duration) {
    int minutes = duration.count() / 60000;
    int seconds = (duration.count() % 60000) / 1000;
    std::ostringstream oss;
    oss << "Scan Time: "
        << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds << "  [mm:ss]";
    DebugUtils::logDebug(oss.str());
    DebugUtils::logDebug("RS Fail Count: " + std::to_string(rshandle.fail_count));
}

/* ---------------------------------------------------------------------------
Post-scan functions, logging scan time, camera config, generating transforms,
and running the filecount check script.
--------------------------------------------------------------------------- */
static void finish_scan(std::chrono::milliseconds duration, int degree_inc, int num_moves) {
    ConfigHandler& config = ConfigHandler::getInstance();

    log_scan_time(duration);

    if (config.getValue<bool>("dslr.enable_collection")) {
        std::string pose_path = scan_folder + PATH_SEP + "pose-" + curr_pose;
        saveCameraConfig(pose_path);
        saveScanTime(duration, pose_path);
        if (config.getValue<bool>("transform_generator.enabled"))
            generate_transforms(degree_inc, num_moves, curr_pose);
    }

    degree_tracker = degree_tracker % 360;
    object_info["Turntable Pos"] = std::to_string(degree_tracker);

    curr_pose++;
    object_info["Pose"] = curr_pose;
    config.writePose(curr_pose);

    run_filecount_check(scan_folder);
    MenuHandler::WaitUntilKeypress();
}

/* ---------------------------------------------------------------------------
Performs a full data collection with default degree increments defined in the 
config by "degree_inc" and "num_moves".
--------------------------------------------------------------------------- */
bool fullScan() {
    ConfigHandler& config = ConfigHandler::getInstance();
    if (liveview_active) {
        DebugUtils::logWarning("Liveview is active, please stop it before scanning.");
        return false;
    }

    DebugUtils::logInfo("======== Starting full scan (pose: " + std::string(1, curr_pose) + ") ========");
    prepare_scan_output_folders();

    int thread_num = config.getValue<int>("thread_num");
    ThreadPool pool(thread_num);
    int degree_inc = config.getValue<int>("degree_inc");
    int num_moves  = config.getValue<int>("num_moves");
    degree_tracker = 0;

    auto start = std::chrono::high_resolution_clock::now();

    for (int rots = 0; rots < num_moves; rots++) {
        DebugUtils::logWhitespace(2);
        DebugUtils::logDebug("Move " + std::to_string(rots + 1) + "/" + std::to_string(num_moves));

        if (!scan(&pool)) {
            DebugUtils::logError("scan() failed at move "
                + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + ". Aborting.");
            break;
        }

        rotate_turntable(degree_inc);
        degree_tracker += degree_inc;
        config.writeDegreeMove(degree_tracker, rots + 1);
        DebugUtils::logDebug("Frame " + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + " done.");
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    DebugUtils::logWhitespace();
    DebugUtils::logInfo("=== Full Scan Finished ===");
    finish_scan(duration, degree_inc, num_moves);
    return true;
}


/* ---------------------------------------------------------------------------
Perform a data collection with custom degree increment and number of moves,
input via command line prompts. 
--------------------------------------------------------------------------- */
bool customScan() {
    ConfigHandler& config = ConfigHandler::getInstance();
    if (liveview_active) {
        DebugUtils::logWarning("Liveview is active, please stop it before scanning.");
        return false;
    }

    DebugUtils::logInfo("======== Starting custom scan ========");
    prepare_scan_output_folders();

    int thread_num = config.getValue<int>("thread_num");
    ThreadPool pool(thread_num);

    int degree_inc = 0, num_moves = 0;
    std::cout << "Enter degrees per move: ";  std::cin >> degree_inc;
    std::cout << "Enter number of moves: ";   std::cin >> num_moves;
    DebugUtils::logDebug("Custom scan: " + std::to_string(degree_inc)
        + "° × " + std::to_string(num_moves) + " moves");

    auto start = std::chrono::high_resolution_clock::now();

    for (int rots = 0; rots < num_moves; rots++) {
        DebugUtils::logWhitespace(2);
        DebugUtils::logInfo("Frame " + std::to_string(rots + 1) + "/" + std::to_string(num_moves));

        if (!scan(&pool)) {
            DebugUtils::logError("scan() failed at move "
                + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + ". Aborting.");
            std::this_thread::sleep_for(3000ms);
            break;
        }

        rotate_turntable(degree_inc);
        degree_tracker += degree_inc;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    DebugUtils::logWhitespace();
    DebugUtils::logInfo("=== Custom Scan Finished ===");
    finish_scan(duration, degree_inc, num_moves);
    return true;
}


/* ---------------------------------------------------------------------------
Attempts to recover from a scan that crashed midway through. 
While scanning, the current position is saved at every step into the config.
This function attempts to pick up where the last scan left off and finish cleanly.
NOTE: This was created when we were experiencing issues with cameras randomly 
crashing during scans, the cause of which was never fully illuminated, but it 
no longer seems to be an issue. 
--------------------------------------------------------------------------- */
bool scanFromSaveState() {
    ConfigHandler& config = ConfigHandler::getInstance();
    if (liveview_active) {
        DebugUtils::logWarning("Liveview is active, please stop it before scanning.");
        return false;
    }

    DebugUtils::logInfo("======== Starting scan from saved state ========");
    prepare_scan_output_folders();

    int thread_num = config.getValue<int>("thread_num");
    ThreadPool pool(thread_num);

    // Read previous state directly from ConfigHandler — no manual file re-read needed
    int prev_turntable_pos = config.getValue<int>("prev_state.turntable_pos");
    int num_moves_done     = config.getValue<int>("prev_state.current_move");
    int num_moves          = config.getValue<int>("num_moves");
    int degree_inc         = config.getValue<int>("degree_inc");

    degree_tracker = prev_turntable_pos;

    // Validity checks
    if (degree_inc <= 0) {
        DebugUtils::logError("Invalid degree_inc: " + std::to_string(degree_inc));
        return false;
    }
    if (num_moves <= 0) {
        DebugUtils::logError("Invalid num_moves: " + std::to_string(num_moves));
        return false;
    }
    if (num_moves_done > num_moves) {
        DebugUtils::logWarning("prev current_move exceeds num_moves, resetting to 0.");
        num_moves_done = 0;
    }

    DebugUtils::logDebug("Resuming from move " + std::to_string(num_moves_done)
        + "/" + std::to_string(num_moves)
        + " at " + std::to_string(degree_tracker) + "°");

    auto start = std::chrono::high_resolution_clock::now();

    for (int rots = num_moves_done; rots < num_moves; rots++) {
        if (!scan(&pool)) {
            DebugUtils::logError("scan() failed at move "
                + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + ". Aborting.");
            std::this_thread::sleep_for(3000ms);
            break;
        }

        rotate_turntable(degree_inc);
        degree_tracker += degree_inc;
        config.writeDegreeMove(degree_tracker, rots + 1);
        DebugUtils::logDebug("Frame " + std::to_string(rots + 1) + "/" + std::to_string(num_moves) + " done.");
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    DebugUtils::logInfo("=== Scan from Saved State Finished ===");
    finish_scan(duration, degree_inc, num_moves);
    return true;
}


/* ---------------------------------------------------------------------------
Collect a single frame from all enabled sensors
--------------------------------------------------------------------------- */
bool collectSampleData() {
    ConfigHandler& config = ConfigHandler::getInstance();
    if (liveview_active) {
        DebugUtils::logWarning("Liveview is active, please stop it before scanning.");
        return false;
    }

    DebugUtils::logInfo("======== Starting single frame capture ========");
    // Create output folders
    prepare_scan_output_folders();

    int thread_num = config.getValue<int>("thread_num");
    ThreadPool pool(thread_num);

    // Collect a single frame from all enabled sensors
    scan(&pool);

    DebugUtils::logInfo("======== Single frame capture complete ========");
    return false;
}