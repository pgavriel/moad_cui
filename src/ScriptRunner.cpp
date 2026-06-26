/*
    ScriptRunner.cpp
    ----------------
    See ScriptRunner.h for documentation.
*/

#include "ScriptRunner.h"
#include "ConfigHandler.h"
#include "DebugUtils.h"

#include <filesystem>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// Platform path separator
#ifdef _WIN32
    static const char PATH_SEP = '\\';
#else
    static const char PATH_SEP = '/';
#endif


/* -----------------------------------------------------------------------------
Creates a json file for adding metadata to scans collected using the MOAD rig
*/
void create_obj_info_json(const std::string& output_dir, const std::string& object_name) {
    DebugUtils::logInfo("Creating object info JSON file: " + object_name);

    if (!fs::exists(output_dir)) {
        DebugUtils::logWarning("Output dir '" + output_dir + "' does not exist.");
        return;
    }

    std::string full_path = output_dir + PATH_SEP + object_name + PATH_SEP + "object_info.json";
    if (fs::exists(full_path)) {
        DebugUtils::logDebug("Object info file already exists. Skipping...");
        return;
    }

    std::stringstream cmd;
    cmd << "python3 "
        << "scripts/create_object_info.py "
        << object_name << " "
        << "-p " << output_dir << " ";

    std::string command = cmd.str();
    DebugUtils::logDebug("Executing: " + command);
    system(command.c_str());
}


/* -----------------------------------------------------------------------------
Generates the "virtual" camera transforms for a scan using a set of camera calibration
extrinsics/intrinsics. Outputs a transforms.json file containing the camera pose for each 
DSLR image frame. Required for NeRF reconstruction and SceneReplica annotation generation.
*/
bool generate_transforms(int degree_inc, int num_moves, char curr_pose) {
    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Generating transforms.json...");

    ConfigHandler& config = ConfigHandler::getInstance();

    bool        force           = config.getValue<bool>("transform_generator.force");
    bool        visualize       = config.getValue<bool>("transform_generator.visualize");
    std::string calibration_dir = config.getValue<std::string>("transform_generator.calibration_dir");
    std::string calibration     = config.getValue<std::string>("transform_generator.calibration_mode");
    std::string output_dir      = config.getValue<std::string>("output_dir");
    std::string object_name     = config.getValue<std::string>("object_name");

    int range = degree_inc * num_moves;

    std::stringstream cmd;
    cmd << "python3 "
        << "scripts/transform_generator.py "
        << object_name      << " "
        << "-d " << degree_inc      << " "
        << "-r " << range           << " "
        << "-c " << calibration     << " "
        << "--calibration_dir " << calibration_dir << " "
        << "-p " << output_dir      << " "
        << "--pose pose-"    << curr_pose;

    if (visualize) cmd << " -v";
    if (force)     cmd << " -f";

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    std::this_thread::sleep_for(500ms);
    system(command.c_str());

    return false;
}


/* -----------------------------------------------------------------------------
Checks that all expected image frames are accounted for (no missing data).
Additionally handles the image copying/downscaling/renaming required for NeRF training.
*/
bool run_filecount_check(const std::string& scan_folder) {
    ConfigHandler& config = ConfigHandler::getInstance();

    if (!config.getValue<bool>("filecount_testing.enabled")) {
        DebugUtils::logDebug("Filecount testing disabled in config, skipping.");
        return false;
    }

    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Running file count checking script...");

    std::stringstream cmd;
    cmd << "python3 scripts/filecount_test.py ";

    if (config.getValue<bool>("filecount_testing.count"))
        cmd << "--count ";
    if (config.getValue<bool>("filecount_testing.create"))
        cmd << "--create ";
    if (config.getValue<bool>("filecount_testing.manual_check"))
        cmd << "--manual_check ";
    if (config.getValue<bool>("filecount_testing.delay"))
        cmd << "--prog_delay --delay=0.1 ";

    cmd << "--directory=\"" << scan_folder << "\" ";

    if (config.getValue<bool>("filecount_testing.check_single_object"))
        cmd << "--check_single_object ";

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    system(command.c_str());

    return true;
}
