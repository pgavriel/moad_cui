/*
    ScriptRunner.cpp
    ----------------
    See ScriptRunner.h for documentation.
*/

#include "ScriptRunner.h"
#include "ConfigHandler.h"
#include "DebugUtils.h"
#include "MOADGlobals.h"

#include <cstdlib>
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
        << moad_dir + "/scripts/create_object_info.py "
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
bool generate_transforms(int degree_inc, int num_moves, char curr_pose, std::string include_cameras_str) {
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
        << moad_dir + "/scripts/transform_generator.py "
        << object_name      << " "
        << "-d " << degree_inc      << " "
        << "-r " << range           << " "
        << "-c " << calibration     << " "
        << "--calibration_dir " << calibration_dir << " "
        << "-p " << output_dir      << " "
        << "--pose pose-"    << curr_pose << " "
        << "--include-cameras " << include_cameras_str;

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
bool run_filecount_check(int total_frames) {
    ConfigHandler& config = ConfigHandler::getInstance();

    if (!config.getValue<bool>("filecount_testing.enabled")) {
        DebugUtils::logDebug("Filecount testing disabled in config, skipping.");
        return false;
    }

    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Running file count checking script...");
    bool check_filecount        = config.getValue<bool>("filecount_testing.check_filecount");
    bool downscale_frames       = config.getValue<bool>("filecount_testing.downscale_frames");
    std::string output_dir      = config.getValue<std::string>("output_dir");
    std::string object_name     = config.getValue<std::string>("object_name");
    char pose = config.getValue<char>("prev_state.current_pose");

    
    std::string target = object_name+"/pose-"+pose;

    std::stringstream cmd;
    cmd << "python3 " + moad_dir + "/scripts/filecount_test.py ";
    cmd << "--data-root " << output_dir << " ";
    cmd << "--target-scans " << target << " ";
    cmd << "--total-frames " << total_frames << " ";
    cmd << "--check-count " << check_filecount << " ";
    cmd << "--downscale " << downscale_frames;

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    system(command.c_str());

    return true;
}


/* -----------------------------------------------------------------------------
Checks that all expected image frames are accounted for (no missing data).
Additionally handles the image copying/downscaling/renaming required for NeRF training.
*/
bool run_replica_live_view() {
    ConfigHandler& config = ConfigHandler::getInstance();

    // TODO: Check that there are files in the scene_replica folder
    
    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Running Scene Replica Live View...");
    if (!liveview_active) {
        DebugUtils::logWarning("Live View must be active for live alignment. Will use fallback images.");
    }

    //TODO: Right now this gets the current calibration from the TF generator section
    std::string calibration   = config.getValue<std::string>("transform_generator.calibration_mode");
    std::string sr_config     = config.getValue<std::string>("scene_replica.config_file");
    
    std::string scene_path = config.getValue<std::string>("scene_replica.scene_root") + "/"
            + config.getValue<std::string>("scene_replica.scene_folder") + "/" 
            + config.getValue<std::string>("scene_replica.scene_file");
    DebugUtils::logDebug("Using scene path from MOAD config: " + scene_path);

    // TODO: Change to take scene from moad_config instead of sr_config
    std::stringstream cmd;
    cmd << "python3 "
        << moad_dir + "/scene_replica_moad/replica_live_viewer.py "
        << "--liveview-root " << moad_dir << "/live_view_filestream "
        << "--calib-root " << moad_dir << "/calibration "
        << "--calibration " << calibration << " "
        << "--scene-config " << moad_dir << "/scene_replica_moad/config/" << sr_config << " "
        << "--moad-config " << moad_dir << "/config/moad_config.json "
        << "--fallback-images-dir " << moad_dir << "/scene_replica_moad/assets/fallback_images/" << calibration << " "
        << "--scene-file " << scene_path;

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    system(command.c_str());

    return true;
}


bool replica_generate_annotations() {
    ConfigHandler& config = ConfigHandler::getInstance();

    // TODO: Check that there are files in the scene_replica folder
    
    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Generating Scene Replica Annotations...");

    //TODO: Right now this gets the current calibration from the TF generator section
    std::string calibration   = config.getValue<std::string>("transform_generator.calibration_mode");
    std::string sr_config     = config.getValue<std::string>("scene_replica.config_file");
    
    std::string scene_path = config.getValue<std::string>("scene_replica.scene_root") + "/"
            + config.getValue<std::string>("scene_replica.scene_folder") + "/" 
            + config.getValue<std::string>("scene_replica.scene_file");
    DebugUtils::logDebug("Using scene path from MOAD config: " + scene_path);

    std::string output_dir = config.getValue<std::string>("output_dir");
    std::string obj_name = config.getValue<std::string>("object_name");
    char pose = static_cast<char>(config.getValue<int>("prev_state.current_pose"));
    std::string pose_folder = "pose-" + std::string(1, pose);
    DebugUtils::logDebug("Pose Folder: "+pose_folder);
    DebugUtils::logDebug("Generating Annotations for "+output_dir+"/"+obj_name+"/"+pose_folder);
    std::string model_library = config.getValue<std::string>("scene_replica.annotations.model_library");
    
    // TODO: Change to take scene from moad_config instead of sr_config
    std::stringstream cmd;
    cmd << "python3 "
        << moad_dir + "/scripts/replica_generate_annotations.py "
        << "--data-root " << output_dir << " "
        << "--object " << obj_name << " "
        << "--pose " << pose_folder << " "
        << "--calib-root " << moad_dir << "/calibration "
        << "--calibration " << calibration << " "
        << "--model-library " << model_library << " ";
        // << "--scene-config " << moad_dir << "/scene_replica_moad/config/" << sr_config << " "
        // << "--moad-config " << moad_dir << "/config/moad_config.json "
        // << "--fallback-images-dir " << moad_dir << "/scene_replica_moad/assets/fallback_images/" << calibration << " "
        // << "--scene-file " << scene_path;

    if (config.getValue<bool>("scene_replica.annotations.pose")){
        cmd << "--generate-poses ";
    }
    if (config.getValue<bool>("scene_replica.annotations.bounding_boxes")){
        cmd << "--generate-bbs ";
    }
    if (config.getValue<bool>("scene_replica.annotations.masks")){
        cmd << "--generate-masks ";
    }

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    system(command.c_str());

    return true;
}

/* -----------------------------------------------------------------------------
Generates the depth frames for DSLR data by running inference on DepthAnythingv3 and scaling the output
using the calibration/settings defined in tools/da3_venv/da3_config.yaml
*/
bool generate_dslr_depth() {
    DebugUtils::logWhitespace();
    DebugUtils::logInfo("Generating DSLR depth frames with DepthAnythingv3...");

    ConfigHandler& config = ConfigHandler::getInstance();

    std::string da3_venv = config.getValue<std::string>("da3_depth.venv_path");
    std::string hf_cache = config.getValue<std::string>("da3_depth.hf_cache");
    // Explicitly set environmental variables, otherwise if running as root, the venv wont be found
    // 1 = overwrite if already set; use 0 to respect an existing value
    setenv("DA3_VENV", da3_venv.c_str(), 1);
    DebugUtils::logDebug("Set DA3_ENV: "+da3_venv);
    setenv("HF_HOME",  hf_cache.c_str(), 1);
    DebugUtils::logDebug("Set HF_HOME: "+hf_cache);

    // std::string calibration_dir = config.getValue<std::string>("transform_generator.calibration_dir");
    // std::string calibration     = config.getValue<std::string>("transform_generator.calibration_mode");
    // std::string output_dir      = config.getValue<std::string>("output_dir");
    char pose = config.getValue<char>("prev_state.current_pose");
    std::string object_name     = config.getValue<std::string>("object_name");
    std::string scan_str        = object_name+"/pose-"+pose;
    DebugUtils::logInfo("Target Scan: "+scan_str);

    std::stringstream cmd;
    cmd << "python3 "
        << moad_dir + "/tools/da3_venv/run_da3_scan.py "
        << "--scan " << scan_str;

    std::string command = cmd.str();
    DebugUtils::logInfo("Executing: " + command);
    std::this_thread::sleep_for(500ms);
    system(command.c_str());

    return false;
}