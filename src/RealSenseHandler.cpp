#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <string>
#include <sstream>
#include <fstream>
#include <typeinfo>

#include <nlohmann/json.hpp>

// PCL includes
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include "RealSenseHandler.h"
#include "DebugUtils.h"
#include "ConfigHandler.h"

using std::string;
using std::cout;
using std::endl;

// Returns a matrix that performs a specified Z-axis rotation.
Eigen::Matrix4f createRotationMatrix(float angle_degrees)
{
    // Convert the angle from degrees to radians
    float angle_radians = -angle_degrees * M_PI / 180.0;

    // Create an Eigen::AngleAxisf object for the rotation
    Eigen::AngleAxisf rotation(angle_radians, Eigen::Vector3f::UnitZ());

    // Create a 4x4 transformation matrix from the rotation
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation.matrix();

    std::stringstream ss; 
    ss << "Rotation Transform for " << angle_degrees << " degrees:\n" << transformation;
    DebugUtils::logRS(ss.str(),3);

    return transformation;
}

RealSenseHandler::RealSenseHandler() {
    //Configure Depth Frame Filters (These are default in RSViewer)
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.2f); // Minimum threshold distance in meters
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.5f); // Maximum threshold distance in meters
    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // Decimation magnitude
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f); // Spatial filter smooth alpha
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); // Spatial filter smooth delta
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // Spatial filter magnitude
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f); // Temporal filter smooth alpha
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); // Temporal filter smooth delta

}

RealSenseHandler::~RealSenseHandler() {
    shutdown();
}

void RealSenseHandler::shutdown() {
    if(!running) return;  // guard against double-calls

    DebugUtils::logRS("Shutting down RealSense Handler...");
    running = false;
    for (auto& pipe : pipeline_map) {
            pipe.second.stop();
            DebugUtils::logRS("Device stopped.");
            // cout << ". ";
    }
    // for (auto& th : frame_thread_map) {
    //         th.second.join();
    // }

    DebugUtils::logRS("Done.");
}


/*
    RealSenseHandler::initialize()
    ─────────────────────────────────────────────────────────────────────────────
    Initializes the RealSense handler by loading camera identities and extrinsic
    transforms, then starting a capture pipeline for each physically connected
    and configured device.

    Data is sourced from two places:
      1. moad_config.json  ("realsense.camera_ids")
             Maps rs-id → serial number string.
             e.g. { "rs1": "215122257111", "rs2": "239222302632", ... }

      2. realsense_cam_parameters.json  (path from "realsense.calibration_file")
             Contains per-camera extrinsics under "cameras.rsN.extrinsics.c2w".
             The 4x4 c2w matrix is used as the point-cloud transform for each
             device. This file is produced by create_calibration.py.

    The two sources are joined on the rs-id key ("rs1", "rs2", ...).
    On a successful join, camera_names[serial] = id and
    camera_transforms[serial] = c2w_matrix are populated, which is the same
    internal representation used by process_frames() and device_check() —
    so nothing downstream needs to change.

    Fails gracefully at each step, logging a clear message and returning early
    rather than crashing, so the rest of the application can continue running
    without RealSense if needed.

    Args:
        calib_path  path to realsense_cam_parameters.json
*/
void RealSenseHandler::initialize(std::string calib_path) {

    ConfigHandler& config = ConfigHandler::getInstance();

    if (!config.getValue<bool>("realsense.enable_collection")) {
        DebugUtils::logRS("Skipping RealSense setup (realsense.enable_collection=false).");
        DebugUtils::logWhitespace();
        return;
    }

    DebugUtils::logInfo("Initializing RealSense Cameras...");

    // ── 1. Load serial numbers from moad_config  ─────────────────────────────
    // "realsense.camera_ids" maps rs-id → serial number string.
    // We invert this into serial → rs-id for the camera_names map that
    // process_frames() and device_check() look up by serial.
    DebugUtils::logRS("Reading serial numbers from moad_config (realsense.camera_ids)...");

    nlohmann::json camera_ids_json;
    try {
        camera_ids_json = config.getValue<nlohmann::json>("realsense.camera_ids");
    } catch (const std::exception& e) {
        DebugUtils::logError("realsense.camera_ids not found in moad_config: " + std::string(e.what()));
        DebugUtils::logError("Cannot initialize RealSense without serial number mappings.");
        return;
    }

    // Build rs-id → serial lookup (used to join with calibration data below)
    std::map<std::string, std::string> id_to_serial;
    for (auto it = camera_ids_json.begin(); it != camera_ids_json.end(); ++it) {
        const std::string& rs_id = it.key();
        std::string serial = it.value().is_string()
            ? it.value().get<std::string>()
            : std::to_string(it.value().get<long long>());
        id_to_serial[rs_id] = serial;
        DebugUtils::logRS("  " + rs_id + "  →  serial: " + serial);
    }

    if (id_to_serial.empty()) {
        DebugUtils::logError("realsense.camera_ids is empty. Cannot initialize RealSense.");
        return;
    }

    // ── 2. Load calibration file  ─────────────────────────────────────────────
    // realsense_cam_parameters.json contains per-camera c2w extrinsic matrices
    // produced by create_calibration.py from the joint COLMAP reconstruction.
    DebugUtils::logRS("Loading RealSense calibration file: " + calib_path);

    nlohmann::json calib_json;
    try {
        std::ifstream calib_file(calib_path);
        if (!calib_file.is_open()) {
            DebugUtils::logError("Calibration file not found: " + calib_path);
            DebugUtils::logError("Check realsense.calibration_file in moad_config.");
            return;
        }
        calib_json = nlohmann::json::parse(calib_file);
    } catch (const std::exception& e) {
        DebugUtils::logError("Failed to parse calibration file: " + std::string(e.what()));
        return;
    }

    if (!calib_json.contains("cameras") || !calib_json["cameras"].is_object()) {
        DebugUtils::logError("Calibration file missing required 'cameras' block: " + calib_path);
        return;
    }

    const auto& cameras_block = calib_json["cameras"];

    // ── 3. Join serial numbers with calibration extrinsics  ───────────────────
    // For each rs-id in moad_config, look up the matching entry in the
    // calibration file's "cameras" block and extract the c2w matrix.
    // Populates camera_names and camera_transforms (both keyed by serial),
    // which is the internal format the rest of the handler expects.

    // DebugUtils::logRS("Joining serial numbers with calibration extrinsics...");

    // Read scale once before the per-camera loop
    scale_factor = 1.0f; 
    try {
        scale_factor = calib_json["_info"]["scaling"]["scale"].get<float>();
        DebugUtils::logRS("Calibration scale factor: " + std::to_string(scale_factor));
    } catch (const std::exception& e) {
        DebugUtils::logWarning("Could not read scale from calibration file, defaulting to 1.0: "
                            + std::string(e.what()));
    }

    int loaded_count = 0;
    for (const auto& [rs_id, serial] : id_to_serial) {

        // Check this rs-id exists in the calibration file
        if (!cameras_block.contains(rs_id)) {
            DebugUtils::logWarning("  " + rs_id + ": not found in calibration file — skipping.");
            continue;
        }

        const auto& cam_entry = cameras_block[rs_id];

        // Validate the extrinsics block exists and has the expected structure
        if (!cam_entry.contains("extrinsics") ||
            !cam_entry["extrinsics"].contains("c2w")) {
            DebugUtils::logWarning("  " + rs_id + ": missing extrinsics.c2w — skipping.");
            continue;
        }

        // Parse the 4x4 c2w matrix, defaulting to identity on any parse error
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        try {
            const auto& c2w = cam_entry["extrinsics"]["c2w"];
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    transform(r, c) = c2w[r][c].get<float>();

            // Apply scaling - Convert from COLMAP calibration scale translation to metric scale
            // The data itself is already metric scale directly from the sensor.
            transform(0, 3) *= scale_factor;
            transform(1, 3) *= scale_factor;
            transform(2, 3) *= scale_factor;
        } catch (const std::exception& e) {
            DebugUtils::logWarning("  " + rs_id + ": c2w parse error, using identity: "
                                   + std::string(e.what()));
        }

        // Populate the internal maps keyed by serial number
        camera_names[serial]     = rs_id;
        camera_transforms[serial] = transform;

        // std::stringstream tf_msg;
        // tf_msg << "  " << rs_id << " (serial: " << serial << ") loaded.\n"
        //        << "    c2w row0: ["
        //        << transform(0,0) << ", " << transform(0,1) << ", "
        //        << transform(0,2) << ", " << transform(0,3) << "]";
        // DebugUtils::logRS(tf_msg.str());

        loaded_count++;
    }

    if (loaded_count == 0) {
        DebugUtils::logError("No cameras were successfully loaded. "
                             "Check that camera_ids in moad_config match entries in "
                             "the calibration file.");
        return;
    }
    DebugUtils::logRS(std::to_string(loaded_count) + " / "
                      + std::to_string(id_to_serial.size())
                      + " cameras loaded from calibration.");

    // ── 4. Start device pipelines  ────────────────────────────────────────────
    // device_check() scans physically connected RS devices and starts a pipeline
    // for each one whose serial number is in camera_names.
    try {
        device_check();
    } catch (const rs2::error& e) {
        DebugUtils::logError("RealSense SDK error during device_check: "
                             + std::string(e.get_failed_function())
                             + "(" + std::string(e.get_failed_args()) + "): "
                             + std::string(e.what()));
    }

    // ── 5. Collect warm-up frames  ────────────────────────────────────────────
    // A short burst of discarded frames allows auto-exposure to settle before
    // actual data collection begins. Skipped if init_test_frames is 0.
    int test_frames = config.getValue<int>("realsense.init_test_frames");
    if (test_frames > 0) {
        DebugUtils::logRS("Collecting " + std::to_string(test_frames)
                          + " warm-up frames to settle auto-exposure...");
        get_frames(test_frames);
        DebugUtils::logRS("Warm-up complete.");
    } else {
        DebugUtils::logRS("Skipping warm-up frames (realsense.init_test_frames = 0).");
    }

    DebugUtils::logWhitespace();
}


// Checks for all connected realsense devices, starts a pipeline for each,
// and adds them all to the pipelines vector.
int RealSenseHandler::device_check() {
    // Get the list of connected devices
    auto devices_list = ctx.query_devices();
    device_count = devices_list.size();

    // check physical devices detected versus devices listed in moad_config.json
    DebugUtils::logRS("Detected " + std::to_string(device_count) + " RealSense devices.");
    int i = 1;
    std::stringstream msg_stream;
    for (auto&& dev : devices_list) {
        msg_stream.str("");
        msg_stream.clear();
        msg_stream << "[" << i << "]\t";
        if (dev.supports(RS2_CAMERA_INFO_NAME)) {
            msg_stream << dev.get_info(RS2_CAMERA_INFO_NAME) << "\t";
        }
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        // TODO: add to verbose flagging
        msg_stream << "Serial: " << serial;

        // check if this matches with one of the serial numbers in the moad_config 
        auto it = camera_names.find(serial);
        if (it != camera_names.end()) {
            msg_stream << "\n\t\t|- Matches with [" << it->second << "]:[" << it->first << "] from moad_config.json";
            start_device(serial);
        } else {
            msg_stream << "\n\t\t|- NO MATCH FOUND IN CONFIG, SKIPPING ";
        }

        DebugUtils::logRS(msg_stream.str());
        i++;
    }

    return device_count;
}

void RealSenseHandler::start_device(std::string serial_number) {
    // Define the configuration to use for each pipeline
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(serial_number);
    cfg.disable_all_streams();

    
    if (ConfigHandler::getInstance().getValue<bool>("realsense.high_res")) {
        cfg.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_RGB8,5);
        cfg.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,5);
        // TODO: add to verbose flagging
        // std::cout << "High resolution mode enabled for " << camera_names[serial_number] << std::endl;
    } else {
        cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,5);
        cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,5);
        // TODO: add to verbose flagging
        // std::cout << "Low resolution mode enabled for " << camera_names[serial_number] << std::endl;
    }

    // Start the stream
    auto r = pipe.start(cfg);


    // auto intr = r.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    // TODO: add to verbose flagging
    // cout << "Depth Intrinsics: [" << intr.fx << ", " << intr.fy << ", " << intr.ppx << ", " << intr.ppy << ", " << intr.model << "]" << endl;
    // cout << "[" << camera_names[serial_number] << "][DEVICE STARTED]\n" << endl;
    
    // Add the pipeline to the vector
    pipeline_map[serial_number] = pipe;

    // Separate thread method
    // std::thread frame_thread(&RealSenseHandler::frame_poll_thread, this,  pipe);
    // frame_thread_map[serial_number] = std::move(frame_thread);
}

void RealSenseHandler::frame_poll_thread(rs2::pipeline pipe) {
    std::string serial_number = pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    rs2::frameset fs;

    std::chrono::steady_clock::time_point start, end;
    std::chrono::milliseconds duration;
    
    start = std::chrono::steady_clock::now();

    while(running) {
        // Poll for frames
        fs = pipe.wait_for_frames();
        
        // Lock the mutex before accessing the shared variable
        std::unique_lock<std::mutex> lock(framesetMutex);
        frameset_map[serial_number] = fs;

        // Release the lock
        lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        end = std::chrono::steady_clock::now();

        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);        
        // int seconds = (duration.count() % 60000) / 1000;
        // int ms = duration.count() % 1000;
        // if (duration.count() > 200) {
        cout << "[" << camera_names[serial_number] << " - " << duration.count() << "ms]\n";
        // }
        start = end;
    }
    cout << camera_names[serial_number] << " thread closed.\n";
}

// Gets num_frames frames from all connected devices, but does nothing with them.
// Verifies proper communication and allows autoexposure to settle.
void RealSenseHandler::get_frames(int num_frames, int timeout_ms) {

    std::stringstream ss;
    ss  << "Getting " << num_frames << " frames from " << pipeline_map.size()
        << " devices:";
    DebugUtils::logRS(ss.str());

    // new_frames.clear();
    for (int i = 0 ; i < num_frames ; i++) {
        if (pipeline_map.size() == 0) break;
        // Iterate through the map
        for (const auto& pipe : pipeline_map) {
            rs2::frameset fs;
            fs = pipe.second.wait_for_frames(timeout_ms);
            cout << "-" << std::flush; // Visual marker for each collected frame
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }
    cout << endl;
    DebugUtils::logRS("Finished getting frames from all connected devices.");
}

// Collects relevant MOAD data from all RealSense devices
void RealSenseHandler::get_current_frame(int degree, int timeout_ms, ThreadPool* pool) {
    // ConfigHandler& config = ConfigHandler::getInstance();

    DebugUtils::logRS("Getting RealSense data at angle " + std::to_string(degree) + " degrees...");

    // Create a rotation matrix for the current turntable position
    DebugUtils::logRS("Getting rotation matrix for " + std::to_string(turntable_position) + " degrees...");
    rot_matrix = createRotationMatrix(turntable_position);

    // Check if the functon is being called with a thread pool
    if (pool == nullptr) {
        // Create vector of threads
        std::vector<std::thread> thread_vector;
        for (const auto& pipe : pipeline_map) {
            // Create a thread for each pipe
            thread_vector.emplace_back([&, pipe, timeout_ms, degree]() {
                process_frames(pipe.second, degree, timeout_ms);});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Join all threads
        for (auto& thread : thread_vector) {
            thread.join();
        }
    }
    else {
        for (const auto& pipe : pipeline_map) {
            // Enqueue a task for each pipe in the thread pool
            pool->enqueueTask([&, pipe, timeout_ms, degree]() {
                // DebugUtils::startTimer();
                process_frames(pipe.second, degree, timeout_ms);
                // DebugUtils::stopTimer("Processing frames for " + camera_names[pipe.first] + " at angle " + std::to_string(degree));
            });
        }
    }

    // cout << "\033[1;46m" << "Got frames from all RS at angle " << degree << ", Saving in the background..." << "\033[0m\n";
    DebugUtils::logRS("Got frames from all RealSense at angle " + std::to_string(degree) + ", saving in the background...");
}

/* ==============================================================================================================
Handles data aquisition and filtering for a single frame of one device. 
Normally this function is called as a separate thread within a threadpool for parallel data colleciton. 
Reads settings from the config to determine what data to collect, which filters to apply, etc.
============================================================================================================== */
void RealSenseHandler::process_frames(rs2::pipeline pipe, int degree, int timeout_ms) {
    ConfigHandler& config = ConfigHandler::getInstance();
    std::stringstream out_file;

    // Acquire thread ID string for printing...
    std::stringstream ss;
    ss << std::this_thread::get_id();
    std::string thr_id_str = ss.str();
    
    // Get serial number for this camera
    std::string serial_number = pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    // DebugUtils::logRS("Processing " + camera_names[serial_number] + " at angle " + std::to_string(degree) + "...");
    DebugUtils::logThread("[ID:"+thr_id_str+"] Processing " + camera_names[serial_number]+ " at angle " + std::to_string(degree) + "...");
    
    // ===========================================================
    // Collect frameset from camera 
    rs2::frameset fs;
    try {
        // Wait for frames
        fs = pipe.wait_for_frames(timeout_ms);
    } catch (const rs2::error& e) {
        fail_count++;
        DebugUtils::logWarning(camera_names[serial_number] + " did not get frames.");
        DebugUtils::logError(camera_names[serial_number] + ": RS error occurred: " + std::string(e.what()));
        return;
    } catch (const std::exception& ex) {
        DebugUtils::logError(camera_names[serial_number] + ": An error occurred: " + std::string(ex.what()));
    } catch (...) {
        DebugUtils::logError("Realsense ERROR: ????");
    }

    // fs = pipe.wait_for_frames(timeout_ms);
    if (fs.size() == 0) {
        DebugUtils::logWarning(camera_names[serial_number] + ": Did not get frames.");
        return;
    }

    // function to swap between data collection strategies (color/depth)
     if (config.getValue<bool>("realsense.align_to_color")) {
        // Align frames to color if enabled
        rs2::align align(RS2_STREAM_COLOR);
        try {
            fs = align.process(fs);
        } catch (const std::exception& ex) {
            std::cerr << camera_names[serial_number] << ": Error aligning frameset: " << ex.what() << std::endl;
            fs = pipe.wait_for_frames(timeout_ms);
            cout << camera_names[serial_number] << ": got another frameset\n";
            fs = align.process(fs);
            cout << camera_names[serial_number] << ": may have recovered.\n";
        }
     } else {
        // Align frames to depth
        rs2::align align(RS2_STREAM_DEPTH);
        try {
            fs = align.process(fs);
        } catch (const std::exception& ex) {
            std::cerr << camera_names[serial_number] << ": Error aligning frameset: " << ex.what() << std::endl;
            fs = pipe.wait_for_frames(timeout_ms);
            cout << camera_names[serial_number] << ": got another frameset\n";
            fs = align.process(fs);
            cout << camera_names[serial_number] << ": may have recovered.\n";
        }
     }

    // Get color and depth frames from the frameset
    rs2::video_frame color = fs.get_color_frame();
    rs2::depth_frame depth = fs.get_depth_frame();

    //Apply threshold filter to depth frame
    depth = threshold_filter.process(depth);
    depth = spatial_filter.process(depth);
    depth = temporal_filter.process(depth);

    // === COLOR IMAGE COLLECTION =====================================================================================================
    // Testing: Create the color/depth images before processing/filtering
    // Check if collecting color images is enabled
    // Convert the color frame to OpenCV Mat
    if (config.getValue<bool>("realsense.collect_color")) {
        cv::Mat color_mat(color.get_height(), color.get_width(), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);

        // Generate image name
        out_file.str("");

        // should not be harcoded as "/" - gs 10/29
        out_file << save_dir << "/" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_color.png";

        // Save the color image
        cv::imwrite(out_file.str(), color_mat);
        DebugUtils::logSaveLoop(camera_names[serial_number] + "(Color): "+ out_file.str());
    }

    // === DEPTH IMAGE COLLECTION =====================================================================================================
    // Check if collecting depth images is enabled
    if (config.getValue<bool>("realsense.collect_depth")) {
        // Convert the depth frame to OpenCV Mat
        cv::Mat depth_mat(cv::Size(depth.get_width(), depth.get_height()), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        if (config.getValue<bool>("realsense.normalize_depth_image")) {
            cv::normalize(depth_mat, depth_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        }

        // Generate image name
        out_file.str("");

        // should not be hardcoded as "/"
        out_file << save_dir << "/" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_depth.png";
        
        // Save the depth image
        cv::imwrite(out_file.str(), depth_mat);
        DebugUtils::logSaveLoop(camera_names[serial_number] + "(Depth): "+ out_file.str());
    }

    // this check must be outside of the below if statements and for loop!! - GS 8/22
    bool COLOR_ORDER_BGR = config.getValue<bool>("realsense.collect_color");


    // === POINT CLOUD COLLECTION =====================================================================================================
    // Check if collecting pointclouds is enabled
    if (config.getValue<bool>("realsense.collect_pointcloud")) {
        // [DEUG] Start Timer for pointcloud creation
        // DebugUtils::startTimer();
        // Create PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // Define the origin point (0, 0, 0, 1)
        Eigen::Vector4f origin(0.0f, 0.0f, 0.0f, 1.0f);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        // Get the number of points and vertices from the frame
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);
        const rs2::vertex* vertices = points.get_vertices();
        int numVertices = points.size();
        // cout << "numVertices: " << numVertices << " - ";
        int i;
        for (i = 0; i < numVertices ; i++) {
            // Create a point and set its coordinates
            pcl::PointXYZRGB point;
            point.x = vertices[i].x;
            point.y = vertices[i].y;
            point.z = vertices[i].z;


            // Get color from the corresponding pixel in the color frame
            // this ensures right color mapping otherwise they turn out with the wreong colors
            const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
            if (COLOR_ORDER_BGR) {
                // BGR format
                point.b = color_data[3 * i];
                point.g = color_data[3 * i + 1];
                point.r = color_data[3 * i + 2];
            } else {
                // RGB format
                point.r = color_data[3 * i];
                point.g = color_data[3 * i + 1];
                point.b = color_data[3 * i + 2];
            }

            // Add the point to the point cloud
            cloud->push_back(point);
        }
        // [DEBUG] Stop Timer for pointcloud creation
        std::stringstream message;
        message << "[" << degree << "]" << camera_names[serial_number] << " PointCloud Created";
        DebugUtils::logDebug(message.str(),3);
        // DebugUtils::stopTimer(message.str());

        // Check for raw pointcloud collection, if enabled, skip the rest of the processing
        if (!config.getValue<bool>("realsense.raw_pointcloud")) {
            // Apply appropriate pointcloud transforms
            pcl::transformPointCloud(*cloud, *cloud, camera_transforms[serial_number]);
            pcl::transformPointCloud(*cloud, *cloud, rot_matrix);
            
            // Camera origin necessary for normal estimation #TODO: Ensure this works with new transform code
            origin = camera_transforms[serial_number] * origin;
            origin = rot_matrix * origin;

            //Apply passthrough filters to remove background
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            float fmin, fmax;

            if (config.getValue<bool>("realsense.filter.xpass.apply") || 
                config.getValue<bool>("realsense.filter.ypass.apply") || 
                config.getValue<bool>("realsense.filter.zpass.apply")){
                message.str("");
                message << "[" << degree << "]" << camera_names[serial_number] << " entering crop block.";
                DebugUtils::logDebug(message.str(),3);
            }
            
            // Check if xpass is enabled
            if (config.getValue<bool>("realsense.filter.xpass.apply")) {
                // [DEBUG] Start Timer for x-pass filter
                // DebugUtils::startTimer();
                
                // Get the min and max values for the x-pass filter
                fmin = config.getValue<float>("realsense.filter.xpass.min");
                fmax = config.getValue<float>("realsense.filter.xpass.max");
                
                // Set the values to the pass filter
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(fmin, fmax);

                // Apply the filter to the point cloud
                pass.filter(*cloud);

                // [DEBUG] Stop Timer for x-pass filter
                std::stringstream x_pass_message;
                x_pass_message << "[" << degree << "]" << camera_names[serial_number] << " X-pass Filter";
                // DebugUtils::stopTimer(x_pass_message.str());
            }

            // Check if ypass is enabled
            if (config.getValue<bool>("realsense.filter.ypass.apply")) {
                // [DEBUG] Start Timer for y-pass filter
                // DebugUtils::startTimer();
                
                // Get the min and max values for the y-pass filter
                fmin = config.getValue<float>("realsense.filter.ypass.min");
                fmax = config.getValue<float>("realsense.filter.ypass.max");

                // Set the values to the pass filter
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(fmin, fmax);

                // Apply the filter to the point cloud
                pass.filter(*cloud);
                
                // [DEBUG] Stop Timer for y-pass filter
                std::stringstream y_pass_message;
                y_pass_message << "[" << degree << "]" << camera_names[serial_number] << " Y-pass Filter";
                // DebugUtils::stopTimer(y_pass_message.str());
            }

            // Check if zpass is enabled
            if (config.getValue<bool>("realsense.filter.zpass.apply")) {
                // [DEBUG] Start Timer for z-pass filter
                // DebugUtils::startTimer();
                
                // Get the min and max values for the z-pass filter
                fmin = config.getValue<float>("realsense.filter.zpass.min");
                fmax = config.getValue<float>("realsense.filter.zpass.max");

                // Set the values to the pass filter
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(fmin, fmax);

                // Apply the filter to the point cloud
                pass.filter(*cloud);

                // [DEBUG] Stop Timer for z-pass filter
                std::stringstream z_pass_message;
                z_pass_message << "[" << degree << "]" << camera_names[serial_number] << " Z-pass Filter";
                // DebugUtils::stopTimer(z_pass_message.str());
            }
            
            // Check if statistical outlier removal (SOR) is enabled
            if (config.getValue<bool>("realsense.filter.sor.apply")) {
                // [DEBUG] Start Timer for SOR filter
                // DebugUtils::startTimer();
                message.str("");
                message << "[" << degree << "]" << camera_names[serial_number] << " entering SOR block.";
                DebugUtils::logDebug(message.str(),3);

                // Create a StatisticalOutlierRemoval filter
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor(true);

                // Get the standard deviation threshold and number of neighbors from the config
                fmin = config.getValue<float>("realsense.filter.sor.stddev");
                int k = config.getValue<int>("realsense.filter.sor.k");

                // Set the parameters for the SOR filter
                sor.setInputCloud(cloud);
                sor.setMeanK(k);  // Number of neighbors to use for mean distance estimation
                sor.setStddevMulThresh(fmin);  // Standard deviation threshold for outlier detection
                
                // Apply the filter to the point cloud
                sor.filter(*cloud);

                // [DEBUG] Stop Timer for SOR filter
                std::stringstream sor_message;
                sor_message << "[" << degree << "]" << camera_names[serial_number] << " SOR Filter";
                // DebugUtils::stopTimer(sor_message.str());
            }

            // Check if voxel grid filter is enabled
            if (config.getValue<bool>("realsense.filter.voxel.apply")) {
                // [DEBUG] Start Timer for voxel grid filter
                // DebugUtils::startTimer();
                message.str("");
                message << "[" << degree << "]" << camera_names[serial_number] << " entering Voxel block.";
                DebugUtils::logDebug(message.str(),3);

                // Create a VoxelGrid filter
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;

                // Get the leaf size from the config
                fmin = config.getValue<float>("realsense.filter.voxel.leaf_size");

                // Set the parameters for the voxel grid filter
                voxel_grid_filter.setInputCloud(cloud);
                voxel_grid_filter.setLeafSize(fmin, fmin, fmin); // Adjust the values as per your needs

                // Apply the filter to the point cloud
                voxel_grid_filter.filter(*cloud);

                // [DEBUG] Stop Timer for voxel grid filter
                std::stringstream voxel_message;
                voxel_message << "[" << degree << "]" << camera_names[serial_number] << " Voxel Filter";
                // DebugUtils::stopTimer(voxel_message.str());
            }
        }

        // Check if computing normals is enabled
        if (config.getValue<bool>("realsense.compute_normals")) {
            // [DEBUG] Start Timer for normals computation
            // DebugUtils::startTimer();
            message.str("");
            message << "[" << degree << "]" << camera_names[serial_number] << " entering Normals block.";
            DebugUtils::logDebug(message.str(),3);

            // Create a pcl::PointCloud<pcl::Normal> to hold the normals
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;

            // Set the input cloud and parameters for normal estimation
            ne.setInputCloud(cloud);

            // Set the number of threads for parallel processing
            ne.setNumberOfThreads(config.getValue<int>("realsense.normals_threads"));
            ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
            ne.setKSearch(2);
            ne.setViewPoint(origin[0],origin[1],origin[2]);
            
            // Compute the normals
            ne.compute(*normals);

            // Concatenate the original point cloud and the computed normals
            pcl::concatenateFields(*cloud, *normals, *normal_cloud);
            
            // [DEBUG] Stop Timer for normals computation
            std::stringstream normals_message;
            normals_message << "[" << degree << "]" << camera_names[serial_number] << " Normals Computation";
            // DebugUtils::stopTimer(normals_message.str());
        }
        
        // Generate pointcloud name and save
        out_file.str("");

        // TODO: should not be hardcoded as "/"
        out_file << save_dir << "/" << camera_names[serial_number] << "_"
             << std::setfill('0') << std::setw(3) << degree << "_cloud.ply";
        std::cout.copyfmt(std::ios(nullptr));

        // Set the locale to "C" to avoid issues with decimal point formatting
        std::locale::global(std::locale("C"));

        // Check if normals are computed to save the appropriate point cloud
        message.str("");
        message << "[" << degree << "]" << camera_names[serial_number] << " entering save block.";
        DebugUtils::logDebug(message.str(),3);
        if (!config.getValue<bool>("realsense.compute_normals")) {
            // Save the point cloud without normals as binary PLY
            pcl::io::savePLYFile(out_file.str(), *cloud, true);
            DebugUtils::logSaveLoop(camera_names[serial_number] + "(Cloud): "+ out_file.str());
        } else {
            // Save the point cloud with normals as binary PLY
            pcl::io::savePLYFile(out_file.str(), *normal_cloud, true);
            DebugUtils::logSaveLoop(camera_names[serial_number] + "(Cloud+N): "+ out_file.str());
        }
    }
}

/* Prints out RealSense device information, if print_streams is true, 
it will print all available stream formats (it's a lot of text)*/
void RealSenseHandler::print_device(rs2::device dev, bool print_streams) {
    cout << "RealSense Device: " << endl;
    cout << "  Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << endl;
    cout << "  Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;
    cout << "  Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << endl;
    // Get the list of available streams
    if (print_streams) {
        cout << "  Available streams:" << endl;
        for (const rs2::sensor& sensor : dev.query_sensors()) {
            cout << "\n  Sensor: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << endl;
            int i = 1;
            for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
                rs2_stream stream_type = profile.stream_type();
                int stream_width = 0, stream_height = 0;
                int stream_fps = 0;
                rs2_format stream_format = RS2_FORMAT_ANY;

                if (auto video_profile = profile.as<rs2::video_stream_profile>())
                {
                    stream_width = video_profile.width();
                    stream_height = video_profile.height();
                    stream_fps = video_profile.fps();
                    stream_format = video_profile.format();
                }

                cout << "[" << rs2_stream_to_string(stream_type) << " : " << stream_width << "x" << stream_height << " : "
                    << stream_fps << " : " << rs2_format_to_string(stream_format) << "] ";
                if (i % 5 == 0) cout << endl;
                i++;
            }
        }
    }
}