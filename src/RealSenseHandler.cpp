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
    Initialize the RealSense Handler
        args: none
        returns: void

    TODO: The config path is redundant, and comes from when the RS calibration was held
    in a different calibration file. I'll keep this implementation for now instead of using the 
    config directly, because I might actually want to put the Realsense transforms back in a 
    separate calibration file later, as it takes up a lot of space in the moad_config.
*/
void RealSenseHandler::initialize(std::string config_path) {

    ConfigHandler& config = ConfigHandler::getInstance();
    if (!config.getValue<bool>("realsense.enable_collection")) {
        DebugUtils::logRS("Skipping RealSense setup, (realsense.enable_collection=false in config).");
        DebugUtils::logWhitespace();
        return;
    }
    
    DebugUtils::logInfo("Initializing RealSense Cameras...");


    // Get Realsense camera transforms from JSON file
    nlohmann::json realsense_json;

    DebugUtils::logDebug("Loading config to find RS info: " + config_path);
    std::ifstream realsense_file(config_path);

    realsense_json = nlohmann::json::parse(realsense_file);

    // If the file wraps the camera entries under a "realsense" object, descend into it.
    // After this, realsense_json will reference the object that contains "rs1", "rs2", ...
    // Descend into realsense.rs_info.rs1, rs2 ...
    if (realsense_json.contains("realsense") && realsense_json["realsense"].is_object()) {
        auto &rs_root = realsense_json["realsense"];
        if (rs_root.contains("rs_info") && rs_root["rs_info"].is_object()) {
            realsense_json = rs_root["rs_info"];
            DebugUtils::logDebug("RealSense mapping info found in config. (ID: [serial, transform])");
            // std::cout << "Using nested 'realsense.rs_info' object from JSON.\n";
        } else {
            // Fallback to older structure realsense->{ "rs1":..., ... }
            realsense_json = rs_root;
            // std::cout << "Using nested 'realsense' object from JSON (no rs_info child).\n";
        }
    }
    else {
        DebugUtils::logWarning("No RealSense IDs were found in " + config_path);
    }

    // Quick sanity check: warn if no device-like keys were found (keys starting with "rs")
    bool found_rs_entry = false;
    for (auto it = realsense_json.begin(); it != realsense_json.end(); ++it) {
        if (!it.key().empty() && it.key().rfind("rs", 0) == 0) { // starts with "rs"
            found_rs_entry = true;
            break;
        }
    }
    if (!found_rs_entry) {
        DebugUtils::logError("Warning: no 'rs*' entries found in realsense JSON. Check file structure: expected realsense.rs_info->{\"rs1\":..., ...} or realsense->{\"rs1\":..., ...}.");
    }


    // create map of id -> (serial, transform_matrix)
    // Example JSON entry expected:
    // "rs1": {
    //   "serial": "215122257111",
    //   "transform_matrix": [[...],[...],[...],[...]]
    // }
    std::map<std::string, std::pair<std::string, Eigen::Matrix4f>> camera_info_map;
    for (auto it = realsense_json.begin(); it != realsense_json.end(); ++it) {

        const std::string id = it.key(); // will be "rs1/2/3 ..."
        const auto &val = it.value(); // used for iterating "serial" and "transform_matrix"

        // std::cout << "id " << id << std::endl;

        // read serial, should be string but checks incase not
        std::string serial;
        if (val.contains("serial")) {
            if (val["serial"].is_string()) {
                serial = val["serial"].get<std::string>();
            } else if (val["serial"].is_number_integer()) {
                serial = std::to_string(val["serial"].get<long long>());
            } else {
                serial = val["serial"].dump();
            }
        }

        // must populate camera_names with serial:id
        camera_names[serial] = id; 

        // read transform matrix, default to identity
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        if (val.contains("transform_matrix")) {
            try {
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        transform(r, c) = val["transform_matrix"][r][c].get<float>();
                    }
                }
            } catch (...) {
                // keep identity on parse error
            }
        }

        // must populate camera_transforms with serial:matrix
        camera_transforms[serial] = transform;

        // reminder, this map is mainly to consolidate all info together
        // TODO: definitely swap everything to use subslices of this map rather
        //      than the two camera_names/camera_transforms maps
        camera_info_map[id] = std::make_pair(serial, transform);
    }
    
    // Check available devices and start data pipelines for each one
    try {
        device_check();
    }
    catch(const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n " << e.what() << endl;
    }

    // After starting all devices, collect test frames from each device.
    // This MAY not be necessary, but I believe it helps to settle the autoexposure
    // TODO: Test this ^
    // Get some frames to settle auto-exposure and verify stream.
    int test_frames = config.getValue<int>("realsense.init_test_frames");
    if (test_frames > 0){
        // DebugUtils::logRS("Getting " + std::to_string(test_frames) + " frames to verify data & settle autoexposure...");
        get_frames(test_frames); // make 30 later
        // DebugUtils::logRS("Success.");
    }else{
        DebugUtils::logRS("Skipping frame stream test... (Config: realsense/init_test_frames)");
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
                DebugUtils::startTimer();
                process_frames(pipe.second, degree, timeout_ms);
                DebugUtils::stopTimer("Processing frames for " + camera_names[pipe.first] + " at angle " + std::to_string(degree));
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
        DebugUtils::startTimer();
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
        DebugUtils::stopTimer(message.str());

        // Check for raw pointcloud collection, if enabled, skip the rest of the processing
        if (!config.getValue<bool>("realsense.raw_pointcloud")) {
            //Apply appropriate pointcloud transform
            pcl::transformPointCloud(*cloud, *cloud, camera_transforms[serial_number]);
            pcl::transformPointCloud(*cloud, *cloud, rot_matrix);
            // pcl::transformPointCloud(*normal_cloud, *normal_cloud, camera_transforms[serial_number]);
            // pcl::transformPointCloud(*normal_cloud, *normal_cloud, rot_matrix);
            origin = camera_transforms[serial_number] * origin;
            origin = rot_matrix * origin;
            //Apply passthrough filters to remove background
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            float fmin, fmax;

            // Check if xpass is enabled
            if (config.getValue<bool>("realsense.filter.xpass.apply")) {
                // [DEBUG] Start Timer for x-pass filter
                DebugUtils::startTimer();
                
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
                DebugUtils::stopTimer(x_pass_message.str());
            }

            // Check if ypass is enabled
            if (config.getValue<bool>("realsense.filter.ypass.apply")) {
                // [DEBUG] Start Timer for y-pass filter
                DebugUtils::startTimer();
                
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
                DebugUtils::stopTimer(y_pass_message.str());
            }

            // Check if zpass is enabled
            if (config.getValue<bool>("realsense.filter.zpass.apply")) {
                // [DEBUG] Start Timer for z-pass filter
                DebugUtils::startTimer();
                
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
                DebugUtils::stopTimer(z_pass_message.str());
            }
            
            // Check if statistical outlier removal (SOR) is enabled
            if (config.getValue<bool>("realsense.filter.sor.apply")) {
                // [DEBUG] Start Timer for SOR filter
                DebugUtils::startTimer();

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
                DebugUtils::stopTimer(sor_message.str());
            }

            // Check if voxel grid filter is enabled
            if (config.getValue<bool>("realsense.filter.voxel.apply")) {
                // [DEBUG] Start Timer for voxel grid filter
                DebugUtils::startTimer();

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
                DebugUtils::stopTimer(voxel_message.str());
            }
        }

        // Check if computing normals is enabled
        if (config.getValue<bool>("realsense.compute_normals")) {
            // [DEBUG] Start Timer for normals computation
            DebugUtils::startTimer();

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
            DebugUtils::stopTimer(normals_message.str());
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