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

#include "ConfigHandler.h"
#include "RealSenseHandler.h"

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
    std::cout << "Shutting down RealSense Handler... ";
    running = false;
    for (auto& pipe : pipeline_map) {
            pipe.second.stop();
            cout << ". ";
    }
    // for (auto& th : frame_thread_map) {
    //         th.second.join();
    // }
    std::cout << "Done.\n";
}

void RealSenseHandler::initialize() {
    std::cout << "Initializing RealSense Handler...\n";
    ConfigHandler& config = ConfigHandler::getInstance();

    // Get Realsense camera transforms from JSON file
    nlohmann::json realsense_json;
    std::string transform_dir = config.getValue<std::string>("realsense.transform_path");
    std::cout << "Loading Realsense camera transforms...\n";
    std::string transform_file = config.getValue<std::string>("realsense.transform_file");
    std::ifstream realsense_file(transform_dir + transform_file);
    realsense_json = nlohmann::json::parse(realsense_file);
    std::cout << "Realsense camera transforms loaded.\n";
    for (auto& [key, value] : realsense_json.items()) {
        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform_matrix(i, j) = value["transform_matrix"][i][j].get<float>();
            }
        }
        camera_names[key] = value["name"].get<std::string>();
        camera_transforms[key] = transform_matrix;
    }

    // Find all RS cameras and open streams
    try {
        device_check();
    }
    catch(const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n " << e.what() << endl;
    }
}


// Checks for all connected realsense devices, starts a pipeline for each,
// and adds them all to the pipelines vector.
int RealSenseHandler::device_check() {
    auto devices_list = ctx.query_devices();
    device_count = devices_list.size();
    cout << device_count << " RealSense detected.\n";
    
    for (auto&& dev : devices_list) {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        cout << "[" << camera_names[serial] << "] " << serial << endl;
    }
    for (auto&& dev : devices_list) {
        // Print Device Information
        bool print_available_streams = false;
        print_device(dev,print_available_streams);
        // Start device data streams
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        start_device(serial);
    }

    return device_count;
}

void RealSenseHandler::start_device(std::string serial_number) {
    // Define the configuration to use for each pipeline
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(serial_number);
    cfg.disable_all_streams();
    // cfg.enable_stream(RS2_STREAM_COLOR,640,360,RS2_FORMAT_RGB8,5); 
    // cfg.enable_stream(RS2_STREAM_DEPTH,640,360,RS2_FORMAT_Z16,5);  
    cfg.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_RGB8,5);
    cfg.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,5); 	

    // Start the stream
    pipe.start(cfg);
    cout << "[" << camera_names[serial_number] << "][DEVICE STARTED]\n" << endl;
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
    start = std::chrono::high_resolution_clock::now();
    while(running) {
        // Poll for frames
        fs = pipe.wait_for_frames();
        // Lock the mutex before accessing the shared variable
        std::unique_lock<std::mutex> lock(framesetMutex);
        // Update the shared variable with the latest frameset
        frameset_map[serial_number] = fs;
        // Release the lock
        lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        end = std::chrono::high_resolution_clock::now();
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
    // cout << "Getting " << num_frames << " frames from " << pipelines.size()
    //     << " devices:\n";
    cout << "Getting " << num_frames << " frames from " << pipeline_map.size()
        << " devices:\n";
    // new_frames.clear();
    for (int i = 0 ; i < num_frames ; i++) {
        if (pipeline_map.size() == 0) break;
        // Iterate through the map
        for (const auto& pipe : pipeline_map) {
            rs2::frameset fs;
            fs = pipe.second.wait_for_frames(timeout_ms);
            cout << "- ";
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        // if (frameset_map.size() == 0) break;
        // else cout << "FS Map Size: " << frameset_map.size() << endl;
        // Iterate through the map
        // std::unique_lock<std::mutex> lock(framesetMutex);
        // for (const auto& fs : frameset_map) {
            // rs2::frameset fs;
            // fs = pipe.second.wait_for_frames(timeout_ms);
            // cout << "- ";
        // }
        // lock.unlock();
        cout << endl;
    }
    cout << " [DONE]\n";
}

// Collects relevant MOAD data from all RealSense devices
void RealSenseHandler::get_current_frame(int timeout_ms, ThreadPool* pool) {
    cout << "\nGetting RealSense Data... \n";
    cout << "Getting rotation matrix for " << turntable_position << " degress...\n";
    rot_matrix = createRotationMatrix(turntable_position);
    cout << rot_matrix << endl << endl;

    if (pool == nullptr) {
        std::vector<std::thread> thread_vector;
        for (const auto& pipe : pipeline_map) {
            thread_vector.emplace_back([&, pipe, timeout_ms]() {
                process_frames(pipe.second, timeout_ms);});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    
        for (auto& thread : thread_vector) {
            thread.join();
        }
    }
    else {
        for (const auto& pipe : pipeline_map) {
            pool->enqueueTask([&, pipe, timeout_ms]() {
                process_frames(pipe.second, timeout_ms);});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    cout << "Got frames from all RS\n";
}

void RealSenseHandler::process_frames(rs2::pipeline pipe, int timeout_ms) {
    ConfigHandler& config = ConfigHandler::getInstance();
    std::stringstream out_file;
    
    // Get serial number for this camera
    std::string serial_number = pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "Processing " << camera_names[serial_number] << "...\n";
    
    // Collect frameset from camera
    rs2::frameset fs;
    try {
        // Wait for frames
        fs = pipe.wait_for_frames(timeout_ms);
    } catch (const rs2::error& e) {
        fail_count++;
        std::cerr << camera_names[serial_number] << ": RS error occurred: " << e.what() << std::endl;
        cout << "WARNING: " << camera_names[serial_number] << " did not get frames.\n";
        return;
    } catch (const std::exception& ex) {
        std::cerr << camera_names[serial_number] << ": An error occurred: " << ex.what() << std::endl;
    } catch (...) {
        cout << "ERROR: ????\n";
    }

    // fs = pipe.wait_for_frames(timeout_ms);
    if (fs.size() == 0) {
        cout << "WARNING: " << camera_names[serial_number] << " did not get frames.\n";
        return;
    }

    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    try {
        fs = align_to_depth.process(fs);
    } catch (const std::exception& ex) {
        std::cerr << camera_names[serial_number] << ": Error aligning frameset: " << ex.what() << std::endl;
        fs = pipe.wait_for_frames(timeout_ms);
        cout << camera_names[serial_number] << ": got another frameset\n";
        fs = align_to_depth.process(fs);
        cout << camera_names[serial_number] << ": may have recovered.\n";
    }

    rs2::video_frame color = fs.get_color_frame();
    rs2::depth_frame depth = fs.get_depth_frame();
    
    // At this point, we should throw the rest of the process in a thread pool 

    // Apply filters to depth frame
    depth = threshold_filter.process(depth);
    depth = spatial_filter.process(depth);
    depth = temporal_filter.process(depth);

    // Test time of saving the pointcloud
    // SAVE POINTCLOUD
    if (config.getValue<bool>("realsense.collect_pointcloud")) {
        auto start = std::chrono::high_resolution_clock::now();
        // Create PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // Define the origin point (0, 0, 0, 1)
        Eigen::Vector4f origin(0.0f, 0.0f, 0.0f, 1.0f);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);
        const rs2::vertex* vertices = points.get_vertices();
        int numVertices = points.size();
        // cout << "numVertices: " << numVertices << " - ";
        int i;
        for (i = 0; i < numVertices ; i++) {
            pcl::PointXYZRGB point;
            point.x = vertices[i].x;
            point.y = vertices[i].y;
            point.z = vertices[i].z;

            // Get color from the corresponding pixel in the color frame
            const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
            point.r = color_data[3 * i];
            point.g = color_data[3 * i + 1];
            point.b = color_data[3 * i + 2];
        
            cloud->push_back(point);
        }
        // Check the time taken to create the pointcloud
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        cout << camera_names[serial_number] << ": Pointcloud created in " << duration.count() << "ms\n";
        // Output 'rsx:C' indicates the pointcloud object has been constructed.
        // cout << "[" << camera_names[serial_number] << ":C]";

        // Don't apply transforms and filters if raw_pointclouds is true
        if (!config.getValue<bool>("realsense.raw_pointcloud")) {
            //Apply appropriate pointcloud transform
            pcl::transformPointCloud(*cloud, *cloud, camera_transforms[serial_number]);
            pcl::transformPointCloud(*cloud, *cloud, rot_matrix);
            // pcl::transformPointCloud(*normal_cloud, *normal_cloud, camera_transforms[serial_number]);
            // pcl::transformPointCloud(*normal_cloud, *normal_cloud, rot_matrix);
            origin = camera_transforms[serial_number] * origin;
            origin = rot_matrix * origin;
            // cout << "[" << camera_names[serial_number] << ":Torigin:" << origin.transpose() << "]";
            // Output 'rsx:T' indicates the appropriate pointcloud transforms have been performed.
            // cout << "[" << camera_names[serial_number] << ":T]";
            //Apply passthrough filters to remove background
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            float fmin, fmax;
            // If rs_xpass=1, apply x-pass filter
            start = std::chrono::high_resolution_clock::now();
            if (config.getValue<bool>("realsense.filter.xpass.apply")) {
                fmin = config.getValue<float>("realsense.filter.xpass.min");
                fmax = config.getValue<float>("realsense.filter.xpass.max");
                // cout << "[" << camera_names[serial_number] << ":X:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            //  Check time for the x-pass filter
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": X-pass filter applied in " << duration.count() << "ms\n";

            // If rs_ypass=1, apply y-pass filter
            start = std::chrono::high_resolution_clock::now();
            if (config.getValue<bool>("realsense.filter.ypass.apply")) {
                fmin = config.getValue<float>("realsense.filter.ypass.min");
                fmax = config.getValue<float>("realsense.filter.ypass.max");
                // cout << "[" << camera_names[serial_number] << ":Y:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            //  Check time for the y-pass filter
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": Y-pass filter applied in " << duration.count() << "ms\n";

            // If rs_zpass=1, apply z-pass filter
            start = std::chrono::high_resolution_clock::now();
            if (config.getValue<bool>("realsense.filter.zpass.apply")) {
                fmin = config.getValue<float>("realsense.filter.zpass.min");
                fmax = config.getValue<float>("realsense.filter.zpass.max");
                // cout << "[" << camera_names[serial_number] << ":Z:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            //  Check time for the z-pass filter
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": Z-pass filter applied in " << duration.count() << "ms\n";

            // If rs_sor=1, apply Statistical outlier removal filter
            start = std::chrono::high_resolution_clock::now();
            if (config.getValue<bool>("realsense.filter.sor.apply")) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor(true);
                fmin = config.getValue<float>("realsense.filter.sor.stddev");
                int k = config.getValue<int>("realsense.filter.sor.k");
                sor.setInputCloud(cloud);
                sor.setMeanK(k);  // Number of neighbors to use for mean distance estimation
                sor.setStddevMulThresh(fmin);  // Standard deviation threshold for outlier detection
                sor.filter(*cloud);
                int removed = sor.getRemovedIndices()->size();
                // cout << "[" << camera_names[serial_number] << ":SOR:" << removed << " points]";
            }
            //  Check time for the SOR filter
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": SOR filter applied in " << duration.count() << "ms\n";

            // If rs_voxel=1, apply Voxel Filter with rs_voxel_leafsize parameter
            start = std::chrono::high_resolution_clock::now();
            if (config.getValue<bool>("realsense.filter.voxel.apply")) {
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
                fmin = config.getValue<float>("realsense.filter.voxel.leaf_size");
                voxel_grid_filter.setInputCloud(cloud);
                voxel_grid_filter.setLeafSize(fmin, fmin, fmin); // Adjust the values as per your needs
                voxel_grid_filter.filter(*cloud);
                // cout << "[" << camera_names[serial_number] << ":VOX:" << fmin << "]";
            }
            //  Check time for the Voxel filter
            stop = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": Voxel filter applied in " << duration.count() << "ms\n";

            // Output 'rsx:F' indicates the pointcloud has been filtered.
            // cout << "[" << camera_names[serial_number] << ":F]";
        }

        // Compute Normals
        if (config.getValue<bool>("realsense.compute_normals")) {
            // Create a NormalEstimation object
            // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            // ne.setInputCloud(cloud);
            // Set the number of neighbors to be considered when estimating normals
            // ne.setKSearch(4);
            // Compute the normals
            auto start = std::chrono::high_resolution_clock::now();
            
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
            ne.setKSearch(4);
            ne.setViewPoint(origin[0],origin[1],origin[2]);
            // cout << "[" << camera_names[serial_number] << ":compN]";
            ne.compute(*normals);
            // Concatenate the original point cloud and the computed normals
            // cout << "[" << camera_names[serial_number] << ":catN]";
            pcl::concatenateFields(*cloud, *normals, *normal_cloud);

            // Check the time taken to compute the normals
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            cout << camera_names[serial_number] << ": Normals computed in " << duration.count() << "ms\n";

            // Output 'rsx:N' indicates the normals have been calculated  
            // cout << "[" << camera_names[serial_number] << ":N]";
        }
        
        // Generate pointcloud name and save
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_cloud.ply";
        // Save the point cloud to a PLY file
        // Output 'rsx:S' indicates the pointcloud is now being saved.
        // cout << "[" << camera_names[serial_number] << ":S]";
        std::cout.copyfmt(std::ios(nullptr));
        std::locale::global(std::locale("C"));
        if (!config.getValue<bool>("realsense.compute_normals")) {
            pcl::io::savePLYFile(out_file.str(), *cloud, true);
        } else {
            pcl::io::savePLYFile(out_file.str(), *normal_cloud, true);
        }
        // pcl::io::savePLYFile(out_file.str(), *normal_cloud);
        cout << "[" << camera_names[serial_number] << ":SAVED]\n";
    }
    
    // SAVE COLOR IMAGE
    if (config.getValue<bool>("realsense.collect_color")) {
        cv::Mat color_mat(color.get_height(), color.get_width(), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);

        // Generate image name and save depth_mat
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_color.png";
        cv::imwrite(out_file.str(), color_mat);
    }

    // SAVE DEPTH IMAGE
    if (config.getValue<bool>("realsense.collect_depth")) {
        cv::Mat depth_mat(cv::Size(depth.get_width(), depth.get_height()), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::normalize(depth_mat, depth_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        
        // Generate image name and save depth_mat
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_depth.png";
        cv::imwrite(out_file.str(), depth_mat);
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