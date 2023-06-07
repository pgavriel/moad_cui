#include <iostream>
#include <thread>
#include <functional>
#include <string>
#include <sstream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "RealSenseHandler.h"


using std::string;
using std::cout;
using std::endl;

/* TODO: 
    - Find out how to automatically determine sensor position w calibration
        image, and add that to the rs2::config based on serial number
    - Find optimal stream configuration (resolutions, fps)
*/

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
    // Assign camera names based on serial numbers
    camera_names["215122257111"] = "rs1";
    camera_names["239222302632"] = "rs2";
    camera_names["213522251284"] = "rs3";
    camera_names["213622251144"] = "rs4";
    camera_names["215122254603"] = "rs5";
    
    // Assign camera transforms to corresponding serial number
    Eigen::Matrix4f rs1_tx {
        {0.998133, -0.057994, 0.019142, -0.079213},
        {-0.023726, -0.079419, 0.996559, -0.764634},
        {-0.056274, -0.995153, -0.080647, 0.071262},
        {0.000000, 0.000000, 0.000000, 1.000000}  };
    camera_transforms["215122257111"] = rs1_tx;
    Eigen::Matrix4f rs2_tx {
        {0.996240, -0.033589, 0.079854, -0.082825},
        {-0.086597, -0.412081, 0.907023, -0.729931},
        {0.002440, -0.910528, -0.413440, 0.372486},
        {0.000000, 0.000000, 0.000000, 1.000000}  };
    camera_transforms["239222302632"] = rs2_tx;
    Eigen::Matrix4f rs3_tx {
        {0.999344, -0.029679, 0.020743, -0.087933},
        {-0.035898, -0.737118, 0.674810, -0.574988},
        {-0.004738, -0.675112, -0.737700, 0.649989},
        {0.000000, 0.000000, 0.000000, 1.000000}  };
    camera_transforms["213522251284"] = rs3_tx;
    Eigen::Matrix4f rs4_tx {
        {0.998020, -0.052458, 0.034693, -0.071821},
        {-0.061411, -0.931829, 0.357664, -0.331198},
        {0.013565, -0.359087, -0.933206, 0.814315},
        {0.000000, 0.000000, 0.000000, 1.000000}  };
    camera_transforms["213622251144"] = rs4_tx;
    Eigen::Matrix4f rs5_tx {
        {0.999721, -0.010826, 0.020993, -0.082966},
        {-0.010539, -0.999850, -0.013728, -0.036477},
        {0.021138, 0.013503, -0.999685, 0.850775},
        {0.000000, 0.000000, 0.000000, 1.000000}  };
    camera_transforms["215122254603"] = rs5_tx;

    //Configure Depth Frame Filters
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.2f); // Minimum threshold distance in meters
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.0f); // Maximum threshold distance in meters
    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // Decimation magnitude
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f); // Spatial filter smooth alpha
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); // Spatial filter smooth delta
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // Spatial filter magnitude
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f); // Temporal filter smooth alpha
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f); // Temporal filter smooth delta

    //Create Output folder
    // save_dir = "C:\\Users\\csrobot\\Documents\\Version13.16.01\\Output";
    // if (CreateDirectory(save_dir.c_str(), NULL) ||
    //     ERROR_ALREADY_EXISTS == GetLastError()) {
    //     cout << "RealSense using output dir: "<<save_dir << endl;
    // }
    // else {
    //    std::cerr << "Failed to create RealSense output directory: "
    //     << save_dir << endl;
    // }
}

RealSenseHandler::~RealSenseHandler() {
    std::cout << "Shutting down RealSense Handler... ";
    for (auto &&pipe : pipelines) {
            pipe.stop();
    }
    std::cout << "Done.\n";
}

void RealSenseHandler::initialize() {
    // Find all RS cameras and open streams
    try {
        device_check();
    }
    catch(const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n " << e.what() << endl;
    }
}

void RealSenseHandler::update_config(std::map<std::string, std::string>& cfg) {
    config = cfg;
    save_depth_img = bool(stoi(config["rs_depth"]));
    save_color_img = bool(stoi(config["rs_color"]));
    save_pointcloud = bool(stoi(config["rs_pointcloud"]));
    raw_pointclouds = bool(stoi(config["rs_raw_pointcloud"]));
}


// Checks for all connected realsense devices, starts a pipeline for each,
// and adds them all to the pipelines vector.
int RealSenseHandler::device_check() {
    auto devices_list = ctx.query_devices();
    device_count = devices_list.size();
    cout << device_count << " RealSense detected.\n";
    int c = 1;
    for (auto&& dev : devices_list) {
        // Print Device Information
        bool print_available_streams = false;
        print_device(dev,print_available_streams);
        
        // Define the configuration to use for each pipeline
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        cfg.enable_stream(RS2_STREAM_COLOR,640,360,RS2_FORMAT_RGB8,5); 
        // cfg.enable_stream(RS2_STREAM_DEPTH,640,360,RS2_FORMAT_Z16,5);  
        // cfg.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_RGB8,5);
        cfg.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,5); 	

        // Start the stream
        pipe.start(cfg);
        cout << "[DEVICE STARTED]\n" << endl;
        // Add the pipeline to the vector
        pipelines.emplace_back(pipe);
        c++;
    }
    return device_count;
}

// Gets num_frames frames from all connected devices, but does nothing with them.
// Verifies proper communication and allows autoexposure to settle.
void RealSenseHandler::get_frames(int num_frames, int timeout_ms) {
    cout << "Getting " << num_frames << " frames from " << pipelines.size()
        << " devices:\n";
    new_frames.clear();
    for (int i = 0 ; i < num_frames ; i++) {
        if (pipelines.size() == 0) break;
        for (auto &&pipe : pipelines) {
            rs2::frameset fs;
            fs = pipe.wait_for_frames(timeout_ms);
            cout << "-";
            for (rs2::frame& f : fs) {
                new_frames.emplace_back(f);
                cout << ".";
            }
        }
        cout << endl;
    }
    cout << " [DONE]\n";
}

// Collects relevant MOAD data from all RealSense devices
void RealSenseHandler::get_current_frame(int timeout_ms) {
    cout << "Getting RS - \n";
    rs2::colorizer color_map;
    std::stringstream out_file;
    cout << "Getting rotation matrix for " << turntable_position << " degress...\n";
    rot_matrix = createRotationMatrix(turntable_position);
    cout << rot_matrix << endl << endl;
    std::vector<std::thread> thread_vector;
    new_frames.clear();
    for (auto &&pipe : pipelines) {
        // Get serial number for this camera
        std::string serial_number = pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        // Collect frameset from camera
        rs2::frameset fs;
        fs = pipe.wait_for_frames(timeout_ms);
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        fs = align_to_depth.process(fs);
        
        thread_vector.emplace_back([&, fs, serial_number]() {
            process_frames(fs, serial_number);
        });
    
        // Add frames to new_frames vector
        // for (rs2::frame f : fs) {
        //     new_frames.emplace_back(f);
        // }    
    }
    for (auto& thread : thread_vector) {
        thread.join();
    }
    cout << "Got frames from all RS\n";
}

void RealSenseHandler::process_frames(rs2::frameset fs, std::string serial_number) {
    std::stringstream out_file;
    rs2::video_frame color = fs.get_color_frame();
    rs2::depth_frame depth = fs.get_depth_frame();
    // Apply filters
    // color = decimation_filter.process(color);
    // depth = decimation_filter.process(depth); // Reduces frame size
    depth = threshold_filter.process(depth);
    depth = spatial_filter.process(depth);
    depth = temporal_filter.process(depth);
    // Depth and color frames must have the same resolution for clouds to be colored properly
    cout << camera_names[serial_number] << " Color: " << color.get_width() << "x" << color.get_height();
    cout << "   Depth : " << depth.get_width() << "x" << depth.get_height() << endl;

    // SAVE POINTCLOUD
    if (save_pointcloud) {
        // Create PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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
        // Output 'rsx:C' indicates the pointcloud object has been constructed.
        cout << "[" << camera_names[serial_number] << ":C]";

        // Don't apply transforms and filters if raw_pointclouds is true
        if (!raw_pointclouds) {
            //Apply appropriate pointcloud transform
            pcl::transformPointCloud(*cloud, *cloud, camera_transforms[serial_number]);
            pcl::transformPointCloud(*cloud, *cloud, rot_matrix);
            // Output 'rsx:T' indicates the appropriate pointcloud transforms have been performed.
            cout << "[" << camera_names[serial_number] << ":T]";
            //Apply passthrough filters to remove background
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            float fmin, fmax;
            if (bool(stoi(config["rs_xpass"]))) {
                fmin = stof(config["rs_xpass_min"]);
                fmax = stof(config["rs_xpass_max"]);
                cout << "[" << camera_names[serial_number] << ":X:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            
            if (bool(stoi(config["rs_ypass"]))) {
                fmin = stof(config["rs_ypass_min"]);
                fmax = stof(config["rs_ypass_max"]);
                cout << "[" << camera_names[serial_number] << ":Y:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            
            if (bool(stoi(config["rs_zpass"]))) {
                fmin = stof(config["rs_zpass_min"]);
                fmax = stof(config["rs_zpass_max"]);
                cout << "[" << camera_names[serial_number] << ":Z:" << fmin << "," << fmax << "]";
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(fmin, fmax);
                pass.filter(*cloud);
            }
            
            if (bool(stoi(config["rs_sor"]))) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor(true);
                fmin = stof(config["rs_sor_stddev"]);
                int k = stoi(config["rs_sor_meank"]);
                sor.setInputCloud(cloud);
                sor.setMeanK(k);  // Number of neighbors to use for mean distance estimation
                sor.setStddevMulThresh(fmin);  // Standard deviation threshold for outlier detection
                sor.filter(*cloud);
                int removed = sor.getRemovedIndices()->size();
                cout << "[" << camera_names[serial_number] << ":SOR:" << removed << " points]";
            }
            

            // Output 'rsx:F' indicates the pointcloud has been filtered.
            cout << "[" << camera_names[serial_number] << ":F]";
        }
        

        // Generate pointcloud name and save
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_cloud.ply";
        // Save the point cloud to a PLY file
        // Output 'rsx:S' indicates the pointcloud is now being saved.
        cout << "[" << camera_names[serial_number] << ":S]";
        pcl::io::savePLYFile(out_file.str(), *cloud);
        cout << "[" << camera_names[serial_number] << ":SAVED]\n";
    }
    
    // SAVE COLOR IMAGE
    if (save_color_img) {
        cv::Mat color_mat(color.get_height(), color.get_width(), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        // cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);

        // Generate image name and save depth_mat
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_color.png";
        cv::imwrite(out_file.str(), color_mat);
    }

    // SAVE DEPTH IMAGE
    if (save_depth_img) {
        cv::Mat depth_mat(cv::Size(depth.get_width(), depth.get_height()), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::normalize(depth_mat, depth_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        
        // Generate image name and save depth_mat
        out_file.str("");
        out_file << save_dir << "\\" << camera_names[serial_number] << "_"
            << std::setfill('0') << std::setw(3) << turntable_position << "_depth.png";
        cv::imwrite(out_file.str(), depth_mat);
    }
    cout << endl;
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