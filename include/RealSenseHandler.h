#include <windows.h>
#include <vector> 
#include <string>
#include <map>

#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSenseHandler {
private:
    bool save_depth_img = true;
    bool save_color_img = true;
    bool save_pointcloud = true;
    bool raw_pointclouds = true;
    std::string save_dir;
    std::map<std::string, std::string> camera_names;
    std::map<std::string, Eigen::Matrix4f> camera_transforms;
    Eigen::Matrix4f rot_matrix;
    size_t device_count;
    rs2::context ctx;
    rs2::threshold_filter threshold_filter;
    rs2::decimation_filter decimation_filter;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    std::vector<rs2::pipeline> pipelines;
    std::vector<rs2::frame> new_frames;
    cv::Mat h;
    // void save_pointcloud(rs2::frameset& fs, std::string camera_id);
    void print_device(rs2::device dev, bool print_streams=true);
    void process_frames(rs2::frameset fs, std::string serial_number);
public:
    int turntable_position = 0;
    RealSenseHandler();
    int device_check();
    void get_frames(int num_frames=1, int timeout_ms=10000);
    void get_current_frame(int timeout_ms=10000);
};
