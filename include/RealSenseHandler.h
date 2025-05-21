#include <windows.h>
#include <vector> 
#include <string>
#include <map>
#include <chrono>
#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "ThreadPool.h"
class RealSenseHandler {
private:
    bool running = true;
    
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
    std::map<std::string, rs2::pipeline> pipeline_map;
    std::map<std::string, rs2::frameset> frameset_map;
    std::mutex framesetMutex;
    std::map<std::string, std::thread> frame_thread_map;

    cv::Mat h;

    void print_device(rs2::device dev, bool print_streams=true);
    void process_frames(rs2::pipeline pipe, int timeout_ms=10000);
    void start_device(std::string serial_number);
    void frame_poll_thread(rs2::pipeline pipe);
public:
    int turntable_position = 0;
    int fail_count = 0;
    std::string save_dir;
    
    RealSenseHandler();
    ~RealSenseHandler();
    int device_check();
    void initialize();
    void get_frames(int num_frames=1, int timeout_ms=10000);
    void get_current_frame(int timeout_ms=10000, ThreadPool* pool=nullptr);
};
