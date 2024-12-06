#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main() {
    std::cout << "\nRUNNING MOAD IMPORT TEST: \n\n";
    // OpenCV Test
    std::cout << "== OpenCV ==\nVersion: " << CV_VERSION << std::endl << std::endl;

    // RealSense Test
    try {
        rs2::context ctx;
        std::cout << "== RealSense ==\nVersion: " << RS2_API_VERSION_STR << std::endl;
        if (ctx.query_devices().size() > 0) {
            std::cout << "RealSense device found!" << std::endl;
        } else {
            std::cout << "No RealSense device connected." << std::endl;
        }
    } catch (const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }

    // PCL Test
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    
    // Fill in the cloud data
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "\n== PCL == \nOK! Point cloud created with " << cloud.points.size() << " points." << std::endl;

    return 0;
}
