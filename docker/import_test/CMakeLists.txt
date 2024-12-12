cmake_minimum_required(VERSION 3.10)
project(import_test)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Find RealSense SDK
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})
message(STATUS "RealSense Include Dirs: ${realsense2_INCLUDE_DIRS}")
message(STATUS "RealSense Library Path: ${realsense2_LIBRARY}")

# Add the test program executable
add_executable(${PROJECT_NAME} import_test.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${realsense2_LIBRARY})
