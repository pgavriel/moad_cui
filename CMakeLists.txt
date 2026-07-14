# CMakeList.txt : MultiCamCui
cmake_minimum_required (VERSION 3.8)

# using vcpkg for PCL was necessary for windows version, now no longer needed - gs 11/12
#  Use vcpkg to include PCL
#  set(CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(MultiCamCui CXX)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INC_DIR ${PROJECT_SOURCE_DIR}/include)

# Moved serial communication code into this repository - PG 6/23/26
# set(SER_DIR ${PROJECT_SOURCE_DIR}/../simple_serial_port/linux)


# Find RealSense SDK includes
find_library(REALSENSE2_FOUND realsense2 HINTS ${RS_LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

# Report where the realsense2 library and headers were found
get_filename_component(REALSENSE2_LIB_DIR ${REALSENSE2_FOUND} DIRECTORY)

message(STATUS "realsense2 library: ${REALSENSE2_FOUND}")
message(STATUS "realsense2 library directory: ${REALSENSE2_LIB_DIR}")

# Try to locate headers if not set
find_path(REALSENSE2_INCLUDE_DIR
    NAMES librealsense2/rs.hpp
    HINTS ${RS_INCLUDE_DIR} /usr/include /usr/local/include /opt/librealsense/include
    PATH_SUFFIXES librealsense2
)

if(REALSENSE2_INCLUDE_DIR)
    message(STATUS "realsense2 include directory: ${REALSENSE2_INCLUDE_DIR}")
    set(RS_INCLUDE_DIR ${REALSENSE2_INCLUDE_DIR} CACHE PATH "RealSense include directory" FORCE)
else()
    message(WARN "Could not locate librealsense2 headers (librealsense2/rs.hpp).")
endif()

# Expose library dir too
if(REALSENSE2_LIB_DIR)
    set(RS_LIBRARY_DIR ${REALSENSE2_LIB_DIR} CACHE PATH "RealSense library directory" FORCE)
endif()

# Find OpenCV includes
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# json parsing library needed for moad_config
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

# Find PCL includes
# set(PCL_DIR "C:\\src\\vcpkg\\installed\\x64-windows\\share\\pcl")
find_package(PCL 1.3 REQUIRED COMPONENTS)


if(MSVC)
    string(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /EHsc")
    set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /DBUILD_TYPE=\\\"Debug\\\"")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /DBUILD_TYPE=\\\"Release\\\"")
elseif(UNIX AND NOT APPLE)
    add_compile_options(-D TARGET_OS_LINUX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fpermissive -pthread")
    set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -DBUILD_TYPE=\\\"Debug\\\"")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBUILD_TYPE=\\\"Release\\\"")
# elseif(APPLE)
    # will anyone be using this with apple??
else()
    message(WARN "not MSVC, UNIX or APPLE compiler")
endif()




add_executable (MultiCamCui
    ${SRC_DIR}/MOADCui.cpp
    
    ${SRC_DIR}/RealSenseHandler.cpp
    ${SRC_DIR}/CanonHandler.cpp
    ${SRC_DIR}/SerialCommunication.cpp
    ${SRC_DIR}/ScriptRunner.cpp
    ${SRC_DIR}/ScanManager.cpp
    
    ${SRC_DIR}/CameraEvent.cpp
    ${SRC_DIR}/Download.cpp
    ${SRC_DIR}/PreSetting.cpp
    ${SRC_DIR}/PressShutter.cpp
    ${SRC_DIR}/Property.cpp
    ${SRC_DIR}/TakePicture.cpp
    ${SRC_DIR}/DownloadEvf.cpp

    ${SRC_DIR}/MenuHandler.cpp
    ${SRC_DIR}/ConfigHandler.cpp
    ${SRC_DIR}/ThreadPool.cpp
    ${SRC_DIR}/DebugUtils.cpp
)

set_target_properties(MultiCamCui PROPERTIES
	CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)

target_include_directories(MultiCamCui
  PUBLIC ${SRC_DIR}
  PUBLIC ${INC_DIR}
  PUBLIC ${INC_DIR}/Camera
  PUBLIC ${INC_DIR}/Class
  PUBLIC ${INC_DIR}/Command
  PUBLIC ${PROJECT_SOURCE_DIR}/../EDSDK/Header
#   PUBLIC ${SER_DIR}
  PUBLIC ${RS_INCLUDE_DIR}
  PRIVATE ${PCL_INCLUDE_DIRS}
  )

  
set(EDSDK_LDIR ${CMAKE_CURRENT_SOURCE_DIR}/../EDSDK) # removed _64

if(MSVC)
    target_link_libraries(MultiCamCui
        PRIVATE
            ${EDSDK_LDIR}/Library/EDSDK.lib
            ${PCL_LIBRARIES}
            nlohmann_json::nlohmann_json
            udev
        PUBLIC
            ${REALSENSE2_FOUND}
            ${OpenCV_LIBS}
    )
elseif(UNIX AND NOT APPLE)
    target_link_libraries(MultiCamCui
        PRIVATE
            ${EDSDK_LDIR}/Library/libEDSDK.so
            ${PCL_LIBRARIES}
            nlohmann_json::nlohmann_json
            realsense2 # necessary to add this for linux port
            udev
        PUBLIC
            ${REALSENSE2_FOUND}
            ${OpenCV_LIBS}
    )
elseif(APPLE)
else()
endif()

message(STATUS "Canon EDSDK Dir: " ${EDSDK_LDIR})
if(MSVC)
    add_custom_command(TARGET MultiCamCui POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EDSDK_LDIR}/Dll/EDSDK.dll" $<TARGET_FILE_DIR:MultiCamCui>
    )
    add_custom_command(TARGET MultiCamCui POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EDSDK_LDIR}/Dll/EdsImage.dll" $<TARGET_FILE_DIR:MultiCamCui>
    )
endif()

# Post Build script to copy realsense2.dll
if(WIN32)
message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
add_custom_command(TARGET MultiCamCui POST_BUILD        # Adds a post-build event to rs-getting-started
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${RS_DLL_DIR}/realsense2.dll"                            # <--this is in-file
        $<TARGET_FILE_DIR:MultiCamCui>)                 # <--this is out-file path
endif()

message(STATUS "CMake Install Prefix: " ${CMAKE_INSTALL_PREFIX})
install(
    TARGETS

    MultiCamCui

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
