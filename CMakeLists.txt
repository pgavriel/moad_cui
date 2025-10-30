# CMakeList.txt : MultiCamCui
cmake_minimum_required (VERSION 3.8)

# Use vcpkg to include PCL
# set(CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(MultiCamCui CXX)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INC_DIR ${PROJECT_SOURCE_DIR}/include)
set(SER_DIR ${PROJECT_SOURCE_DIR}/../simple_serial_port)


set(RS_INCLUDE_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include") # folder where librealsense2 folder is found
set(RS_LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64") # where realsense2.lib is found
set(RS_DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64") # where realsense2.lib is found

find_library(REALSENSE2_FOUND realsense2 HINTS ${RS_LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)


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
elseif(APPLE)
else()
endif()

add_executable (MultiCamCui
    ${SRC_DIR}/MOADCui.cpp
    
    ${SRC_DIR}/RealSenseHandler.cpp
    ${SRC_DIR}/CanonHandler.cpp
    ${SER_DIR}/SimpleSerial.cpp
    
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
    
    
    #${SRC_DIR}/MultiCamCui.cpp
    #${SRC_DIR}/MOADCui.cpp
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
  PUBLIC ${SER_DIR}
  PUBLIC ${RS_INCLUDE_DIR}
  PRIVATE ${PCL_INCLUDE_DIRS}
  )

  
set(EDSDK_LDIR ${CMAKE_CURRENT_SOURCE_DIR}/../EDSDK_64)

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
            udev
            realsense2
            udev
        PUBLIC
            ${REALSENSE2_FOUND}
            ${OpenCV_LIBS}
    )
elseif(APPLE)
else()
endif()

message(WARN ${EDSDK_LDIR})
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

message(WARN ${CMAKE_INSTALL_PREFIX})
install(
    TARGETS

    MultiCamCui

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)



# cmake_minimum_required(VERSION 3.12)
# project(MultiCamCui LANGUAGES CXX)

# # ============================================================
# # Compiler Configuration
# # ============================================================

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# if(MSVC)
#     add_compile_options(/W4 /EHsc)
# else()
#     add_compile_options(-Wall -Wextra -pthread -fpermissive)
# endif()

# # ============================================================
# # VCPKG Integration (for RealSense)
# # ============================================================

# # Set this manually if vcpkg is not globally integrated
# # Example: cmake -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -S . -B build
# if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "${CMAKE_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake")
#     set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
# endif()

# # ============================================================
# # Directories
# # ============================================================

# set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
# set(INC_DIR ${PROJECT_SOURCE_DIR}/include)
# set(SER_DIR ${PROJECT_SOURCE_DIR}/../simple_serial_port)
# set(EDSDK_DIR ${PROJECT_SOURCE_DIR}/../EDSDK_64)

# # ============================================================
# # Dependencies
# # ============================================================

# # OpenCV (via apt or vcpkg)
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})


# # PCL (via apt)
# find_package(PCL 1.3 REQUIRED COMPONENTS common io)
# include_directories(${PCL_INCLUDE_DIRS})
# add_definitions(${PCL_DEFINITIONS})


# # nlohmann/json (header-only)
# find_package(nlohmann_json QUIET)
# if(NOT nlohmann_json_FOUND)
#     include(FetchContent)
#     FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
#     FetchContent_MakeAvailable(json)
# endif()

# # RealSense (via vcpkg)
# find_package(realsense2 REQUIRED)

# # EDSDK (Canon SDK, local)
# include_directories(${EDSDK_DIR}/Header)
# link_directories(${EDSDK_DIR}/Library)


# # ============================================================
# # Source Files
# # ============================================================

# file(GLOB SRC_FILES
#     ${SRC_DIR}/*.cpp
#     ${SER_DIR}/SimpleSerial.cpp
# )

# # ============================================================
# # Executable
# # ============================================================

# add_executable(MultiCamCui ${SRC_FILES})

# target_include_directories(MultiCamCui PUBLIC
#     ${SRC_DIR}
#     ${INC_DIR}
#     ${INC_DIR}/Camera
#     ${INC_DIR}/Class
#     ${INC_DIR}/Command
#     ${SER_DIR}
# )

# target_link_libraries(MultiCamCui
#     PRIVATE
#         nlohmann_json::nlohmann_json
#         ${PCL_LIBRARIES}
#         udev
#     PUBLIC
#         ${OpenCV_LIBS}
#         realsense2
# )

# # Canon SDK linkage
# if(WIN32)
#     target_link_libraries(MultiCamCui PRIVATE ${EDSDK_DIR}/Library/EDSDK.lib)
# elseif(UNIX)
#     target_link_libraries(MultiCamCui PRIVATE ${EDSDK_DIR}/Library/libEDSDK.so)
# endif()

# # ============================================================
# # Installation
# # ============================================================

# install(TARGETS MultiCamCui RUNTIME DESTINATION bin)













# # # CMakeList.txt : MultiCamCui
# # cmake_minimum_required (VERSION 3.8)

# # # Use vcpkg to include PCL
# # set(CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")

# # project(MultiCamCui CXX)

# # set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
# # set(INC_DIR ${PROJECT_SOURCE_DIR}/include)
# # set(SER_DIR ${PROJECT_SOURCE_DIR}/../simple_serial_port)


# # set(RS_INCLUDE_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include") # folder where librealsense2 folder is found
# # set(RS_LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64") # where realsense2.lib is found
# # set(RS_DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64") # where realsense2.lib is found

# # find_library(REALSENSE2_FOUND realsense2 HINTS ${RS_LIBRARY_DIR} REQUIRED)
# # if(NOT REALSENSE2_FOUND)
# #     SET(REALSENSE2_FOUND "realsense2")
# #     message(WARN "Failed to find_library(realsense2)")
# # endif()

# # set(OpenCV_DIR "C:/opencv/build")
# # find_package(OpenCV REQUIRED)
# # include_directories(${OpenCV_INCLUDE_DIRS})

# # include(FetchContent)
# # FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
# # FetchContent_MakeAvailable(json)


# # # set(PCL_DIR "C:\\src\\vcpkg\\installed\\x64-windows\\share\\pcl")
# # find_package(PCL 1.3 REQUIRED COMPONENTS)


# # if(MSVC)
# #     string(REPLACE "/W3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
# #     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /EHsc")
# #     set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /DBUILD_TYPE=\\\"Debug\\\"")
# #     set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /DBUILD_TYPE=\\\"Release\\\"")
# # elseif(UNIX AND NOT APPLE)
# #     add_compile_options(-D TARGET_OS_LINUX)
# #     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fpermissive -pthread")
# #     set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -DBUILD_TYPE=\\\"Debug\\\"")
# #     set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBUILD_TYPE=\\\"Release\\\"")
# # elseif(APPLE)
# # else()
# # endif()

# # add_executable (MultiCamCui
# #     ${SRC_DIR}/MOADCui.cpp
    
# #     ${SRC_DIR}/RealSenseHandler.cpp
# #     ${SRC_DIR}/CanonHandler.cpp
# #     ${SER_DIR}/SimpleSerial.cpp
    
# #     ${SRC_DIR}/CameraEvent.cpp
# #     ${SRC_DIR}/Download.cpp
# #     ${SRC_DIR}/PreSetting.cpp
# #     ${SRC_DIR}/PressShutter.cpp
# #     ${SRC_DIR}/Property.cpp
# #     ${SRC_DIR}/TakePicture.cpp
# #     ${SRC_DIR}/DownloadEvf.cpp

# #     ${SRC_DIR}/MenuHandler.cpp
# #     ${SRC_DIR}/ConfigHandler.cpp
# #     ${SRC_DIR}/ThreadPool.cpp
# #     ${SRC_DIR}/DebugUtils.cpp
    
    
# #     #${SRC_DIR}/MultiCamCui.cpp
# #     #${SRC_DIR}/MOADCui.cpp
# # )

# # set_target_properties(MultiCamCui PROPERTIES
# # 	CXX_STANDARD 17
# #     CXX_STANDARD_REQUIRED YES
# #     CXX_EXTENSIONS NO
# #     BUILD_RPATH "$ORIGIN"
# #     INSTALL_RPATH "$ORIGIN"
# # )

# # target_include_directories(MultiCamCui
# #   PUBLIC ${SRC_DIR}
# #   PUBLIC ${INC_DIR}
# #   PUBLIC ${INC_DIR}/Camera
# #   PUBLIC ${INC_DIR}/Class
# #   PUBLIC ${INC_DIR}/Command
# #   PUBLIC ${PROJECT_SOURCE_DIR}/../EDSDK/Header
# #   PUBLIC ${SER_DIR}
# #   PUBLIC ${RS_INCLUDE_DIR}
# #   PRIVATE ${PCL_INCLUDE_DIRS}
# #   )

  
# # set(EDSDK_LDIR ${CMAKE_CURRENT_SOURCE_DIR}/../EDSDK_64)

# # if(MSVC)
# #     target_link_libraries(MultiCamCui
# #         PRIVATE
# #             ${EDSDK_LDIR}/Library/EDSDK.lib
# #             ${PCL_LIBRARIES}
# #             nlohmann_json::nlohmann_json
# #         PUBLIC
# #             ${REALSENSE2_FOUND}
# #             ${OpenCV_LIBS}
# #     )
# # elseif(UNIX AND NOT APPLE)
# #     target_link_libraries(MultiCamCui
# #         PRIVATE
# #             ${EDSDK_LDIR}/Library/libEDSDK.so
# #             ${PCL_LIBRARIES}
# #         PUBLIC
# #             ${REALSENSE2_FOUND}
# #             ${OpenCV_LIBS}
# #     )
# # elseif(APPLE)
# # else()
# # endif()

# # message(WARN ${EDSDK_LDIR})
# # if(MSVC)
# #     add_custom_command(TARGET MultiCamCui POST_BUILD
# #         COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EDSDK_LDIR}/Dll/EDSDK.dll" $<TARGET_FILE_DIR:MultiCamCui>
# #     )
# #     add_custom_command(TARGET MultiCamCui POST_BUILD
# #         COMMAND ${CMAKE_COMMAND} -E copy_if_different "${EDSDK_LDIR}/Dll/EdsImage.dll" $<TARGET_FILE_DIR:MultiCamCui>
# #     )
# # endif()

# # # Post Build script to copy realsense2.dll
# # if(WIN32)
# # message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
# # message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
# # add_custom_command(TARGET MultiCamCui POST_BUILD        # Adds a post-build event to rs-getting-started
# #     COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
# #         "${RS_DLL_DIR}/realsense2.dll"                            # <--this is in-file
# #         $<TARGET_FILE_DIR:MultiCamCui>)                 # <--this is out-file path
# # endif()

# # message(WARN ${CMAKE_INSTALL_PREFIX})
# # install(
# #     TARGETS

# #     MultiCamCui

# #     RUNTIME DESTINATION
# #     ${CMAKE_INSTALL_PREFIX}/bin
# # )