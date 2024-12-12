
# Start with an Ubuntu base image
FROM ubuntu:22.04
RUN echo "Building MOAD Control Docker Image..."

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
   build-essential \
   cmake \
   g++ \
   git \
   libopencv-dev \
   libusb-1.0-0-dev \
   pkg-config \
   wget \
   && rm -rf /var/lib/apt/lists/*

# ----------------------------
# Install OpenCV
# ----------------------------
RUN echo "Installing OpenCV..."
RUN apt-get update && apt-get install -y \
    python3-opencv

# ----------------------------
# Install PCL (Point Cloud Library)
# ----------------------------
RUN echo "Installing PCL..."
RUN apt-get update && apt-get install -y \
   libpcl-dev

# ----------------------------
# Install Intel RealSense SDK
# ----------------------------
# Instructions from: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md 
RUN echo "Installing RealSense SDK.."
RUN apt-get install -y sudo apt-transport-https curl lsb-release
# RUN mkdir -p /etc/apt/keyrings
# RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
#     tee /etc/apt/sources.list.d/librealsense.list
# RUN apt-get update
# RUN apt-get install -y librealsense2-dkms
# RUN apt-get install -y librealsense2-utils
# RUN apt-get install -y librealsense2-dev
# RUN apt-get install -y librealsense2-dbg
# Instructions from: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md 
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
RUN apt-get install -y git wget cmake build-essential
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at v4l-utils 
WORKDIR /workspace
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /workspace/librealsense
RUN ./scripts/setup_udev_rules.sh
# RUN ./scripts/patch-realsense-ubuntu-lts-hwe.sh
RUN mkdir build
WORKDIR /workspace/librealsense/build
# RUN cmake ..
RUN cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
RUN make uninstall && make clean && make && make install


# ----------------------------
# Canon EDSDK Installation
# ----------------------------
# TODO: Email Canon support to find out when Intel Linux support will be rereleased


# Copy the test program into the container
RUN echo "asdwwaawz"

ADD import_test/. /workspace/import_test
# COPY import_test /workspace
# COPY import_test/import_test.cpp /workspace/import_test/
# COPY import_test/CMakeLists.txt /workspace/import_test/
RUN mkdir /workspace/import_test/build
WORKDIR /workspace/import_test/build
RUN cmake .. && make
# Default command
WORKDIR /workspace/import_test

CMD ["/bin/bash"]
# For Testing
# ENTRYPOINT ["/workspace/import_test/build/import_test"]