# this file assumes Canon's EDSDK was succesfully downloaded and simple_serial_port with
#   necessary linux version was cloned correctly and the directory structure looks lie
#   the following

# reminder of directory structure
# moadrig_NERVE_location/
# ├── EDSDK/
# ├── moad_cui/ # <-- THIS REPO
# │   ...
# │   ├── install_requirements.sh # <-- THIS FILE
# │   ...
# └── simple_serial_port/
#     ├── linux/
#     ...

# note that this might move you outside of moad_cui or into the librealsense directory

echo "Installing required system libraries/packages..."





# realsense sdk
echo "--RealSense SDK--"
echo "from https://github.com/realsenseai/librealsense/blob/master/doc/installation.md which has more detail"

# must clone librealsense2 and run, moving out a directory assuming above directory structure
cd ..

apt-get update && apt-get upgrade

apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

apt-get install git wget cmake build-essential

apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

git clone https://github.com/realsenseai/librealsense.git

# possible error running next librealsense script, pre-installing v4l-utils should prevent this
#   info from if-statement inside setup_udev_rules.sh at 
#   https://github.com/realsenseai/librealsense/blob/master/scripts/setup_udev_rules.sh
apt install v4l-utils

cd librealsense
./scripts/setup_udev_rules.sh

mkdir build && cd build

cmake ../

make uninstall && make clean && make && make install




# opencv
echo "--OpenCV--"

# make sure pip is installed
apt install python3-pip  

pip install opencv-python



# pcl
echo "--OpenCV--"

apt install libpcl-dev


# nlohmann
# cmakelists autofinds package from github

