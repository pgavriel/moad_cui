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

echo "Installing required system libraries/packages..."

# realsense sdk
echo "--RealSense SDK--"
echo "from https://github.com/realsenseai/librealsense/blob/master/doc/installation.md which has more detail"

# must clone librealsense2 and run, moving out a directory assuming above directory structure
cd ..


sudo apt-get update && sudo apt-get upgrade

sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

sudo apt-get install git wget cmake build-essential

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

git clone https://github.com/realsenseai/librealsense.git

cd librealsense
./scripts/setup_udev_rules.sh

mkdir build && cd build

cmake ../

sudo make uninstall && make clean && make && sudo make install