# adasa_rover

A supermart bot for inventory management via RFID area scanning and SLAM using a Zed2 camera and 3 sonars at the back.

## Pre-requisites

1. Ubuntu 20.04
2. Ros 2 Foxy Desktop
3. Time

## Setup instructions

1. Install Nvidia graphics drivers 530 for your system.
2. Install CUDA 12.1 from [here](https://developer.nvidia.com/cuda-downloads?target_os=Linux).
3. Install zstd for installing Zed SDk 4.0 if you don't have it already `sudo apt install zstd`
4. Install Zed SDK 4.0 from [ZED SDK Download Page](https://www.stereolabs.com/developers/release/)

## Build ros2 packages

``` bash

# Create robot workspace
cd ~
mkdir -p adasa_ws/src
# Clone adasa_rover repository
cd adasa_ws/src
git clone https://github.com/carpit680/adasa_rover
# Install all dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y
# build all packages
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
# Add sourcing entry to bashrc
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
# or zshrc
echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc
# Restart the terminal or source .bashrc file
source ~/.bashrc 
# or .zshrc 
source ~/.zshrc
```
