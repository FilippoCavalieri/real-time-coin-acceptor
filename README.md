
# Real-Time Coin Acceptor

[![License: Unlicense](https://img.shields.io/badge/license-Unlicense-blue.svg)](http://unlicense.org/)

A real-time coin acceptor powered by a Raspberry Pi Pico shipped with FreeRTOS and Micro-ROS.  
This project includes:
- C++ code for Raspberry Pi Pico
- calibration utilities written in Python
- Python frontend of the coin acceptor

## Dependencies

### 0. Configure Groups
``` bash
# add user to groups
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER

# To mount PICO without sudo
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0003", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-rpi-pico.rules > /dev/null
```
(Reboot your system afterward.)

### 1. Install Pico SDK
First, make sure the Pico SDK is properly installed and configured:

```bash
# Install dependencies
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

### 2. build Micro-ROS agent (first time only)
Micro-ROS uses a client-server architecture, so you must start the Micro-ROS Agent before.

```bash
# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
```

### 3. Clone this repository

Clone the repository recursively.
```bash
git clone --recursive https://github.com/FilippoCavalieri/real-time-coin-acceptor
```

> [!WARNING]
> **ONLY** `jazzy` ROS distribution is supported

## Running project

#### Build

```bash
cd real-time-coin-acceptor
mkdir build
cd build
cmake ..
make
```

#### Flash

To flash the device, hold the boot button, plug in the USB, and run:
```bash
cp coin_acceptor_microros.uf2 $(findmnt -rn -o TARGET -S LABEL=RPI-RP2)/
```

#### Run micro ROS agent
```bash
# Source workspace
cd microros_ws
source install/local_setup.bash

# Run microros agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### check published messages
```bash
ros2 topic list
/coinValuePublisher
/parameter_events
/rosout

ros2 topic echo /coinValuePublisher
data: 0
---
data: 1
---
data: 2
---
.....
```
