# Image Conversion Package

A ROS 2 package to convert images from a USB camera to grayscale or keep them in color.

## Prerequisites
Install th usb_cam package in your workspace:
```bash
sudo apt-get install ros-<ros2-distro>-usb-cam
```
My system consists of ROS2 Humble for this package

## Installation
Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Naturalwimp/image_conversion_pkg.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Usage
Launch the file using thsi command line in terminal
```bash
ros2 launch image_conversion_pkg image_conversion_launch.py
```

## Use ROS2 Service 
Call the service to change the mode:
```bash
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"  # Set to Greyscale
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}" # Set to Color
```
