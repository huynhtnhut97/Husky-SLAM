# Husky A200 SLAM with RPLIDAR A3 and ZED 2i Camera

This repository integrates the Husky A200 robot, RPLIDAR A3, NVIDIA Jetson AGX Orin, and ZED 2i camera for real-time SLAM, camera view, and point cloud visualization using ROS Noetic.

## Prerequisites
- **Ubuntu 20.04** and **ROS Noetic** installed on Jetson AGX Orin (with JetPack 5.1.2).
- **ZED SDK 4.0** installed on Jetson AGX Orin.

## Installation
1. **Create a Catkin Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
