# Husky A200 SLAM with RPLIDAR A3 and ZED 2i Camera

This repository integrates the Husky A200 robot, RPLIDAR A3, NVIDIA Jetson AGX Orin, and ZED 2i camera for real-time SLAM, camera view, and point cloud visualization using ROS Noetic.

## Overview

This project was developed to enable real-time Simultaneous Localization and Mapping (SLAM) on a Husky A200 robot equipped with an RPLIDAR A3 sensor and a ZED 2i stereo camera. The NVIDIA Jetson AGX Orin serves as the compute platform, running ROS Noetic on Ubuntu 20.04 (JetPack 5.1.2). The `my_gmapping.launch` file orchestrates the SLAM pipeline using Gmapping, publishes sensor data from the LIDAR and camera, and launches RViz for visualization of the map, camera view, point cloud, and robot model.

## Prerequisites

- **Ubuntu 20.04** and **ROS Noetic** installed on Jetson AGX Orin (with JetPack 5.1.2).
  - Follow the official ROS Noetic installation guide: http://wiki.ros.org/noetic/Installation/Ubuntu
- **ZED SDK 4.2** installed on Jetson AGX Orin.
  - Download and install from: https://www.stereolabs.com/developers/release/

## Installation

1. **Create a Catkin Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   ```

2. **Clone Third-Party ROS Packages**:
   - **ZED ROS Wrapper**:
     ```bash
     cd ~/catkin_ws/src
     git clone --branch v4 --recursive https://github.com/stereolabs/zed-ros-wrapper.git
     ```
   - **RPLIDAR ROS Package**:
     ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/Slamtec/rplidar_ros.git
     ```
   - Install dependencies and build:
     ```bash
     cd ~/catkin_ws
     rosdep install --from-paths src --ignore-src -r -y
     catkin_make
     source devel/setup.bash
     ```

3. **Clone This Repository**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/<your-username>/Husky-SLAM.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Install Gmapping**:
   ```bash
   sudo apt-get install ros-noetic-slam-gmapping
   ```

## Usage

1. **Start ROS Master**:
   ```bash
   roscore
   ```

2. **Launch the SLAM Setup**:
   ```bash
   roslaunch my_husky_slam my_gmapping.launch
   ```
   This will:
   - Start Gmapping for SLAM using RPLIDAR A3 data.
   - Publish ZED 2i camera data (RGB image, point cloud).
   - Launch RViz for visualization of the map, LIDAR scans, camera view, point cloud, and Husky robot model.

3. **Navigate the Robot**:
   - Use a joystick or keyboard control to move the Husky A200 and build the map. For example, launch teleop:
     ```bash
     roslaunch husky_control teleop.launch
     ```

## Directory Structure

- `src/my_husky_slam/`:
  - `launch/my_gmapping.launch`: Launch file to start SLAM, sensor publishing, and visualization.
  - `rviz/my_gmapping.rviz`: RViz configuration for visualizing the map, camera view, point cloud, and robot model.
  - `husky.urdf`: Generated URDF file for the Husky A200 (optional, for reference).
- `src/husky/`: Modified version of the Husky ROS packages (see below for details).

## Notes on the `husky` Package

- This repository includes a modified version of the `husky` package, originally developed by Clearpath Robotics (https://github.com/husky/husky). Modifications include:
  - Updated `husky_description/urdf/husky.urdf.xacro` to add mounts for RPLIDAR A3 (positioned at x=0.2206, y=0.0, z=0.3) and ZED 2i camera (positioned at x=0.3, y=0.0, z=0.5).
  - Adjusted `husky_navigation/launch/gmapping.launch` for compatibility with RPLIDAR A3 (e.g., updated scan topic and parameters).
- The `husky` package is licensed under the BSD-3-Clause license. The original license and copyright notice from Clearpath Robotics are preserved in the `husky/LICENSE` file.

## Acknowledgments

- **ZED ROS Wrapper**: [Stereolabs](https://github.com/stereolabs/zed-ros-wrapper)
- **RPLIDAR ROS Package**: [Slamtec](https://github.com/Slamtec/rplidar_ros)
- **Husky ROS Packages**: [Clearpath Robotics](https://github.com/husky/husky)

## License

This project is licensed under the BSD-3-Clause license, except for the `husky` package, which retains its original BSD-3-Clause license from Clearpath Robotics (see `husky/LICENSE` for details).

## Contact

For questions or issues, please open an issue on this repository or contact <your-email@example.com>.
