# DETECTION Module

This module is designed to detect ArUco markers and specific components (connectors and cables) using a RealSense camera. It includes scripts for detection, configuration files, and ROS launch files to initialize the detection pipeline.

## Directory Structure

### 1. Root Files
- **CMakeLists.txt**: CMake configuration file for building and managing the detection module within a ROS workspace.
- **package.xml**: Defines ROS package dependencies and metadata for the `dlo_detection` package.
- **requirements.txt**: Lists Python dependencies required by the detection scripts, including `numpy`, `opencv-python`, and `pyrealsense2`.

### 2. `aruco_detection/` Directory
Contains scripts and utilities for detecting ArUco markers and setting up the detection process.
- **aruco_config.py**: Contains configuration settings for ArUco marker detection, such as camera parameters and specific marker IDs.
- **aruco_detection.py**: Main script for detecting ArUco markers, which processes image data to identify markers and extract position and orientation.
- **aruco_detection_filtered.py**: An extended version of `aruco_detection.py` with added filtering methods to reduce noise in detection results.
- **aruco_realsense.py**: Manages integration with the RealSense camera, capturing frames to be processed in the detection pipeline.
- **aruco_trackbars.py**: Provides an interface using trackbars to adjust detection parameters in real-time (e.g., HSV values).
- **aruco_vision.py**: Handles the visualization of detected markers, rendering relevant information on the output display.
- **hsv_tuner.py**: A tool for calibrating HSV color ranges to optimize detection settings for various lighting conditions.

### 3. `launch/` Directory
Contains ROS launch files for initializing different parts of the detection and visualization pipeline.
- **camera_pose.launch**: Launches and configures the camera to retrieve its pose.
- **detection_pipeline.launch**: Main launch file that starts the entire detection pipeline, including camera and ArUco detection nodes.
- **my_rviz_config.PANDA.yml**: Custom RViz configuration for visualizing the PANDA robot and detection outputs.
- **my_rviz_config.rviz**: RViz configuration file that sets up display settings for visualizing camera data and detected objects.

### 4. Additional Files
- **aruco_main.py**: Primary script to run the ArUco detection pipeline, integrating different scripts from `aruco_detection/`.
- **params_v2.json**: JSON file containing HSV threshold values for detecting connectors and cables, with separate parameters for each component type.
