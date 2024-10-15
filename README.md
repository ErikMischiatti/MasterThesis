# Master Thesis Project - Robotic Manipulation with Learning from Demonstration

This repository contains the code and resources for my Master Thesis project developed at the Technical University of Vienna (TUW) as part of the Mechatronic Engineering Master's program at the University of Trento. The project involves the use of a **Franka Emika Panda** robotic arm and an **Intel RealSense D435i** camera to implement a system for robotic manipulation tasks based on **demonstration learning**.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Project Structure](#project-structure)
6. [Future Work](#future-work)
7. [Acknowledgements](#acknowledgements)

## Project Overview

In this project, I aim to teach a robotic arm how to perform complex tasks using demonstration learning techniques. The key components include:
- Unified 6D Pose Estimation for object tracking.
- Behavior Trees for task execution.
- Imitation learning algorithms to reverse previously learned tasks.
- Modular communication system using **Ultra-Wideband (UWB)** technology (as described in my co-authored paper).
  
The system is designed to manipulate objects by performing a sequence of predefined trajectories, detect objects like cables, and adapt the trajectories based on sensory input. Through the use of demonstration learning, the robot can learn from human demonstrations and apply those learned skills in novel situations.

The project is split into several stages:
- **Learning from Demonstration (LfD)**: Capture human demonstrations using both the robot and the Intel RealSense camera.
- **Behavior Tree Integration**: Implement task sequences through a modular approach using behavior trees for robust decision-making and execution of robotic tasks.
- **Object Tracking and Pose Estimation**: Use 6D pose estimation for real-time object tracking to adjust robotic trajectories dynamically.
- **Trajectory Execution**: Use both predefined and learned trajectories to manipulate objects in the environment, adapting to changes detected through sensory input.

The robotic arm is controlled using **ROS** (Robot Operating System), which facilitates the communication between the different components and subsystems, such as the camera, sensors, and the robotic arm.

## Requirements

To run this project, you need the following:

### Hardware
- **Franka Emika Panda** robotic arm.
- **Intel RealSense D435i** camera for depth and RGB-D vision.
- **PC** running Ubuntu 20.04 (or later) with a ROS-compatible environment.

### Software
- **ROS Noetic** (or ROS 1 distribution compatible with your system).
- **Franka Control Interface (libfranka)** for controlling the robotic arm.
- **Intel RealSense SDK** for integrating the camera.
- **Python 3.8+**.
- **Behavior Trees Libraries** (PyTrees or BehaviorTree.CPP).
- Python dependencies (can be installed with `rosdep`).

### ROS Packages
You will need to install the following ROS packages:
- `franka_ros`: ROS interface for controlling Franka Emika Panda.
- `realsense-ros`: ROS drivers for Intel RealSense cameras.
- `moveit`: For motion planning and control of the robotic arm.

## Installation

### Clone the repository:

# Install ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full

# Install Python 3 dependencies and rosdep
sudo apt install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Install Intel RealSense SDK
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo focal main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Install Franka Emika Panda dependencies
sudo apt install ros-noetic-franka-ros

# Clone the repository
git clone https://github.com/ErikMischiatti/your-private-repo.git catkin_ws
cd catkin_ws

# Install ROS package dependencies with rosdep
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make
source devel/setup.bash

# Step 1: Source the ROS workspace
source ~/catkin_ws/devel/setup.bash

# Step 2: Launch the system (robot drivers and camera)
roslaunch <your_package_name> start_system.launch

# Step 3: Begin demonstration learning
roslaunch <your_package_name> demonstration_learning.launch

# Step 4: To execute tasks using learned trajectories, run:
roslaunch <your_package_name> task_execution.launch

# Step 5: If you want to record demonstrations (teleoperation or hand-guiding mode):
rosrun <your_package_name> record_demonstration.py

# Step 6: Run the behavior tree manager for task execution
roslaunch <your_package_name> behavior_tree.launch

# Navigate to your catkin workspace
cd ~/catkin_ws

# Display the project structure
tree -L 2

# Expected project structure:
# catkin_ws/
# ├── src/
# │   ├── <your_package_name>/  # Main package containing all the project code
# │   │   ├── launch/           # Launch files for various system components
# │   │   ├── scripts/          # Python scripts for task execution and learning
# │   │   ├── config/           # Configuration files (e.g., for camera, robot)
# │   │   ├── urdf/             # URDF models for the robot and objects
# │   │   └── msg/              # Custom ROS messages and services
# │   └── other_packages/       # Additional ROS packages used in the project
# ├── README.md                 # This file
# ├── .gitignore                # Files and directories to ignore in the repository
# └── CMakeLists.txt            # CMake configuration for the ROS workspace

# Future Work:
# - Extend the system to handle multi-object manipulation, enabling the robot to interact with multiple objects simultaneously.
# - Implement advanced error recovery mechanisms to detect and correct errors during task execution.
# - Improve the generalization of the learned tasks, allowing the robot to apply learned behaviors to new and unseen scenarios.


# Acknowledgements:
# I would like to express my gratitude to:
# - My supervisor at the Technical University of Vienna (TUW) for their guidance and feedback.
# - The University of Trento for the education and support provided during my Mechatronic Engineering studies.
# - The ROS and Franka open-source communities for providing invaluable tools and resources for this project.

