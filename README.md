# ROS2_MOT
This is a repo for Object Tracking
Multi-Object Tracking (MOT) ROS 2 Package
Overview

This ROS 2 package implements a Multi-Object Tracking (MOT) system capable of tracking multiple objects in real-time. It utilizes an advanced object detection algorithm (e.g., YOLOv4) for identifying objects in video streams and integrates state-of-the-art tracking algorithms (e.g., ByteTrack) to track the detected objects across frames. This package is designed for real-time applications in robotics and autonomous systems, providing robust tracking capabilities in dynamic environments.

Features

    Real-time object detection and tracking
    Integration with advanced detection algorithms like YOLOv4
    Utilization of Kalman Filter for accurate state prediction
    Efficient data association using IoU and appearance features
    Customizable tracking parameters for different scenarios

Dependencies

    ROS 2 (Tested on ROS 2 Humble)
    OpenCV
    NumPy
    cv_bridge for ROS 2
    A deep learning framework (e.g., PyTorch or TensorFlow) for running the detection model
    [Optional] CUDA support for GPU acceleration
    
    Installation
ROS 2 Setup

Ensure you have ROS 2 installed on your system. Follow the official ROS 2 documentation for installation instructions: https://docs.ros.org/en/humble/Installation.html

Clone and Build the Package:

# Navigate to your ROS 2 workspace src folder
cd ~/ros2_ws/src

# Clone the MOT package repository
git clone <mot_tracking_package>

# Navigate back to your ROS 2 workspace root and build the package
cd ..
colcon build --packages-select mot_tracking_package

# Source the setup script
source install/setup.bash

Usage
Running the MOT Node

To launch the MOT node, which subscribes to video frames and publishes tracking data:
ros2 run mot_tracking_package mot_node
Running the Video Publisher Node

To publish video frames to the MOT node from a specified video file:
ros2 run mot_tracking_package video_publisher
Recording and Playing Back Data with ROS 2 Bag

To record tracking data:
ros2 bag record /camera/image_raw /tracking_data
To play back recorded data:

ros2 bag play <bag_file_directory>
Configuration

You can adjust the MOT and video publisher nodes' parameters (e.g., video file path, detection threshold) by modifying the corresponding Python scripts or through ROS 2 parameter files.
Acknowledgments

    Object Detection and Tracking algorithms: YOLOv4, ByteTrack
    ROS 2 community for guidance and tools
    [Optional] Mention any specific individuals, organizations, or projects

License

This project is licensed under the MIT - see the LICENSE.md file for details.
