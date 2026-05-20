# ROS2-UGV-1-10-Platform
ROS 2 modular architecture for a 1/10 autonomous RC car. Supports indoor vision-based track following and outdoor GPS waypoint navigation. Designed with separate sensing, navigation, control, and state management layers for clean structure, scalability, and easy experimentation.


**🚗 1/10 Scale Autonomous RC Vehicle – ROS 2**
📌 Overview

This repository contains the software architecture for a 1/10 scale autonomous RC vehicle built using ROS 2.

The platform is designed to operate in both:

🏠 **Indoor environments** using camera-based vision for track following
🌍 **Outdoor environments** using GPS-based waypoint navigation

The system follows a modular and layered robotics architecture to ensure scalability, maintainability, and clear separation of responsibilities.

🏗️ System Architecture

The software stack is organized into independent functional layers:

**Sensors → Perception → Navigation → Control → Actuation
                   ↘
                 State Manager**
Core Modules
Package	Responsibility
rc_drivers	Hardware abstraction (camera, GPS, IMU, ESC, steering)
rc_perception	Vision processing and feature extraction
rc_navigation_indoor	Vision-based track following
rc_navigation_outdoor	GPS waypoint navigation
rc_control	Low-level steering and throttle control
rc_state_manager	Mode switching (indoor / outdoor)
rc_bringup	Launch files and system configuration
rc_description	URDF model and TF tree
🏠 Indoor Mode

**Indoor navigation relies on camera-based perception.**

Pipeline:

Camera → Vision Processing → Path Estimation → Control

Typical outputs:

Lane/track center offset
Direction estimation
Confidence metric
🌍 Outdoor Mode

**Outdoor navigation uses GPS-based waypoint tracking.**

Pipeline:

GPS + IMU → Waypoint Tracking → Heading & Speed Command → Control

Features:

Waypoint following
Heading correction
Mode fallback support
🔀 State Management

**A dedicated state manager handles mode switching:
**
Indoor mode (vision-based)
Outdoor mode (GPS-based)
Safe stop fallback

The control layer receives unified velocity and steering commands regardless of active mode.

⚙️ **Technologies**
ROS 2
C++ / Python
OpenCV (for vision)
GPS module
Raspberry Pi (onboard compute)
PWM-based servo & ESC control
🧠 **Design Principles**
Modular ROS 2 packages
Clear separation between perception, navigation, and control
Hardware abstraction layer
No hard-coded parameters (ROS parameters used)
Clean topic-based communication
Scalable for future extensions (e.g., SLAM, obstacle avoidance)
🚀 **Running the System**
colcon build
source install/setup.bash
ros2 launch rc_bringup full_system.launch.py
📈 Future Improvements
Sensor fusion (GPS + IMU EKF)
Obstacle detection
Dynamic speed adaptation
Behavior tree–based decision layer
Multi-robot scalability
