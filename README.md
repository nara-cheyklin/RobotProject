🤖 Hand-Gesture-Controlled Autonomous Robot Navigation

ROS 2 | SLAM | Nav2 | Computer Vision | Human-Robot Interaction


📌 Project Summary

This project integrates autonomous mobile robot navigation with real-time hand gesture control, enabling intuitive human-robot interaction in shared environments.
Instead of using a keyboard or joystick, users visually select waypoints in RViz and control execution (start, pause, resume, stop) through dynamic hand gestures detected via computer vision.


The system demonstrates advanced integration of:
- ROS 2 distributed architecture
- SLAM-based mapping
- Autonomous path planning
- Action server communication
- Real-time perception with MediaPipe

🎯 Motivation

This project aims to:
- Combine autonomous navigation
- Enable natural, human-in-the-loop control
- Improve usability in shared environments

🏗️ Hardware Architecture

The system is built using:
- TurtleBot3 Burger
- OpenCR Control Board
- Raspberry Pi 4 (On-board computer)
- LDS-02 LiDAR
- Dynamixel XL430-W250 motors
- USB Web Camera

🧠 System Architecture

Core Concept:
- User selects navigation goals visualized in RViz
- Hand gestures control execution behavior

Distributed Architecture
-TurtleBot3
- Sensing
- SLAM
- Navigation
- External PC
- Vision-heavy gesture recognition
- ROS 2

💻 Software Stack
- Operating Environment
- Ubuntu Linux (Robot & PC)
- ROS 2 Humble (DDS-based middleware)

✋ Hand Gesture Recognition (MediaPipe)
 MediaPipe Hands Framework
- Detects 21 hand landmarks
- Provides normalized 2D joint coordinates
- Robust to hand orientation and scaling

Gesture Commands:
- 1 finger	Start navigation
- 2 fingers	Pause current goal
- 4 fingers	Continue navigation
- 5 fingers	Stop & cancel goals

ROS Integration
- Camera → /image_raw/compressed
- Gesture Node → /gesture_command
- Navigation node reacts in real time

📦 Custom ROS 2 Nodes
- hand_gesture_node
- multi_point_nav_node

Custom launch file combines:
- navigation2.launch.py
- multi_point_nav_node
- hand_gesture_node

🔄 System Workflow
- Build map using SLAM
- Save occupancy grid
- Load map in Nav2
- Select 3 waypoints in RViz
- Use gestures to: Start / Pause / Resume
- Stop navigation

🎥 Video Demonstration
https://youtu.be/Uw6x6yV6_6k?si=UW3o-mwzbli-Bdvc

📚 Key Learning Outcomes
- ROS 2 distributed system design
- SLAM implementation
- Autonomous path planning
- Action server communication
- Real-time computer vision integration
- Human-robot interaction design

🔮 Future Improvements
- Add dynamic gesture classification (ML-based)
- Voice command integration
- Obstacle-aware gesture overrides
- Performance optimization on embedded system

📄 Conclusion
This project demonstrates the integration of:
- Autonomous mobile robotics
- Real-time perception
- Human-in-the-loop control
