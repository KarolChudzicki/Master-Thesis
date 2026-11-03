# Vision-Guided Robotic System for Real-Time Object Tracking and Handling

This project was developed as part of my master thesis at Technical University of Denmark. The objective was to design a robotic arm to pick up different objects from a conveyor belt using only input from a camera mounted to the end effector of the robotic arm.

## Overview

This thesis describes a robotic system consisting of:
- **UR10 Cobot** - for pick-and-place operations,
- **Webcam (1920 x 1080)** - used for visual input,
- **Conveyor belt** - supplied various parts for the robotic arm to handle,
- **PC** - runs image processing, controls the robot and handles communication,
- **Robotiq Gripper** - grasps objects from the conveyor belt.


The project was developed primarily in Python, with OpenCV used for computer vision.

## Repository Structure

```text
Master-Thesis/
├── 3D files/                     # 3D files used for the project
├── Calibration/                  # Camera and hand-eye calibration tools
├── Plots/                        # Plots with results
├── calibration_images/           # Camera calibration images
├── calibration_images_hand_eye/  # Hand-eye calibration images
│
├── main.py                       # Main application entry point
├── camera.py                     # Image acquisition and processing
├── robot_control.py              # Robot movement control
├── conveyor_belt.py              # Conveyor belt communication and control
├── gripper.py                    # Robotiq gripper control
├── gui.py                        # Graphical user interface
├── ur.py                         # UR commands used for RTDE
├── urData.py                     # UR robot data handling
├── misc.py                       # Utility functions
│
├── calib_param.txt               # Camera calibration parameters
├── calib_param_hand_eye.txt      # Hand-eye calibration parameters
├── slider_params_*.json          # Image-processing parameters
└── README.md                     # Project documentation
```
