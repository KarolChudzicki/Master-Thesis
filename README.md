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

## Repository structure

Master-Thesis/
├── main.py                     # Main application
├── robot_control.py            # Robot motion and control
├── camera.py                   # Image acquisition and processing
├── conveyor_belt.py            # Conveyor communication and control
├── gripper.py                  # Robotiq gripper control
├── gui.py                      # User interface
│
├── Calibration/                # Camera calibration
├── calibration_images/         # Camera calibration dataset
├── calibration_images_hand_eye/# Hand-eye calibration dataset
├── Plots/                      # Experimental results and plots
└── Thesis/                     # Master's thesis 3D designs/photos/documenations
