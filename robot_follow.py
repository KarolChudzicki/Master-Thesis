import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import gripper


class robotFollow:
    def __init__(self, camera_instance):
        self.camera = camera_instance
        self.receiver = URData()
        self.HOME = [-0.1, 0.766, 0.1, 0, 3.1415, 0]

        self.max_X = 0.1
        self.min_X = -0.5
        self.max_Y = 0.9
        self.min_Y = 0.7
        self.max_Z = 0.2
        self.min_Z = -0.1

    def is_within_bounds(self, position):
        x, y, z = position
        
        result = [
        1 if x < self.min_X else (-1 if x > self.max_X else 0),
        1 if y < self.min_Y else (-1 if y > self.max_Y else 0),
        1 if z < self.min_Z else (-1 if z > self.max_Z else 0)]

        return np.array(result)

    def move_to(self, part_number):
        pose_from = self.receiver.get_pose()  # Current robot pose (XYZ + orientation)
        camera_pose, area = self.camera.capture_and_get_coords(part_number)  # Detected part coords (XYZ)

        # Handle case where camera couldn't detect part
        if np.all(camera_pose == 0):
            return [0, 0, 0, 0, 0, 0]

        # Target pose: use current X/Y from camera, and a fixed Z (gripper height)
        pose_to = pose_from[:3] - camera_pose[:3]
        pose_to[2] = self.min_Z  # Set a fixed gripper Z height for grabbing

        # Compute motion vector
        direction = pose_to - pose_from[:3]
        distance = np.linalg.norm(direction)

        # Avoid division by zero
        if distance < 1e-6:
            return [0, 0, 0, 0, 0, 0]

        # Adaptive speed based on distance
        max_speed = 0.5
        min_speed = 0.01
        k = 5
        speed = np.clip(k * distance, min_speed, max_speed)

        # Final velocity vector (XYZ only)
        velocity = direction / distance * speed
        
        velocity = velocity.astype(float).tolist()
        # Avoid moving too far down (prevent crashing into table)
        if pose_from[2] <= -0.095:
            velocity[2] = 0

        velocity_vector = [-velocity[0], velocity[1], velocity[2], 0, 0, 0]  # Adjust sign if necessary

        # Safety check: avoid going out of bounds
        escape_vector = self.is_within_bounds(pose_from[:3])
        if not np.all(escape_vector == 0):
            adjusted_escape = (escape_vector.T * np.array([0.1, 0.1, 0.1]))
            adjusted_escape = adjusted_escape.astype(float).tolist()
            return [adjusted_escape[0], adjusted_escape[1], adjusted_escape[2], 0, 0, 0]

        return velocity_vector





