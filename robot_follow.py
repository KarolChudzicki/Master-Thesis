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
        self.coords_array = []
        self.time_stamps = []
        
    def rough_estimation(self, part_number):
        parts_area = [37000, 46000, 0]
        
        
        coords, area, _ = self.camera.capture_and_get_coords(part_number)  # Detected part coords (XYZ)
        coords = coords[:2]
        
        if part_number == 0 and area > parts_area[0]:
            self.coords_array.append(coords)
            self.time_stamps.append(time.time())
        if part_number == 1 and area > parts_area[1]:
            self.coords_array.append(coords)
            self.time_stamps.append(time.time())
        if part_number == 2 and area > parts_area[2]:
            self.coords_array.append(coords)
            self.time_stamps.append(time.time())
        
        
        
        if len(self.coords_array) > 30:
            velocities = []
            for i in range(1, len(self.coords_array)):
                delta_pos = self.coords_array[i] - self.coords_array[i - 1]
                delta_time = self.time_stamps[i] - self.time_stamps[i - 1]
                if delta_time > 0:
                    velocity = delta_pos / delta_time  # [vx, vy]
                    velocities.append(velocity)
            
            avg_velocity = np.mean(velocities, axis=0)
            avg_velocity_x = round(float(avg_velocity[0]),5)
            
            # Return average x velocity
            return avg_velocity_x, self.coords_array[-1]
            

        


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
        print(distance)

        # # Avoid division by zero
        # if distance < 1e-6:
        #     return [0, 0, 0, 0, 0, 0]

        # Adaptive speed based on distance
        max_speed = 0.3
        min_speed = 0.01
        k = 3
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





