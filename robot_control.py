import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import gripper

gripper = gripper.Gripper()

gripper.activate()
gripper.connect()

# 0.081 because then the camera is exactly 30cm from the conveyor
HOME1 = [0.1, 0.766, 0.081, 3.1415, 0, 0]
HOME2 = [-0.1, 0.766, 0.081, 3.1415, 0, 0]

URRobot = ur.URRobot()



class robotControl:
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
        
        self.catched = False
        
        
    def move_home(self):
        URRobot.movel(HOME1, 0.01, 0.05, 5)
        
    def rough_estimation(self, part_number):
        parts_area = [40000, 27000, 53000]
        
        
        
        coords, area, _, angle = self.camera.capture_and_get_coords_center(part_number)
        coords = coords[:2]
        
        if part_number == 0 and area > parts_area[0]:
            self.coords_array.append(coords[0])
            self.time_stamps.append(time.time())
        if part_number == 1 and area > parts_area[1]:
            self.coords_array.append(coords[0])
            self.time_stamps.append(time.time())
        if part_number == 2 and area > parts_area[2]:
            self.coords_array.append(coords[0])
            self.time_stamps.append(time.time())
        
        
        
        if len(self.coords_array) >= 20:
            velocities = []
            for i in range(5, len(self.coords_array)):
                delta_pos = self.coords_array[i] - self.coords_array[i - 1]
                delta_time = self.time_stamps[i] - self.time_stamps[i - 1]
                if delta_time > 0:
                    velocity = delta_pos / delta_time  # [vx, vy]
                    velocities.append(velocity)
            
            avg_velocity = np.mean(velocities, axis=0)
            avg_velocity_x = round(float(avg_velocity),7)
            
            avg_velocity2 = (self.coords_array[5]-self.coords_array[-1])/(self.time_stamps[5]-self.time_stamps[-1])
            avg_velocity2 = round(avg_velocity2,7)
            
            last_coords = self.coords_array[-1]
            self.coords_array = []
            self.time_stamps = []
            
            
            
            # Return average x velocity
            return avg_velocity_x, avg_velocity2, last_coords
        
        return None, None, None           


    def is_within_bounds(self, position):
        x, y, z = position
        
        result = [
        1 if x < self.min_X else (-1 if x > self.max_X else 0),
        1 if y < self.min_Y else (-1 if y > self.max_Y else 0),
        1 if z < self.min_Z else (-1 if z > self.max_Z else 0)]

        return np.array(result)


    def follow_part(self, part_number, initial_speed, last_position_time):
        
        # Configuration values
        max_speed_x = -0.025
        
        max_speed_y = 0.01
        min_speed_y = 0.001
        
        x_speed = initial_speed
        
        slowdown_radius = 0.09 # 9cm
        x_threshold_distance = 0.005 # 5mm
        y_threshold_distance = 0.002 # 5mm
        
        kx = 0.2
        ky = 3
        
        last_pose = self.receiver.get_pose()
        
        while True:
            #predicted_ditance = (time.time() - last_position_time) * initial_speed
            #predicted_position = [last_pose[0] + predicted_ditance, last_pose[1], last_pose[2]]
            coords, area, frame, angle = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)
            coords = np.array(coords)
            
            pose_differenceXY = coords[:2]

            x_distance = pose_differenceXY[0]
            y_distance = -pose_differenceXY[1]

            # Update X speed
            if abs(x_distance) > slowdown_radius:
                x_speed = initial_speed + max_speed_x
            elif abs(x_distance) > x_threshold_distance:
                # Quadratic easing: slows down more gently as we approach
                normalized_dist = (abs(x_distance) - x_threshold_distance) / (slowdown_radius - x_threshold_distance)
                slow_factor = normalized_dist ** 2  # smoother slowdown
                x_speed = initial_speed + max_speed_x * slow_factor
                
            else:
                print("Last speed", x_speed)
                delta_speed = x_speed - initial_speed
                if abs(delta_speed) > 0.001:
                    steps = 5
                    for i in range(steps):
                        x_speed = x_speed + delta_speed/5
                        velocity_vector = [x_speed, 0, 0, 0, 0, 0]
                        URRobot.speedl(velocity_vector, 0.5, 10)
                        time.sleep(0.01)
                        
                velocity_vector = [initial_speed, 0, 0, 0, 0, 0]
                URRobot.speedl(velocity_vector, 0.5, 10)
                print("Finished Maintain current", x_speed)
                break
            
            
            # Update Y speed with correct direction (signed)
            print(pose_differenceXY)
            if abs(y_distance) > y_threshold_distance:
                if y_distance > 0:
                    y_speed = min(ky * y_distance, max_speed_y)
                else:
                    y_speed = max(ky * y_distance, -max_speed_y)
            else:
                y_speed = 0
                    
            print(x_speed, y_speed)
            
            #Safety check: avoid going out of bounds
            pose = self.receiver.get_pose()  # Current robot pose (XYZ + orientation)
            escape_vector = self.is_within_bounds(pose[:3])
            if not np.all(escape_vector == 0):
                adjusted_escape = (escape_vector.T * np.array([0.1, 0.1, 0.1]))
                adjusted_escape = adjusted_escape.astype(float).tolist()
                x_speed = adjusted_escape[0]
                y_speed = adjusted_escape[1]

            velocity_vector = [x_speed, y_speed, 0, 0, 0, 0]
            URRobot.speedl(velocity_vector, 0.2, 15)
        
        
        return initial_speed

        
    def descend_and_grab(self, velocity_vector, angle):
        descending_vector = velocity_vector
        last_pose = self.receiver.get_pose()
        while True:
        
            URRobot.speedl(descending_vector, 0.2, 15)

