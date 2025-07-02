import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
import urData
import gripper
import math
from scipy.spatial.transform import Rotation as R
import threading

gripper = gripper.Gripper()

gripper.activate()
gripper.connect()






URRobot = ur.URRobot()

URReceiver = urData.URData()


class robotControl:
    def __init__(self, camera_instance):
        self.camera = camera_instance
        # 0.081 because then the camera is exactly 30cm from the conveyor
        self.HOME1 = [0.1, 0.766, 0.081, 3.1415, 0, 0]
        self.HOME2 = [-0.1, 0.766, 0.081, 3.1415, 0, 0]

        # Tray positions
        pose_above = 0.2
        self.DROP1 = [-0.625, 0.035, 0.063, 3.0352, -0.0244, -0.7915]

        self.DROP1_ABOVE = self.DROP1.copy()
        self.DROP1_ABOVE[2] = pose_above

        shift_for_drop2 = 0.07025
        self.DROP2 = self.DROP1.copy()
        self.DROP2[1] += shift_for_drop2
        self.DROP2_ABOVE = self.DROP2.copy()
        self.DROP2_ABOVE[2] = pose_above

        shift_for_drop3 = 0.05775
        self.DROP3 = self.DROP2.copy()
        self.DROP3[1] += shift_for_drop3
        self.DROP3_ABOVE = self.DROP3.copy()
        self.DROP3_ABOVE[2] = pose_above


        self.max_X = 0.1
        self.min_X = -0.5
        self.max_Y = 0.9
        self.min_Y = 0.7
        self.max_Z = 0.2
        self.min_Z = -0.1
        self.coords_array = []
        self.time_stamps = []
        
        self.catched = False
        
        self.indicator_array = [0,0,0,0,0,0]
        

        
    def move_home(self, time_to):
        URRobot.movel(self.HOME1, 0.1, 0.1, time_to)
        
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
        max_speed_x = -0.04
        
        max_speed_y = 0.01
        
        x_speed = initial_speed
        y_speed = 0
        
        slowdown_radius = 0.08 # 8cm
        x_threshold_distance = 0.002 # 2mm
        y_threshold_distance = 0.0005 # 1mm
        
        kx = 0.2
        ky = 1
        
        
        while True:
            pose = URReceiver.get_pose()  # Current robot pose (XYZ + orientation)
            
            coords, area, frame, angle = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)
            coords[0] -= 0.01 # to account for delays
            coords = np.array(coords)
            
            pose_differenceXY = coords[:2]

            x_distance = pose_differenceXY[0]
            y_distance = pose_differenceXY[1]

            # Update X speed
            if abs(x_distance) > slowdown_radius:
                x_speed = initial_speed + max_speed_x
                x_speed = float(x_speed)
            elif abs(x_distance) > x_threshold_distance:
                # Quadratic easing: slows down more gently as we approach
                normalized_dist = (abs(x_distance) - x_threshold_distance) / (slowdown_radius - x_threshold_distance)
                slow_factor = normalized_dist ** 0.5  # smoother slowdown
                x_speed = initial_speed + max_speed_x * slow_factor
                x_speed = float(x_speed)
            else:
                delta_speed = x_speed - 0.0325
                if abs(delta_speed) > 0.001:
                    steps = 10
                    for i in range(steps):
                        x_speed = x_speed - delta_speed/steps
                        velocity_vector = [x_speed, 0, 0, 0, 0, 0]
                        URRobot.speedl(velocity_vector, 0.5, 0.5)
                        
                        
                velocity_vector = [initial_speed, 0, 0, 0, 0, 0]
                URRobot.speedl(velocity_vector, 0.5, 3)
                break
            
            
            # Update Y speed with correct direction (signed)
            
            if abs(y_distance) > y_threshold_distance:
                y_speed = ky * y_distance
                # Clamp y_speed to [-max_speed_y, max_speed_y]
                y_speed = -max(-max_speed_y, min(y_speed, max_speed_y))
                y_speed = float(y_speed)
            else:
                y_speed = y_speed * 0.05
                if abs(y_speed) < 1e-6:
                    y_speed = 0
                y_speed = float(y_speed)
                    

            print("Y distance, speed: ",y_distance , y_speed)
            #Safety check: avoid going out of bounds
            
            escape_vector = self.is_within_bounds(pose[:3])
            if not np.all(escape_vector == 0):
                adjusted_escape = (escape_vector.T * np.array([0.1, 0.1, 0.1]))
                adjusted_escape = adjusted_escape.astype(float).tolist()
                x_speed = adjusted_escape[0]
                y_speed = adjusted_escape[1]

            
            
            velocity_vector = [x_speed, y_speed, 0, 0, 0, 0]
            #print("Vel vector: ",velocity_vector)
            URRobot.speedl(velocity_vector, 0.2, 0.5)
            
        
        
        return velocity_vector

        
    def descend_and_grab(self, velocity_vector, angle, part_number):
        x_speed = velocity_vector[0]
        y_speed = velocity_vector[1]
        max_speed_z = -0.05
        max_speed_rz = 0.2
        descend_height = -0.102  # target height, e.g. 7cm
        slowdown_radius = 0.03  # 3cm slowdown radius
        slowdown_angle = 0.5
        
        rz_speed = 0.0
        z_speed = 0.0
        
        ramp_rate_z = 0.001
        ramp_rate_rz = 0.0005
        
        # To prevent jumps
        if abs(angle) > 88:
            angle = 0
        elif angle > 0:
            angle -= 90
        elif angle < 0:
            angle += 90
        target_angle_rad = math.radians(angle)
        
        
        last_rotation_time = time.time()
        rz_speed = 0
        rz_pose = 0
        
        #threading.Thread(target=image_show, daemon=True).start()
        
        while True:
            pose = URReceiver.get_pose()  # current z position
            z_pose = pose[2]
        
            # Calculate vertical distance above descend height, but never negative
            z_distance = max(z_pose - descend_height, 0)

            # Normalize distance in slowdown zone [0..1]
            normalized_dist = np.clip(z_distance / slowdown_radius, 0.0, 1.0)

            # Quadratic easing to slow down smoothly as we approach target
            slow_factor = normalized_dist ** 0.5

            # Compute z_speed with slowing down
            z_speed_target = float(max_speed_z * slow_factor)
            
            # Smooth ramp for z_speed
            if abs(z_speed_target - z_speed) < ramp_rate_z:
                z_speed = z_speed_target
            elif z_speed_target > z_speed:
                z_speed += ramp_rate_z
            else:
                z_speed -= ramp_rate_z
            
            
            # Rotation speed
            current_rotation_time = time.time()
            dt = current_rotation_time - last_rotation_time
            last_rotation_time = current_rotation_time
            rz_pose -= rz_speed * dt
            
            
            
            
            
            if math.isnan(rz_pose):
                logging.info("Rz pose error, returning home")
                self.move_home(4)
                return None
            
            else: 
                rz_rotation = rz_pose - target_angle_rad
                normalized_rotation = np.clip(rz_rotation / slowdown_angle, -1.0, 1.0)
                slow_factor_angle = abs(normalized_rotation) ** 0.5 * np.sign(normalized_rotation)
            
                if abs(rz_rotation) < 0.01:
                    rz_speed = rz_speed * 0.1
                    if abs(rz_speed) < 1e-6:
                        rz_speed = 0
                else:
                    rz_speed_target = float(max_speed_rz * slow_factor_angle)
                    
                    if abs(rz_speed_target - rz_speed) < ramp_rate_rz:
                        rz_speed = rz_speed_target
                    elif rz_speed_target > rz_speed:
                        rz_speed += ramp_rate_rz
                    else:
                        rz_speed -= ramp_rate_rz
                        
                

                velocity_vector = [x_speed, y_speed, z_speed, 0, 0, rz_speed]
                #print("Vel vector descending: ",velocity_vector)
                URRobot.speedl(velocity_vector, 0.2, 0.5)

                # If within 1mm from target height, stop
                if z_distance < 0.001:
                    z_speed = 0
                    velocity_vector = [0, 0, 0, 0, 0, 0]
                    URRobot.speedl(velocity_vector, 0.2, 5)
                    if part_number == 0:
                        gripper.open_close(50, 100, 1)
                    elif part_number == 1:
                        gripper.open_close(46, 100, 1)
                    elif part_number == 2:
                        gripper.open_close(55, 100, 1)
                    time.sleep(0.5)
                    return 0
                
                

           
    def move_part_away(self, part_number):
        
        DROP_ABOVE = [self.DROP1_ABOVE, self.DROP2_ABOVE, self.DROP3_ABOVE]
        DROP = [self.DROP1, self.DROP2, self.DROP3]
        
        INDICES = [
            (3, 0),  # for part 0
            (4, 1),  # for part 1
            (5, 2)   # for part 2
        ]
        
        above = DROP_ABOVE[part_number]
        target = DROP[part_number]
        idx1, idx2 = INDICES[part_number]
        
        POSE = URReceiver.get_pose()
        POSE[2] += 0.3
        # Move above the conveyor
        URRobot.movel(POSE, 0.5, 0.3, 2)
        
        if self.indicator_array[idx1] == 0:
            self.indicator_array[idx1] = 1
            self.move_and_drop(above, target)
        elif self.indicator_array[idx2] == 0:
            self.indicator_array[idx2] = 1
            self.move_and_drop(above, target)
        else:
            logging.info(f"Storage for part {part_number + 1} full !!!")
            
            
        required_parts = [self.indicator_array[3], self.indicator_array[4], self.indicator_array[5]]       
        
        if all(required_parts):
            print("We have all parts")
        
        self.move_home(5)
        
        
        
        return self.indicator_array

    def move_and_drop(self, above, target):
        URRobot.movel(above, 0.5, 0.3, 3)
        URRobot.movel(target, 0.2, 0.2, 3)
        gripper.open_close(85, 100, 1)
        time.sleep(0.5)
        URRobot.movel(above, 0.2, 0.2, 3)


    def assemble_product(self):
        pass