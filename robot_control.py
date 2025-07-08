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
import csv
import os
from datetime import datetime

gripper = gripper.Gripper(port='COM5')
gripper.activate()



URRobot = ur.URRobot()

URReceiver = urData.URData()


class robotControl:
    def __init__(self, camera_instance):
        self.camera = camera_instance
        # 0.081 because then the camera is exactly 30cm from the conveyor
        self.HOME1 = [0.15, 0.766, 0.081, 3.1415, 0, 0]
        self.HOME2 = [-0.1, 0.766, 0.081, 3.1415, 0, 0]

        # Tray positions
        pose_above = 0.2
        self.DROP1 = [-0.625, 0.035, 0.0617, 3.0383, -0.0147, -0.7923]

        self.DROP1_ABOVE = self.DROP1.copy()
        self.DROP1_ABOVE[2] = pose_above

        shift_for_drop2 = 0.066
        self.DROP2 = self.DROP1.copy()
        self.DROP2[1] += shift_for_drop2
        self.DROP2_ABOVE = self.DROP2.copy()
        self.DROP2_ABOVE[2] = pose_above

        shift_for_drop3 = 0.063
        self.DROP3 = self.DROP2.copy()
        self.DROP3[1] += shift_for_drop3
        self.DROP3_ABOVE = self.DROP3.copy()
        self.DROP3_ABOVE[2] = pose_above
        
        # Pick and assembly
        tilted = 25 #Tray for pickup is tilted 25 degrees
        self.PICK1 = [-0.545, 0.0365, 0.018, 3.0613, -0.0147, -0.7]
        x_new = (pose_above - self.PICK1[2]) * math.tan(math.radians(tilted))
        self.PICK1_ABOVE = self.PICK1.copy()
        self.PICK1_ABOVE[0] += x_new
        self.PICK1_ABOVE[2] = pose_above
    


        self.max_X = 0.3
        self.min_X = -0.7
        self.max_Y = 0.9
        self.min_Y = 0.7
        self.max_Z = 0.2
        self.min_Z = -0.1
        self.coords_array_x = []
        self.coords_array_y = []
        self.time_stamps = []
        
        self.catched = False
        
        self.indicator_array = [0,0,0,0,0,0]
        
        
        # ================= PARAMETERS FOR CONTROLLER =================
        self.integral = 0
        self.derivative = 0
        self.last_time = None
        self.previous_error = None
        self.Kp = 1.1
        self.Ki = 0.1
        self.Kd = 0.1
        self.Kff = 0.0
        self.stablization_points = 0
        self.start_time_log = 0
        
        self.data_log = []

    def save_log_csv(self, log_data, folder="Plots/data"):
        # Create the folder if it doesn't exist
        os.makedirs(folder, exist_ok=True)

        # Create a unique filename with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"pid_log_{timestamp}.csv"
        filepath = os.path.join(folder, filename)

        # Save the log
        if log_data:
            keys = log_data[0].keys()
            with open(filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(log_data)
            print(f"[LOG] PID data saved to: {filepath}")
        else:
            print("[LOG] No data to save.")
        
    def move_home(self, time_to):
        URRobot.movel(self.HOME1, 0.1, 0.1, time_to)
        
    def move_drop1(self, time_to):
        gripper.open_close(40, 10, 1)
        time.sleep(5)
        URRobot.movel(self.DROP1_ABOVE, 0.1, 0.1, time_to)
        URRobot.movel(self.DROP1, 0.1, 0.1, time_to)
        
    def rough_estimation(self, part_number):
        parts_area = [34000, 22000, 48000]
        incorrect_readings = 0
        max_incorrect_readings = 20
        coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
        
        min_readings = 7
        
        while incorrect_readings < max_incorrect_readings:
            while area > parts_area[part_number]:
                coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
                #print("Coords", coords)
                self.coords_array_x.append(coords[0])
                self.coords_array_y.append(coords[1])
                self.time_stamps.append(time.time())
            incorrect_readings += 1
        
        array_length = len(self.coords_array_x)
        if array_length >= min_readings:
            delta_x = self.coords_array_x[2]-self.coords_array_x[array_length - 2]
            delta_t = self.time_stamps[2]-self.time_stamps[array_length - 2]
            avg_velocity = delta_x/delta_t
            avg_velocity = round(avg_velocity,7)
            avg_pose_y = np.mean(self.coords_array_y[:(array_length - 1)])
            last_coords = [self.coords_array_x[array_length - 2], float(avg_pose_y)]
            last_coords_robot = URReceiver.get_pose()[:2]
            last_coords_time = self.time_stamps[array_length - 2]
            self.coords_array_x = []
            self.coords_array_y = []
            self.time_stamps = []
            
        
            if abs(avg_velocity) > 0.02:
                return avg_velocity, last_coords, last_coords_time, last_coords_robot
            else:
                print("Average velocity too slow")
                self.coords_array_x = []
                self.coords_array_y = []
                self.time_stamps = []
                return None, None, None, None
        
        else:
            # Too few readings
            print("Too few readings to calculate average velocity")
            self.coords_array_x = []
            self.coords_array_y = []
            self.time_stamps = []
            return None, None, None, None


    def is_within_bounds(self, position):
        x, y, z = position
        
        result = [
        1 if x < self.min_X else (-1 if x > self.max_X else 0),
        1 if y < self.min_Y else (-1 if y > self.max_Y else 0),
        1 if z < self.min_Z else (-1 if z > self.max_Z else 0)]

        return np.array(result)


    def follow_part(self, part_number, initial_speed, last_coords, last_coords_time, last_coords_robot):
        self.start_time_log = time.time()
        # Max speed x so the robot can catch the part within 1 second
        #max_speed_x = initial_speed * 3
        max_speed_x = initial_speed + (last_coords[0] + initial_speed * (time.time() - last_coords_time)) / 2
        #print("Max speed: ", max_speed_x, last_coords[0] + initial_speed * (time.time() - last_coords_time))
        
        # Max speed y (1 second to cover the delta y) but not less than 5mm/s
        max_speed_y = min(abs(last_coords[1])/2, 0.015)
        min_speed_y = 0.00025
        direction_y = np.sign(last_coords[1])
        
        #print("Initial speed:", initial_speed, max_speed_x, max_speed_y)
        
        x_speed = initial_speed
        y_speed = 0
        
        y_threshold_distance = 0.003 # 3mm
        
        x_speed_goal = False
        y_speed_goal = False
        
        smooth_points_y_start = 5
        smooth_points_y_stop = 5
        
        alpha = 0.9

        while True:
            pose = URReceiver.get_pose()  # Current robot pose (XYZ + orientation)
            coords, area, frame, angle = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)
            coords = np.array(coords)
                
            # Predicted pose when the part is not visible based on inital speed
            x_distance_predicted = (last_coords[0] + initial_speed*(time.time() - last_coords_time)) - (pose[0] - last_coords_robot[0])
            
            y_distance = coords[1]
            
            if coords is not [0,0,0] and area > 20000:
                x_distance = alpha * x_distance_predicted + (1 - alpha) * coords[0]
                y_distance = alpha * last_coords[1] + (1 - alpha) * coords[1]
                alpha -= 0.05
                if alpha <= 0:
                    alpha = 0
                print("Real camera feed", alpha)
            else:
                alpha = 0.9
                x_distance = x_distance_predicted
                y_distance = last_coords[1]
                print("Predicting")
                
            print("X and Y distances: ", x_distance, x_distance_predicted, y_distance, last_coords[1])
            
            # Update X speed using a controler
            if x_speed_goal is False:
                x_speed, x_speed_goal = self.decelerate_to_target(x_distance, x_speed, max_speed = max_speed_x, min_speed=initial_speed/2)
                if x_speed_goal is True:
                    # Controller stopped, save data to a log file
                    self.save_log_csv(log_data=self.data_log)
                    
            
        
            if abs(y_distance) > y_threshold_distance and y_speed_goal is False:
                if smooth_points_y_start > 0:
                    smooth_points_y_start -= 1
                    y_speed = min_speed_y * -direction_y
                else:
                    y_speed = np.clip(-y_distance, -max_speed_y, max_speed_y) 
                        
            else:
                y_speed_goal = True
                if smooth_points_y_stop > 0:
                    smooth_points_y_stop -= 1
                    y_speed = min_speed_y * -direction_y
                else:
                    print("Y decelerated")
                    y_speed_goal = True
                    y_speed = 0    
                        
                        
            y_speed = float(y_speed)
                
            
            escape_vector = self.is_within_bounds(pose[:3])
            if not np.all(escape_vector == 0):
                adjusted_escape = (escape_vector.T * np.array([0.1, 0.1, 0.1]))
                adjusted_escape = adjusted_escape.astype(float).tolist()
                x_speed = adjusted_escape[0]
                y_speed = adjusted_escape[1]

            velocity_vector = [x_speed, y_speed, 0, 0, 0, 0]
            #print("Vel vector chasing: ",velocity_vector, coords)
            URRobot.speedl(velocity_vector, 0.3, 0.5)
            
            if x_speed_goal and y_speed_goal:
                break
        
        return velocity_vector
    
    
    def decelerate_to_target(self, distance_to_target, initial_speed, max_speed, min_speed):

        direction = np.sign(distance_to_target)
        current_time = time.time()
        if self.last_time is None:
            dt = 0.01
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        
        # I
        error = distance_to_target
        self.integral += error * dt
        
        # D
        if self.previous_error is None:
            self.previous_error = error
        else:
            self.derivative = (error - self.previous_error) / dt
            self.previous_error = error
        
        # PID controller speed output
        speed = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative + self.Kff * initial_speed

        # Clamp speed to max/min speed limits
        #direction = np.sign(initial_speed)
        speed = max(speed, max_speed)
        speed = float(speed)
        
        #print("Controller:", error, self.integral, self.derivative, speed)
        
        
        # Log data
        self.data_log.append({
            "time": current_time,
            "error": error,
            "x_speed": speed,
            "p_term": self.Kp * error,
            "i_term": self.Ki * self.integral,
            "d_term": self.Kd * self.derivative
        })
        
        
        self.stablization_points += (abs(error) < 0.02) # self.Kd * self.derivative < 0.005
        
        if self.stablization_points > 10:
            self.integral = 0
            self.derivative = 0
            self.last_time = None
            self.previous_error = None
            at_target = True
        else:
            at_target = False
        
        
        return speed, at_target
        
    def descend_and_grab(self, velocity_vector, angle, part_number):
        x_speed = velocity_vector[0]
        max_speed_z = -0.2
        max_speed_rz = 0.25
        descend_height = -0.105  # target height -10.3cm
        slowdown_angle = 0.5
        
        rz_speed = 0.0
        z_speed = 0.0
        
        ramp_rate_z = 0.001
        ramp_rate_rz = 0.0025
        
        # Total z distance:
        pose = URReceiver.get_pose()  # current z position
        z_pose = pose[2]
        
        # Calculate vertical distance above descend height, but never negative
        z_half = (z_pose - descend_height) / 2
        
        # To prevent jumps
        if angle > 0:
            angle -= 90
        elif angle < 0:
            angle += 90
        target_angle_rad = math.radians(angle)
        
        last_rotation_time = time.time()
        rz_speed = 0
        rz_pose = 0
        
        smooth_points_z_start = 5
        smooth_points_z_stop = 5
        smooth_points_rz_start = 5
        smooth_points_rz_stop = 5
        
        while True:
            pose = URReceiver.get_pose()  # current z position
            z_pose = pose[2]
        
            # Calculate vertical distance above descend height, but never negative
            z_distance = max(z_pose - descend_height, 0)

            if z_distance > z_half:
                if smooth_points_z_start > 0:
                    smooth_points_z_start -= 1
                    z_speed = -0.0001
                else:
                    z_speed -= ramp_rate_z
                    z_speed = max(z_speed, max_speed_z)
                    z_speed_achieved = z_speed
            elif z_distance > 0.002:
                # Normalize distance in slowdown zone [0..1]
                normalized_dist = np.clip(z_distance / z_half, 0.0, 1.0)
                    
                #slow_factor = 0.5 * (1 - math.cos(math.pi * normalized_dist))
                slow_factor = normalized_dist ** 0.5
                z_speed = z_speed_achieved * slow_factor
                    
                z_speed = max(z_speed, max_speed_z)
                    

            z_speed = float(z_speed)
            
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
                slow_factor_angle = 0.5 * (1 - math.cos(math.pi * abs(normalized_rotation))) * np.sign(normalized_rotation)
                #slow_factor_angle = (abs(normalized_rotation) ** 0.8) * np.sign(normalized_rotation)
                #print("rz_rotation:",rz_rotation)
                if abs(rz_rotation) < 0.05:
                    if smooth_points_rz_stop > 0:
                        smooth_points_rz_stop -= 1
                        rz_speed *= 0.5
                        if abs(rz_speed) < 1e-4:
                            rz_speed = 0.0001 * np.sign(normalized_rotation)
                    else:
                        rz_speed = 0
                else:
                    rz_speed_target = float(max_speed_rz * slow_factor_angle)
                    
                    if smooth_points_rz_start > 0:
                        smooth_points_rz_start -= 1
                        rz_speed = 0.0005 * np.sign(normalized_rotation)
                    else:
                        if abs(rz_speed_target - rz_speed) < ramp_rate_rz:
                            rz_speed = rz_speed_target
                        elif rz_speed_target > rz_speed:
                            rz_speed += ramp_rate_rz
                        else:
                            rz_speed -= ramp_rate_rz

                

                # If within 2mm from target height, stop
                if z_distance <= 0.002:
                    if smooth_points_z_stop > 0:
                        z_speed *= 0.5
                        if z_speed > -1e-4:
                            z_speed = z_last_speed
                            smooth_points_z_stop -= 1
                        else:
                            z_last_speed = z_speed
                    else:
                        velocity_vector[2] = 0
                        URRobot.speedl(velocity_vector, 0.1, 5)
                        if part_number == 0:
                            gripper.open_close(50, 100, 1)
                        elif part_number == 1:
                            gripper.open_close(46, 100, 1)
                        elif part_number == 2:
                            gripper.open_close(55, 100, 1)
                        time.sleep(0.5)
                        return 0
                        
                
                
                rz_speed = float(rz_speed)
                velocity_vector = [x_speed, 0, z_speed, 0, 0, rz_speed]
                print("Vel vector descending: ",velocity_vector, z_distance)
                URRobot.speedl(velocity_vector, 0.2, 0.5)
                


           
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
        URRobot.movel(POSE, 0.1, 0.2, 2)
        
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