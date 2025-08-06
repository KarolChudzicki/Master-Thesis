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


        # ========================== ROBOT POSITIONS ==========================
        # Tray positions
        pose_above = 0.2
        shift_for_position2 = 0.0705
        shift_for_position3 = 0.072
        
        self.DROP1 = [-0.625, 0.0275, 0.067, 3.0383, -0.0147, -0.7923]
        self.DROP1_ABOVE = self.DROP1.copy()
        self.DROP1_ABOVE[2] = pose_above

        
        self.DROP2 = self.DROP1.copy()
        self.DROP2[1] += shift_for_position2
        self.DROP2_ABOVE = self.DROP2.copy()
        self.DROP2_ABOVE[2] = pose_above

        
        self.DROP3 = self.DROP2.copy()
        self.DROP3[1] += shift_for_position3
        self.DROP3_ABOVE = self.DROP3.copy()
        self.DROP3_ABOVE[2] = pose_above
        
        # Pick and assembly
        tilted = 30 #Tray for pickup is tilted 25 degrees
        self.PICK1 = [-0.5414, 0.026, 0.025, 3.05, 0, -0.714]
        x1_new = (pose_above - self.PICK1[2]) * math.tan(math.radians(tilted))
        self.PICK1_ABOVE = self.PICK1.copy()
        self.PICK1_ABOVE[0] += x1_new
        self.PICK1_ABOVE[2] = pose_above
        
        self.PICK2 = [-0.537, 0.098, 0.0234, 3.05, 0, -0.714]
        x2_new = (pose_above - self.PICK2[2]) * math.tan(math.radians(tilted))
        self.PICK2_ABOVE = self.PICK1.copy()
        self.PICK2_ABOVE[0] += x2_new
        self.PICK2_ABOVE[1] += shift_for_position2
        self.PICK2_ABOVE[2] = pose_above
        
        self.PICK3 = self.PICK1.copy()
        self.PICK3[1] += shift_for_position3 + shift_for_position2
        self.PICK3_ABOVE = self.PICK1_ABOVE.copy()
        self.PICK3_ABOVE[1] += shift_for_position3 + shift_for_position2
        
        
        self.ASSEMBLY1_1 = [0.0993, -0.747, 0.003, 2.22, 2.22, 0]
        self.ASSEMBLY_ABOVE = self.ASSEMBLY1_1.copy()
        self.ASSEMBLY_ABOVE[2] = pose_above
        
        self.ASSEMBLY1_2 = self.ASSEMBLY1_1.copy()
        self.ASSEMBLY1_2[2] += 0.015
        
        self.ASSEMBLY1_3 = self.ASSEMBLY1_1.copy()
        self.ASSEMBLY1_3[2] += 0.003
        self.ASSEMBLY1_3[1] += 0.002 # add 2 mm because of some alligment issues
        
        
        self.INBETWEEN = self.PICK1_ABOVE.copy()
        self.INBETWEEN[1] = self.ASSEMBLY1_1[1]
    
        # =====================================================================

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
        self.Kp = 1.7
        self.Ki = 0.25
        self.integral_min = -0.4
        self.Kd = 0.0
        self.Kff = 0.0
        self.stablization_points = 0
        self.start_time_log = 0
        self.start_time = 0
        
        self.data_log = []
        
        self.follow_part_log = []
        self.follow_part_start_time_log = 0
        self.descend_log = []
        self.descend_start_time_log = 0
        self.xy_log = []
        self.xy_start_time_log = 0
        self.pred_vs_cam_log = []
        self.pred_vs_cam_time_log = 0
        
        self.time_log = []
        
        self.estimation_time_start = 0
        self.estimation_time_stop = 0
        self.catch_time_start = 0
        self.catch_time_stop = 0
        self.start_time_system = 0
        self.start_pos_x = 0
        self.stop_pos_x = 0

    def save_log_csv(self, log_data, folder, name):
        # Create the folder if it doesn't exist
        os.makedirs(folder, exist_ok=True)

        # Create a unique filename with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = name + f"_{timestamp}.csv"
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
        
    def rough_estimation(self, part_number, start_time_system):
        self.start_time_system = start_time_system
        self.part_identification_time_stop = time.time()
        self.estimation_time_start = time.time()
        parts_area = [34000, 22000, 48000]
        incorrect_readings = 0
        max_incorrect_readings = 5
        coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
        
        min_readings = 7
        timeout = time.time()
        while incorrect_readings < max_incorrect_readings:
            coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
            if area > parts_area[part_number]:
                self.coords_array_x.append(coords[0])
                self.coords_array_y.append(coords[1])
                self.time_stamps.append(time.time())
            else:
                # Check if there even was a valid reading
                if len(self.time_stamps) > 0:
                    incorrect_readings += 1
            
            if time.time() - timeout > 10:
                print("Reading loop timeout")
                break
        
        array_length = len(self.coords_array_x)
        if array_length >= min_readings:
            delta_x = self.coords_array_x[2]-self.coords_array_x[array_length - 3]
            delta_t = self.time_stamps[2]-self.time_stamps[array_length - 3]
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
                self.estimation_time_stop = time.time()
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
        self.catch_time_start = time.time()
        self.start_pos_x = URReceiver.get_pose()[0]
        # Max speed x so the robot can catch the part within 1 second
        #max_speed_x = initial_speed * 3
        max_speed_x = initial_speed + (last_coords[0] + initial_speed * (time.time() - last_coords_time)) /1.5
        #print("Max speed: ", max_speed_x, last_coords[0] + initial_speed * (time.time() - last_coords_time))
        
        # Max speed y (1 second to cover the delta y) but not less than 15mm/s
        max_speed_y = min(abs(last_coords[1]), 0.015)
        direction_y = np.sign(last_coords[1])
        
        #print("Initial speed:", initial_speed, max_speed_x, max_speed_y)
        
        x_speed = initial_speed
        y_speed = 0
        
        y_slowdown_radius = 0.01 # 10mm
        y_threshold_distance = 0.002 # 2mm
        
        x_speed_goal = False
        y_speed_goal = False
        
        
        alpha = 1
        delta_alpha = 0.1
        alpha_progress = 1
        
        self.xy_start_time_log = time.time()

        while True:
            pose = URReceiver.get_pose()  # Current robot pose (XYZ + orientation)
            if pose == [None] * 6:
                logging.error("Lost robot pose")
                self.move_home(4)
                return None
            coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)
            coords = np.array(coords)
                
            # Predicted pose when the part is not visible based on inital speed
            x_distance_predicted = (last_coords[0] + initial_speed*(time.time() - last_coords_time)) - (pose[0] - last_coords_robot[0])
            y_distance_predicted = last_coords[1] - (last_coords_robot[1] - pose[1])
            
            y_distance = coords[1]
            
            if not np.array_equal(coords, [0, 0, 0]) and area > 20000:
                alpha_progress -= delta_alpha
                if alpha_progress <= 0:
                    alpha_progress = 0
                last_coords[0] = coords[0]
                last_coords[1] = coords[1]
                last_coords_time = time.time()
                last_coords_robot[0] = pose[0]
                last_coords_robot[1] = pose[1]
            else:
                alpha_progress += delta_alpha
                if alpha_progress >= 1:
                    alpha_progress = 1
                    
            alpha = round(alpha_progress,10)
            print("X:",coords[0], x_distance_predicted)
            print("Y:",coords[1], y_distance_predicted)

            x_distance = alpha * x_distance_predicted + (1 - alpha) * coords[0]
            y_distance = alpha * y_distance_predicted + (1 - alpha) * coords[1]    
            
            print("Final xy:",x_distance, y_distance)
            
            
            
            # Update X speed using a controler
            if x_speed_goal is False:
                x_speed, x_speed_goal = self.decelerate_to_target(alpha, x_distance, x_speed, max_speed = max_speed_x, min_speed=initial_speed/2)
                if x_speed_goal is True:
                    # Controller stopped, save data to a log file
                    self.save_log_csv(log_data=self.data_log, folder = "Plots/pid", name = "pid")
                    self.data_log = []
                    self.camera.coords_history = []
                    
            
            if abs(y_distance) > y_threshold_distance and y_speed_goal is False:
                norm_y = np.clip(abs(y_distance)/y_slowdown_radius, 0, 1)
                y_speed_factor = norm_y ** 4
                y_speed = -max_speed_y * direction_y
            else:
                y_speed_goal = True
                y_speed = y_speed * 0.5
                if abs(y_speed) < 1e-4:
                    y_speed = 0 
                        
                        
            y_speed = float(y_speed)
                
            
            escape_vector = self.is_within_bounds(pose[:3])
            if not np.all(escape_vector == 0):
                adjusted_escape = (escape_vector.T * np.array([0.1, 0.1, 0.1]))
                adjusted_escape = adjusted_escape.astype(float).tolist()
                x_speed = adjusted_escape[0]
                y_speed = adjusted_escape[1]

            velocity_vector = [x_speed, y_speed, 0, 0, 0, 0]
            #print("Vel vector chasing: ",velocity_vector, y_distance)
            URRobot.speedl(velocity_vector, 1, 0.5)
            
            self.xy_log.append({
                "time": time.time() - self.xy_start_time_log,
                "x_speed": x_speed,
                "y_speed": y_speed,
                "x_camera":coords[0],
                "x_pred":x_distance_predicted,
                "y_camera":coords[1],
                "y_pred":y_distance_predicted,
                "x_final":x_distance,
                "y_final":y_distance,
                "alpha":alpha
            })
            
            if x_speed_goal and y_speed_goal:
                self.save_log_csv(log_data=self.xy_log, folder = "Plots/follow", name = "xy")
                self.xy_log = []
                break
        
        return velocity_vector
    
    
    def decelerate_to_target(self, alpha, distance_to_target, initial_speed, max_speed, min_speed):

        if self.start_time == 0:
            self.start_time = time.time()
        
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
        
        # Anti windup
        self.integral = max(self.integral, self.integral_min)
        
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
            "time": current_time - self.start_time,
            "error": error,
            "x_speed": speed,
            "p_term": self.Kp * error,
            "i_term": self.Ki * self.integral,
            "d_term": self.Kd * self.derivative,
            "alpha": alpha
        })
        
        
        self.stablization_points += (abs(error) < 0.03) # self.Kd * self.derivative < 0.005
        #print("Error", error)
        
        if self.stablization_points > 10:
            self.integral = 0
            self.derivative = 0
            self.last_time = None
            self.previous_error = None
            self.stablization_points = 0
            at_target = True
            self.start_time = 0
        else:
            at_target = False
        
        
        return speed, at_target
        
    def descend_and_grab(self, velocity_vector, angle, part_number):
        x_speed = velocity_vector[0]
        max_speed_z = -0.2
        descend_height = -0.105
        
        rz_speed = 0.001
        z_speed = 0.0
        
        ramp_rate_z = 0.003
        
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
        
        min_speed_rz = 0.1
        max_speed_rz = 0.8
        min_angle = 5    
        max_angle = 90
        angle_clamped = np.clip(abs(angle), min_angle, max_angle)
        
        top_speed = min_speed_rz + (angle_clamped - min_angle) / (max_angle - min_angle) * (max_speed_rz - min_speed_rz)
        print("Top speed", top_speed)
        
        target_angle_rad = math.radians(angle)
        
        last_rotation_time = time.time()
        rz_speed = 0
        rz_pose = 0
        
        rz_acceleration_threshold = 0.1
        rz_deceleration_threshold = 0.9
        acceleration_angle = rz_acceleration_threshold * target_angle_rad
        deceleration_angle = rz_deceleration_threshold * target_angle_rad
        min_speed_rz = 0.001
        
        smooth_points_z_start = 5
        
        
        self.descend_start_time_log = time.time()
        
        rz_speed_prev = 0
        
        while True:
            pose = URReceiver.get_pose()  # current z position
            
            if pose == [None] * 6:
                logging.error("Lost robot pose")
                self.move_home(4)
                return None
            
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
            elif z_distance > 0.001:
                # Normalize distance in slowdown zone [0..1]
                normalized_dist = np.clip(z_distance / z_half, 0.0, 1.0)
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
                # Rz speed control if the angle is bigger than 5 deg
                if abs(angle) > min_angle:
                    # Rotation to target_angle
                    progress_acceleration = np.clip(abs(rz_pose/acceleration_angle), 0.0, 1.0)
                    progress_deceleration = np.clip(abs((rz_pose-deceleration_angle)/(target_angle_rad-deceleration_angle)), 0.0, 1.0)
                    sign_rz = -np.sign(target_angle_rad)
                    
                    
                    if abs(rz_pose) < abs(acceleration_angle):
                        p = progress_acceleration
                        
                        #accel = p**2
                        # Ease-Out Cubic
                        #accel = 1 - (1-p)**3
                        # Exponential Ramp-Up
                        #print("P:", p)
                        accel = 1 - np.exp(-4 * p)
                        # Smoothstep
                        #accel = 3 * p**2 - 2 * p**3
                        
                        rz_speed = top_speed * accel
                        rz_speed = max(rz_speed, min_speed_rz)
                        rz_speed *= sign_rz
                        top_achieved_rz_speed = rz_speed
                    elif abs(rz_pose) >= abs(deceleration_angle):
                        p = progress_deceleration
                        decel = 1 - (1 - p)**2
                        rz_speed = top_achieved_rz_speed * (1 - decel)
                        if abs(rz_speed) <= 1e-4:
                            rz_speed = 0 
                    else:
                        rz_speed = rz_speed
                else:
                    rz_speed = 0
                
                rz_speed = float(rz_speed)
                    
                        

                

                # If within 3mm from target height, stop
                if z_distance <= 0:
                    velocity_vector[2] = 0
                    URRobot.speedl(velocity_vector, 3, 5)
                    if part_number == 0:
                        gripper.open_close(50, 100, 1)
                    elif part_number == 1:
                        gripper.open_close(46, 100, 1)
                    elif part_number == 2:
                        gripper.open_close(55, 100, 1)
                    time.sleep(0.5)
                    self.catch_time_stop = time.time()
                    self.stop_pos_x = URReceiver.get_pose()[0]
                    self.time_log.append({
                        "part_number": part_number,
                        "part_rotation": math.degrees(target_angle_rad),
                        "distance_traveled": self.start_pos_x - self.stop_pos_x,
                        "time_ident_stop": round(self.part_identification_time_stop - self.start_time_system,5),
                        "time_speed_est_start": round(self.estimation_time_start - self.start_time_system,5),
                        "time_speed_est_stop": round(self.estimation_time_stop - self.start_time_system,5),
                        "time_catch_start": round(self.catch_time_start - self.start_time_system,5),
                        "time_catch_stop": round(self.catch_time_stop - self.start_time_system,5)
                    })
                    
                    
                    
                    self.save_log_csv(log_data=self.time_log, folder="Plots/time", name="time")
                    self.save_log_csv(log_data=self.descend_log, folder="Plots/descend", name="positions")
                    self.descend_log = []
                    return 0
                        
                        
                
                
                rz_speed = float(rz_speed)
                velocity_vector = [x_speed, 0, z_speed, 0, 0, rz_speed]
                current_time = time.time()
                
                self.descend_log.append({
                    "time": current_time - self.descend_start_time_log,
                    "z_pos": z_pose,
                    "rotation": rz_pose,
                    "z_speed": z_speed,
                    "rz_speed": rz_speed,
                    "z_target": descend_height,
                    "rz_target": target_angle_rad,
                    "delta_rz_speed": rz_speed - rz_speed_prev
                })
                
                rz_speed_prev = rz_speed
                
                print("Vel vector descending: ",velocity_vector)
                URRobot.speedl(velocity_vector, 3, 0.5)
        


           
    def move_part_away(self, part_number, velocity_vector):
        
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
        # Slow the robot first
        x_speed_start = velocity_vector[0]
        x_speed = x_speed_start
        z_speed = 0
        z_speed_ramp = 0.005
        time_start = time.time()
        ascend_duration = 2
        z_max_speed_ascend = 0.25
        while time.time() - time_start <= ascend_duration:
            x_speed *= 0.98
            if x_speed > -1e-4:
                x_speed = 0
            if (time.time() - time_start) < ascend_duration/2:
                z_speed += z_speed_ramp
                z_speed = min(z_speed, z_max_speed_ascend)
            else:
                z_speed -= z_speed_ramp
                if z_speed <= 1e-4:
                    z_speed = 0
            velocity_vector[0] = x_speed
            velocity_vector[2] = z_speed
            URRobot.speedl(velocity_vector, 2, 0.5)
        
        self.move_and_drop(above, target)
        
        # if self.indicator_array[idx1] == 0:
        #     self.indicator_array[idx1] = 1
        #     self.move_and_drop(above, target)
        # elif self.indicator_array[idx2] == 0:
        #     self.indicator_array[idx2] = 1
        #     self.move_and_drop(above, target)
        # else:
        #     logging.info(f"Storage for part {part_number + 1} full !!!")
            
            
        required_parts = [self.indicator_array[3], self.indicator_array[4], self.indicator_array[5]]       
        
        if all(required_parts):
            print("We have all parts")
            #self.assemble_product()
            self.indicator_array = [0,0,0,0,0,0]
        
        self.move_home(2)
        gripper.open_close(85, 100, 1)
        
        
        
        return self.indicator_array

    def move_and_drop(self, above, target):
        URRobot.movel(above, 0.5, 0.3, 2)
        URRobot.movel(target, 0.2, 0.2, 2)
        gripper.open_close(65, 1, 1)
        time.sleep(0.5)
        URRobot.movel(above, 0.2, 0.2, 2)
        gripper.open_close(85, 100, 1)


    def assemble_product(self):
        t_l = 4
        t_s = 2
        # Part 1
        URRobot.movel(self.PICK1_ABOVE, 0.5, 0.2, t_s)
        gripper.open_close(60, 100, 1)
        URRobot.movel(self.PICK1, 0.5, 0.2, t_s)
        gripper.open_close(50, 100, 1)
        time.sleep(1)
        URRobot.movel(self.PICK1_ABOVE, 0.5, 0.2, t_s)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY1_1, 0.5, 0.2, t_s)
        gripper.open_close(85, 100, 1)
        time.sleep(0.5)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_l)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        
        # Part 2
        URRobot.movel(self.PICK2_ABOVE, 0.5, 0.2, t_l)
        gripper.open_close(50, 100, 1)
        URRobot.movel(self.PICK2, 0.5, 0.2, t_s)
        gripper.open_close(45, 100, 1)
        time.sleep(1)
        URRobot.movel(self.PICK2_ABOVE, 0.5, 0.2, t_s)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY1_2, 0.5, 0.2, t_s)
        gripper.open_close(85, 100, 1)
        time.sleep(0.5)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_l)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        
        
        # Part 3
        URRobot.movel(self.PICK3_ABOVE, 0.5, 0.2, t_l)
        gripper.open_close(65, 100, 1)
        URRobot.movel(self.PICK3, 0.5, 0.2, t_s)
        gripper.open_close(55, 100, 1)
        time.sleep(1)
        URRobot.movel(self.PICK3_ABOVE, 0.5, 0.2, t_s)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_l)
        URRobot.movel(self.ASSEMBLY1_3, 0.5, 0.2, t_s)
        gripper.open_close(85, 100, 1)
        time.sleep(0.5)
        URRobot.movel(self.ASSEMBLY_ABOVE, 0.5, 0.2, t_s)
        URRobot.movel(self.INBETWEEN, 0.5, 0.2, t_l)
        URRobot.movel(self.PICK3_ABOVE, 0.5, 0.2, t_l)