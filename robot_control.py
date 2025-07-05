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

gripper = gripper.Gripper(port='COM5')
gripper.activate()



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


        self.max_X = 0.1
        self.min_X = -0.5
        self.max_Y = 0.9
        self.min_Y = 0.7
        self.max_Z = 0.2
        self.min_Z = -0.1
        self.coords_array_x = []
        self.coords_array_y = []
        self.time_stamps = []
        
        self.catched = False
        
        self.indicator_array = [0,0,0,0,0,0]
        

        
    def move_home(self, time_to):
        URRobot.movel(self.HOME1, 0.1, 0.1, time_to)
        
    def move_drop1(self, time_to):
        gripper.open_close(40, 10, 1)
        time.sleep(5)
        URRobot.movel(self.DROP1_ABOVE, 0.1, 0.1, time_to)
        URRobot.movel(self.DROP1, 0.1, 0.1, time_to)
        
    def rough_estimation(self, part_number):
        parts_area = [40000, 27000, 53000]
        incorrect_readings = 0
        max_incorrect_readings = 5
        coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
        
        while incorrect_readings < max_incorrect_readings:
            while area > parts_area[part_number]:
                coords, area, _, _ = self.camera.capture_and_get_coords_center(part_number)
                #print("Coords", coords)
                self.coords_array_x.append(coords[0])
                self.coords_array_y.append(coords[1])
                self.time_stamps.append(time.time())
            incorrect_readings += 1
        
        array_length = len(self.coords_array_x)
        if array_length > 5:
            avg_velocity = (self.coords_array_x[2]-self.coords_array_x[array_length - 2])/(self.time_stamps[2]-self.time_stamps[array_length - 2])
            avg_velocity = round(avg_velocity,7)
            avg_pose_y = np.mean(self.coords_array_y[:(array_length - 1)])
            #print("Test", self.coords_array_y[:(array_length - 1)])
            last_coords = [self.coords_array_x[array_length - 2], float(avg_pose_y)]
            last_coords_time = self.time_stamps[array_length - 2]
            self.coords_array_x = []
            self.coords_array_y = []
            self.time_stamps = []
            
        
            if abs(avg_velocity) > 0.02:
                return avg_velocity, last_coords, last_coords_time
            else:
                print("Average velocity too slow")
                self.coords_array_x = []
                self.coords_array_y = []
                self.time_stamps = []
                return None, None, None
        
        else:
            # Too few readings
            print("Too few readings to calculate average velocity")
            self.coords_array_x = []
            self.coords_array_y = []
            self.time_stamps = []
            return None, None, None


    def is_within_bounds(self, position):
        x, y, z = position
        
        result = [
        1 if x < self.min_X else (-1 if x > self.max_X else 0),
        1 if y < self.min_Y else (-1 if y > self.max_Y else 0),
        1 if z < self.min_Z else (-1 if z > self.max_Z else 0)]

        return np.array(result)


    def follow_part(self, part_number, initial_speed, last_coords, last_coords_time):
        # Max speed x so the robot can catch the part within 2 seconds
        max_speed_x = initial_speed + (last_coords[0] + initial_speed * (time.time() - last_coords_time)) / 2
        
        # Max speed y (1 second to cover the delta y)
        max_speed_y = abs(last_coords[1])
        
        print("Initial speed:", initial_speed, max_speed_x, max_speed_y)
        # Configuration values
        # max_speed_x = initial_speed * 3
        # max_speed_y = abs(initial_speed)
        
        x_speed = initial_speed
        y_speed = 0
        
        x_threshold_distance = abs(initial_speed)
        y_threshold_distance = 0.005 # 5mm
        
        x_speed_goal = False
        y_speed_goal = False
        
        
        # Catch up until the part is visible and within catchup radius and adjust Y
        while True:
            coords, area, frame, angle = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)
            velocity_vector = [max_speed_x, 0, 0, 0, 0, 0]
            URRobot.speedl(velocity_vector, 0.5, 0.5)
            print(coords)
            if coords is not [0,0,0] and area > 20000:
                break
        
        while True:
            pose = URReceiver.get_pose()  # Current robot pose (XYZ + orientation)
            
            coords, area, frame, angle = self.camera.capture_and_get_coords_center(part_number)  # Detected part coords (XYZ)

            #coords[0] -= 0.02 # to account for delays
            coords = np.array(coords)
                
            pose_differenceXY = coords[:2]

            x_distance = pose_differenceXY[0]
            y_distance = pose_differenceXY[1]
            

            print("X distance, y distance", x_distance, y_distance)
            # Update X speed
            if abs(x_distance) > x_threshold_distance:
                x_speed = max_speed_x
                x_speed = float(x_speed)
            else:
                self.decelerate_speed(velocity_vector, abs(x_distance), x_speed, initial_speed, axis=0)
                x_speed = initial_speed
                print("Initial speed achieved")
                x_speed_goal = True
                
            
            
            # Update Y speed with correct direction (signed)
            if not y_speed_goal:
                if abs(y_distance) > y_threshold_distance:
                    y_speed = max_speed_y
                    # Clamp y_speed to [-max_speed_y, max_speed_y]
                    y_speed = -max(-max_speed_y, min(y_speed, max_speed_y))
                    y_speed = float(y_speed)
                else:               
                    y_speed = y_speed * 0.01
                    if abs(y_speed) < 1e-5:
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
            print("Vel vector chasing: ",velocity_vector)
            URRobot.speedl(velocity_vector, 0.2, 0.5)
            
            if x_speed_goal and y_speed_goal:
                break
            
        
        
        return velocity_vector

        
    def descend_and_grab(self, velocity_vector, angle, part_number):
        
        x_speed = velocity_vector[0]
        y_speed = velocity_vector[1]
        max_speed_z = -0.15
        max_speed_rz = 0.5
        descend_height = -0.1  # target height -10cm
        slowdown_radius = 0.1  # 10cm slowdown radius
        slowdown_angle = 0.2
        #decelerate_radius = 0.03 # 3cm 
        
        z_threshold_distance = 0.05
        
        rz_speed = 0.0
        z_speed = 0.0
        
        ramp_rate_z = 0.001
        ramp_rate_rz = 0.01
        
        print("Target angle:", angle)
        
        # To prevent jumps
        if angle > 0:
            angle -= 90
        elif angle < 0:
            angle += 90
        target_angle_rad = math.radians(angle)
        print("Target angle:", target_angle_rad)
        
        last_rotation_time = time.time()
        rz_speed = 0
        rz_pose = 0
        
        
        while True:
            pose = URReceiver.get_pose()  # current z position
            z_pose = pose[2]
        
            # Calculate vertical distance above descend height, but never negative
            z_distance = max(z_pose - descend_height, 0)
            
            
            # Smooth ramp for z_speed
            if abs(max_speed_z - z_speed) < ramp_rate_z:
                z_speed = max_speed_z
            elif max_speed_z > z_speed:
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
                    rz_speed = rz_speed * 0.01
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
                print("Vel vector descending: ",velocity_vector, z_distance)
                URRobot.speedl(velocity_vector, 0.2, 0.5)

                if z_distance < z_threshold_distance:
                    self.decelerate_speed(velocity_vector, z_distance, z_speed, 0, 2)
                    if part_number == 0:
                        gripper.open_close(50, 100, 1)
                    elif part_number == 1:
                        gripper.open_close(46, 100, 1)
                    elif part_number == 2:
                        gripper.open_close(55, 100, 1)
                    time.sleep(0.5)
                    return 0
                
    def decelerate_speed(self, velocity_vector, total_distance, initial_speed, target_speed, axis):
        """
        Smoothly decelerates from initial_speed to target_speed over total_distance.

        Parameters:
            total_distance (float): The full distance to reduce speed.
            initial_speed (float): Starting speed (positive or negative).
            target_speed (float): Desired speed at the end of deceleration.
            axis (int): Axis index (0 - x, ..., 5 - rz).
        """
        
        if target_speed == initial_speed:
            logging.warning("Initial and target speeds are the same. No deceleration needed.")
            return
        else:
        
            direction = 1 if initial_speed > 0 else -1
            start_pos = URReceiver.get_pose()[axis]
            
            # Estimate average speed for total time
            avg_speed = (abs(initial_speed) + abs(target_speed)) / 2
            
            estimated_time = total_distance / avg_speed
            ideal_step_duration = 0.03  # You can tweak this
            #Minimum 10 steps
            steps = max(10, int(estimated_time / ideal_step_duration))
            print("Deceleration stuff: ", estimated_time, steps, total_distance, target_speed)

            while True:
                current_pos = URReceiver.get_pose()[axis]
                distance_covered = abs(current_pos - start_pos)
                print("Distance covered: ", distance_covered)
                t = min(distance_covered / max(total_distance, 1e-6), 1.0)
                # EXPONENTIAL
                #speed = initial_speed + (target_speed - initial_speed) * (1 - math.exp(-5 * (i / (steps - 1))))
                #
                #speed = initial_speed + (target_speed - initial_speed) * ((i / (steps - 1)) ** 0.5)
                # SINE EASE OUT
                # t = (i + 1) / steps
                # speed = initial_speed + (target_speed - initial_speed) * math.sin(t * math.pi / 2)
                # SIGMOID
                k = 5  # steepness factor
                speed = target_speed + (initial_speed - target_speed) / (1 + math.exp(k * (t - 0.5)))
                                
                
                print("speed for decelerating: ",speed)
                
                velocity_vector[axis] = speed

                URRobot.speedl(velocity_vector, 0.5, 0.2)
                
                if distance_covered >= abs(total_distance):
                    logging.info("Target distance reached, stopping deceleration.")
                    break
            
            # Ensure final target speed is reached
            velocity_vector[axis] = target_speed
            URRobot.speedl(velocity_vector, 0.5, 0.2)

           
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