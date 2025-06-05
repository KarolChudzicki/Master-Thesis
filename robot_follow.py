import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import camera
import gripper


class robotFollow:
    def __init__(self):
        self.camera = camera.Camera()
        self.camera.connect(1, 1280, 720)
        self.receiver = URData()
        self.camera.initSlider()
        self.HOME = [-0.1, 0.766, 0.1, 0, 3.1415, 0]

        self.max_X = 0.1
        self.min_X = -0.2
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

    def move_to(self):
        pose_from = self.receiver.get_pose()
        camera_pose, area = self.camera.capture(600)

        pose_to = pose_from[:3] - camera_pose[:3]
        pose_to[2] = self.min_Z

        
        distance = pose_to - pose_from[:3]
        distance = np.linalg.norm(distance)
        max_speed = 1
        min_speed = 0.01
        k = 25
        # speed = min(max_speed, max(min_speed, k * distance))
        # if speed < 0.01:
        #     speed = 0
        print(distance)
        if distance < 0.002:
            # Smooth deceleration near target
            scale = distance * 1  # 1 → 0
            
            speed = min(max_speed, max(min_speed, k * distance))
            #speed = max(min_speed, max_speed * (scale ** 2))  # quadratic decay
            #print("Scale - ", scale, "Speed - ", speed)
            if distance < 0.001:
                speed = 0
                #print("Speed")
        else:
            speed = max_speed

        #Calculate direction and velocity
        direction = pose_to - pose_from[:3]
        #direction[:2] /= np.linalg.norm(direction[:2])  # normalize position part

        velocity = np.round(direction * speed,5)
        velocity = velocity.astype(float).tolist()
        # print("Vel: ",velocity)
        # print("Pose from", pose_from[2])
        if pose_from[2] <= -0.095: velocity[2] = 0
        velocity_vector = [velocity[0], -velocity[1], velocity[2], 0, 0, 0]
        #print(velocity_vector)
        
        if camera_pose.all() != 0:
            #print("Vel vector: ",velocity_vector)
            escape_vector = self.is_within_bounds(pose_from[:3])
            #print("Esc vector: ",escape_vector)
            if not escape_vector.any() == 0:
                nvv = escape_vector.T * np.array([0.1,0.1,0.1])
                nvv = nvv.astype(float).tolist()
                new_velocity_vector = [nvv[0], nvv[1], nvv[2], 0, 0, 0]
                #print("NVV: ",new_velocity_vector)
                return new_velocity_vector
            else:
                return velocity_vector
        else:
            return [0,0,0,0,0,0]





