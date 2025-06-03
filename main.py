import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import camera
import gripper

HOME = [-0.1, 0.766, 0.1, 0, 3.1415, 0]

max_X = 0.1
min_X = -0.5
max_Y = 0.9
min_Y = 0.7
max_Z = 0.2
min_Z = -0.105

URRobot = ur.URRobot()
gripper = gripper.Gripper()

URRobot.movel(HOME, 0.01, 0.05, 5)

receiver = URData()

camera = camera.Camera()
camera.connect(1, 1280, 720)
camera.initSlider()
gripper.activate()
gripper.connect()

def is_within_bounds(position):
    x, y, z = position
    
    if 

    return (min_X <= x, x <= max_X, min_Y <= y, y <= max_Y, min_Z <= z, z <= max_Z)

while True:
    pose_from = receiver.get_pose()
    camera_pose, area = camera.capture(600)
    #print(camera_pose)
    # Get X and Y coords
    pose_to = pose_from[:2] - camera_pose[:2]

    #print(pose_to, pose_from[:2])
    
    distance = pose_to - pose_from[:2]
    distance = np.linalg.norm(distance)
    #print("Distance: ", distance)
    dt = 0.1
    max_speed = 1
    min_speed = 0
    k = 50
    speed = min(max_speed, max(min_speed, k * distance))
    if speed < 0.01:
        speed = 0

    print("Speed: ",speed)

    #Calculate direction and velocity
    direction = pose_to - pose_from[:2]
    #direction[:2] /= np.linalg.norm(direction[:2])  # normalize position part

    velocity = np.round(direction * speed,5)
    velocity = velocity.astype(float).tolist()
    #print(velocity)
    velocity_vector = [velocity[0], -velocity[1], 0, 0, 0, 0]
    
    if area > 10000 and camera_pose.all() != 0:
        print(velocity_vector)
        escape_vector = is_within_bounds(pose_from[:3])
        if not escape_vector.any:
            new_velocity_vector = escape_vector * [0,0,]
            URRobot.speedl([0,0,0,0,0,0], 0.05, 5)
        else:
            URRobot.speedl(velocity_vector, 1, 5)
    else:
        URRobot.speedl([0,0,0,0,0,0], 0.05, 5)


    

    key = cv.waitKey(1) & 0xFF
    if key == ord('o') and not toggle:
        gripper.open_close(POSITION_REQUEST=0, SPEED=50, FORCE=1)
    elif key == ord('c') and not toggle:
        gripper.open_close(POSITION_REQUEST=85, SPEED=50, FORCE=1)
    else:
        toggle = False





