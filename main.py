import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import camera

HOME = [-0.1, 0.766, 0, 0, 3.1415, 0]

URRobot = ur.URRobot()

URRobot.movel(HOME, 0.01, 0.05, 5)

#receiver = URData()

camera = camera.Camera()
camera.connect(1, 1280, 720)
camera.initSlider()

while True:
    #current_pose = receiver.get_pose()
    #print("Pose:", current_pose)
    
    camera.capture(600)






