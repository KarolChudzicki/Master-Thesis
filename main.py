import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import camera

URRobot = ur.URRobot()

receiver = URData()

# camera = camera.Camera()
# camera.connect(1, 1280, 720)
# camera.initSlider()

while True:
    current_pose = receiver.get_pose()
    print("Pose:", current_pose)

    time.sleep(0.1)





