import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
from urData import URData
import camera
import gripper
import conveyor_belt
import robot_follow
import tkinter as tk
from gui import Gui

root = tk.Tk()
app = Gui(root)
root.mainloop()

HOME = [-0.1, 0.766, 0.1, 0, 3.1415, 0]

max_X = 0.1
min_X = -0.2
max_Y = 0.9
min_Y = 0.7
max_Z = 0.2
min_Z = -0.1

#URRobot = ur.URRobot()
gripper = gripper.Gripper()
robotFollow = robot_follow.robotFollow()

#URRobot.movel(HOME, 0.01, 0.05, 5)


gripper.activate()
gripper.connect()


while True:
    cv.waitKey(1)
    # URRobot.speedl(robotFollow.move_to(0), 0.05, 3)
    # print(robotFollow.move_to(0))


    # key = cv.waitKey(1) & 0xFF
    # if key == ord('c') and not toggle:
    #     gripper.open_close(POSITION_REQUEST=0, SPEED=50, FORCE=1)
    # elif key == ord('o') and not toggle:
    #     gripper.open_close(POSITION_REQUEST=85, SPEED=50, FORCE=1)
    # else:
    #     toggle = False





