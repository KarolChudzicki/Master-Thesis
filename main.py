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
from gui import Gui
import threading
import tkinter as tk

camera = camera.Camera()
camera.connect(1, 1280, 720)

HOME = [-0.1, 0.766, 0.1, 0, 3.1415, 0]

max_X = 0.1
min_X = -0.2
max_Y = 0.9
min_Y = 0.7
max_Z = 0.2
min_Z = -0.1

#URRobot = ur.URRobot()
gripper = gripper.Gripper()
#robotFollow = robot_follow.robotFollow()

#URRobot.movel(HOME, 0.01, 0.05, 5)


gripper.activate()
gripper.connect()

def image_show():
    while True:
        _, edges, thresh = camera.capture(width=600, part_number=0, show_or_not=False, from_json=True, params=0)
        cv.imshow("Main view", thresh)
        cv.waitKey(1)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    print("1")
    

root = tk.Tk()
app = Gui(root)
threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()

# Optional: initial indicator state
app.update_indicators([0, 1, 0, 1, 1, 0])

root.mainloop()


