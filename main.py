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

HOME1 = [-0.1, 0.766, 0.1, 0, 3.1415, 0]
inBetweenHOME1HOME2 = [-0.1, 0.766, 0.1, 1.3, -2.8, 0]
HOME2 = [-0.1, 0.766, 0.1, 3.1415, 0, 0]

max_X = 0.1
min_X = -0.2
max_Y = 0.9
min_Y = 0.7
max_Z = 0.2
min_Z = -0.1

URRobot = ur.URRobot()
gripper = gripper.Gripper()
robotFollow = robot_follow.robotFollow(camera_instance=camera)

URRobot.movel(inBetweenHOME1HOME2, 0.01, 0.05, 5)
URRobot.movel(HOME2, 0.01, 0.05, 5)


gripper.activate()
gripper.connect()

def image_show():
    while True:
        _, edges, thresh, _ = camera.capture(width=600, part_number=0, show_or_not=False, from_json=True, params=0)
        cv.imshow("Main view", thresh)
        cv.waitKey(1)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    while True:
        contours, edges, thresh, frame, gray_gray = camera.capture(width=600, part_number=0, show_or_not=False, from_json=True, params=0)
        points, area, frame = camera.calculate_coords(contours, frame)
        print(points, area)
        cv.imshow("Frame", frame)
        cv.waitKey(1)
        velocity_vector = robotFollow.move_to(part_number=0)
        print(velocity_vector)
        URRobot.speedl(velocity_vector, 0.5, 2)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    

root = tk.Tk()
app = Gui(root, camera_instance=camera)
#threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()

# Optional: initial indicator state
app.update_indicators([0, 1, 0, 1, 1, 0])

root.mainloop()


