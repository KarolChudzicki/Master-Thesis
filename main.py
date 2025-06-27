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

HOME1 = [-0.4, 0.766, 0.1, 3.1415, 0, 0]
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

URRobot.movel(HOME2, 0.01, 0.05, 5)


gripper.activate()
gripper.connect()

def image_show():
    while True:
        coords, area, frame, angle = camera.capture_and_get_coords_edge(0)
        cv.imshow("Frame", frame)
        #cv.waitKey(1)
        #print(area, angle)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    # Detecting the part and identifying it
    while True:
        coords1, area, frame, angle = camera.capture_and_get_coords_center(0)
        coords2, area, frame, angle = camera.capture_and_get_coords_edge(0)
        print("Coords center = ",coords1)
        print("Coords edge = ",coords2)
        # part_number = None
        # while part_number is None:
        #     part_number = camera.identifyingPart()
            
        # print(part_number)

        # while True:
        #     avg_velocity_x, last_coords = robotFollow.rough_estimation(part_number)
        #     if avg_velocity_x is not None:
        #         break

        # print(avg_velocity_x, last_coords)
        # time.sleep(5)
    

root = tk.Tk()
app = Gui(root, camera_instance=camera)
threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()

# Optional: initial indicator state
app.update_indicators([0, 1, 0, 1, 1, 0])

root.mainloop()


