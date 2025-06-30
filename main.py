import cv2 as cv
import numpy as np
import serial
import time
import logging
import ur
import camera
import gripper
import conveyor_belt
import robot_control
from gui import Gui
import threading
import tkinter as tk

camera = camera.Camera()
camera.connect(1, 1280, 720)

robotControl = robot_control.robotControl(camera_instance=camera)

robotControl.move_home(4)

def image_show():
    while True:
        coords, area, frame, angle = camera.capture_and_get_coords_center(1)
        #print(coords)
        #print(angle)
        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    # Detecting the part and identifying it
    while True:

        part_number = None
        while part_number is None:
            part_number, angle = camera.identifyingPart()
            
        #print(part_number, angle)

        loopTimeout = time.time()
        while True:
            avg_velocity_x, avg_velocity_x2, _ = robotControl.rough_estimation(part_number)
            last_position_time = time.time()
            if avg_velocity_x is not None:
                break
            elif time.time() - loopTimeout > 10:
                break

        #print(avg_velocity_x, avg_velocity_x2)
        
        
        # Guide the robot to the part
        velocity_vector = robotControl.follow_part(part_number, avg_velocity_x2, last_position_time)
        
        ret = robotControl.descend_and_grab(velocity_vector, angle, part_number)
        if ret is not None:
            storage_indicator_array = robotControl.move_part_away(part_number)
            app.update_indicators(storage_indicator_array)
            
        time.sleep(5)
    

root = tk.Tk()
app = Gui(root, camera_instance=camera)
threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()


# Optional: initial indicator state
app.update_indicators([0, 0, 0, 0, 0, 0])

root.mainloop()


