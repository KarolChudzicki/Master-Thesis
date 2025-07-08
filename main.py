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

camera = camera.Camera(1, 1280, 720)

robotControl = robot_control.robotControl(camera_instance=camera)
robotControl.move_home(4)

start_flag = False

def image_show():
    while True:
        coords, area, frame, angle = camera.capture_and_get_coords_center(0)
        #print(coords)
        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    # Detecting the part and identifying it
    while True:
        print(start_flag)
        if start_flag:
            part_number = None
            while part_number is None:
                part_number, angle = camera.identifyingPart()


            avg_velocity, last_coords, last_coords_time, last_coords_robot = robotControl.rough_estimation(part_number)
            if avg_velocity is not None:
                avg_formatted = f"{avg_velocity:.7f}".replace('.', ',')
                print("Avg velocity: ", avg_formatted)
            
            if avg_velocity is not None:
            
                # Guide the robot to the part
                velocity_vector = robotControl.follow_part(part_number, avg_velocity, last_coords, last_coords_time, last_coords_robot)
                
                ret = robotControl.descend_and_grab(velocity_vector, angle, part_number)
                if ret is not None:
                    storage_indicator_array = robotControl.move_part_away(part_number)
                    app.update_indicators(storage_indicator_array)
                    
                
            else:
                print("Invalid part speed")
        
            time.sleep(3)

root = tk.Tk()
app = Gui(root, camera_instance=camera, robot_instance=robotControl, flag = start_flag)
threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()


# Optional: initial indicator state
app.update_indicators([0, 0, 0, 0, 0, 0])

root.mainloop()


