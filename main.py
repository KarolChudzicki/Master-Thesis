import cv2 as cv
import numpy as np
import time
import camera
import robot_control
from gui import Gui
import threading
import tkinter as tk

camera = camera.Camera(1, 1280, 720)

robotControl = robot_control.robotControl(camera_instance=camera)
robotControl.move_home(4)

flags = {'start': True}

def image_show():
    while True:
        coords, area, frame, angle = camera.capture_and_get_coords_center(0)
        #print(coords)
        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
def robot_main_loop():
    # Detecting the part and identifying it
    while flags['start']:
        part_number = None
        while part_number is None and flags['start']:
            part_number, angle = camera.identifyingPart()


        avg_velocity, last_coords, last_coords_time, last_coords_robot = robotControl.rough_estimation(part_number)
        if avg_velocity is not None:
            avg_formatted = f"{avg_velocity:.7f}".replace('.', ',')
            print("Avg velocity: ", avg_formatted)
        
        #avg_velocity = None    
        if avg_velocity is not None:
            
            # Guide the robot to the part
            velocity_vector = robotControl.follow_part(part_number, avg_velocity, last_coords, last_coords_time, last_coords_robot)
            if velocity_vector != None: 
                ret = robotControl.descend_and_grab(velocity_vector, angle, part_number)
            else:
                ret = None
            if ret is not None:
                storage_indicator_array = robotControl.move_part_away(part_number, velocity_vector)
                app.update_indicators(storage_indicator_array)
                    
                
        else:
            print("Invalid part speed")
        
        time.sleep(5)

root = tk.Tk()
app = Gui(root, camera_instance=camera, robot_instance=robotControl, flags = flags)
threading.Thread(target=image_show, daemon=True).start()
threading.Thread(target=robot_main_loop, daemon=True).start()


# Optional: initial indicator state
app.update_indicators([0, 0, 0, 0, 0, 0])

root.mainloop()


