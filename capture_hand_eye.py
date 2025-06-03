import cv2 as cv
import time
import ur
import numpy as np
import re
import socket
from urData import URData

receiver = URData()

for i in range(10):
    print(receiver.get_pose())
    time.sleep(0.1)

# Open the camera (1 refers to the default camera)
cap = cv.VideoCapture(1, cv.CAP_DSHOW)

# Set desired resolution
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Read a frame from the camera
n = 1


# Camera calibration params

with open('calib_param.txt', 'r') as f:
    lines = f.readlines()



cleaned = re.sub(r'[\[\]]', '', lines[1])
cleaned = re.sub(r'\s+', ' ', cleaned)
row1 = np.fromstring(cleaned.strip(), sep=' ')

cleaned = re.sub(r'[\[\]]', '', lines[2])
cleaned = re.sub(r'\s+', ' ', cleaned)
row2 = np.fromstring(cleaned.strip(), sep=' ')

cleaned = re.sub(r'[\[\]]', '', lines[3])
cleaned = re.sub(r'\s+', ' ', cleaned)
row3 = np.fromstring(cleaned.strip(), sep=' ')

camera_matrix = np.array([
    row1,
    row2,
    row3
])

cleaned = re.sub(r'[\[\]]', '', lines[5])
cleaned = re.sub(r'\s+', ' ', cleaned)
distortion_coeffs = np.array([np.fromstring(cleaned.strip(), sep=' ')])

toggle = False


TCP_pose_array = []



while n <= 15:
    ret, frame = cap.read()
    # pose = URRobot.current_Position()
    # print("Pose:", pose)

    new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 1, (width, height))

    # ==================== Undistort the image ====================
    undistorted_img = cv.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)
            
    # ==================== Crop the image to the valid region of interest ====================
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]
    frame = undistorted_img


    cv.imshow('img1',frame)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q') and not toggle:
        toggle = True
        cv.imwrite("calibration_images_hand_eye/captured_image"+ str(n) +".jpg", frame)
        print("Image saved as 'calibration_images_hand_eye/captured_image'"+ str(n) +".jpg")
        
        try:
            pose = receiver.get_pose()
            print("Pose:", pose)
            TCP_pose_array.append(pose)
        except Exception as e:
            print("Error fetching pose:", e)

            
        n = n + 1
    elif key != ord('q'):

        toggle = False
    if key == ord('x'):
        break
        
print(TCP_pose_array)

with open('calib_param_hand_eye.txt', 'w') as file:
    file.write("===============TCP POSE===============\n")
    for i in range(15):
        file.write(str(TCP_pose_array[i]) + '\n')

# Release the camera
cap.release()

# Close any OpenCV windows (if opened)
cv.destroyAllWindows()
