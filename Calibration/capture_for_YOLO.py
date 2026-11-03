import cv2 as cv
import time
import os
import re
import numpy as np
cap = cv.VideoCapture(1, cv.CAP_DSHOW)

# Set desired resolution
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))



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


# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

folder_path = "training_images"
file_count = len([f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))])
print(file_count)
toggle = False
n = file_count

# width = 600

new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (frame_width, frame_height), 1, (frame_width, frame_height))

while True:
    ret, frame = cap.read()
    
    key = cv.waitKey(1) & 0xFF
    
    if ret:
        
        # ==================== Undistort the image ====================
        undistorted_img = cv.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)
            
        # ==================== Crop the image to the valid region of interest ====================
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]
        frame = undistorted_img

        # CROPPING THE IMAGE
        # cv.rectangle(frame, (0, 0), (width//2, h), (0, 0, 0), -1)
        # cv.rectangle(frame, (w - width//2, 0), (w, h), (0, 0, 0), -1)


        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)

        clahe = cv.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)

        lab_clahe = cv.merge((l_clahe, a, b))
        frame_clahe = cv.cvtColor(lab_clahe, cv.COLOR_LAB2BGR)
    
    
        cv.imshow('Camera view',frame_clahe)
    
    
    
        if key == ord('q') and not toggle:
            toggle = True
            cv.imwrite("training_images/part3/captured_image"+ str(n) +".jpg", frame_clahe)
            print("Image saved as 'training_images/part3/captured_image'"+ str(n) +".jpg")
            n = n + 1
        
        toggle = False
    
    if key == 27:
        break
        


# Release the camera
cap.release()

# Close any OpenCV windows (if opened)
cv.destroyAllWindows()
