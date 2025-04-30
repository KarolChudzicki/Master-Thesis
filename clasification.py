import cv2 as cv
import time
# Open the camera (0 refers to the default camera)
cap = cv.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()



while True:
    ret, frame = cap.read()
    cv.imshow('img1',frame)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break

