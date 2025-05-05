import cv2 as cv
import time
# Open the camera (1 refers to the default camera)
cap = cv.VideoCapture(1)

# Set desired resolution
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Read a frame from the camera
n = 30

toggle = False
while n > 0:
    ret, frame = cap.read()
    cv.imshow('img1',frame)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q') and not toggle:
        toggle = True
        cv.imwrite("calibration_images/captured_image"+ str(n) +".jpg", frame)
        print("Image saved as 'calibration_images/captured_image'"+ str(n) +".jpg")
        n = n - 1
    elif key != ord('q'):

        toggle = False
    if key == 27:
        break
        

# Check if the frame was captured successfully
if not ret:
    print("Error: Could not read frame.")
else:
    # Save the captured image to a file
    cv.imwrite("captured_image1.jpg", frame)
    print("Image saved as 'captured_image.jpg'.")

# Release the camera
cap.release()

# Close any OpenCV windows (if opened)
cv.destroyAllWindows()
