import cv2 as cv
import time
import numpy as np
import math
import re

import serial
import time
import logging

logging.basicConfig(level=logging.INFO)








class Camera:
    def __init__(self):

                # Read file
        with open('calib_param.txt', 'r') as f:
            self.lines = f.readlines()



        self.cleaned = re.sub(r'[\[\]]', '', self.lines[1])
        self.cleaned = re.sub(r'\s+', ' ', self.cleaned)
        self.row1 = np.fromstring(self.cleaned.strip(), sep=' ')

        self.cleaned = re.sub(r'[\[\]]', '', self.lines[2])
        self.cleaned = re.sub(r'\s+', ' ', self.cleaned)
        self.row2 = np.fromstring(self.cleaned.strip(), sep=' ')

        self.cleaned = re.sub(r'[\[\]]', '', self.lines[3])
        self.cleaned = re.sub(r'\s+', ' ', self.cleaned)
        self.row3 = np.fromstring(self.cleaned.strip(), sep=' ')

        self.camera_matrix = np.array([
            self.row1,
            self.row2,
            self.row3
        ])

        self.cleaned = re.sub(r'[\[\]]', '', self.lines[5])
        self.cleaned = re.sub(r'\s+', ' ', self.cleaned)
        self.distortion_coeffs = np.array([np.fromstring(self.cleaned.strip(), sep=' ')])

        IC_width = 60
        IC_length = 80
        self.IC_points = np.array([
            [-IC_length/2, -IC_width/2, 0],  # Corner 1
            [-IC_length/2, IC_width/2, 0],  # Corner 2
            [IC_length/2, IC_width/2, 0],  # Corner 3
            [IC_length/2, -IC_width/2, 0]   # Corner 4
        ], dtype=np.float32)

    def connect(self, camera_id, width, height) -> None:
        self.cap = cv.VideoCapture(camera_id) 
        # Set desired resolution
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.width = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    def capture(self, width) -> None:

        (ret, frame) = self.cap.read()
        self.fps = self.cap.get(cv.CAP_PROP_FPS)
        if ret:
        
            new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coeffs, (self.width, self.height), 1, (self.width, self.height))

            # ==================== Undistort the image ====================
            undistorted_img = cv.undistort(frame, self.camera_matrix, self.distortion_coeffs, None, new_camera_matrix)
            
            # ==================== Crop the image to the valid region of interest ====================
            x, y, w, h = roi
            undistorted_img = undistorted_img[y:y+h, ((x+w)//2-width//2):((x+w)//2+width//2)]
            frame = undistorted_img

            # ==================== Get parameters from sliders ====================

            h_low = cv.getTrackbarPos("H low", "Camera params")
            s_low = cv.getTrackbarPos("S low", "Camera params") 
            v_low = cv.getTrackbarPos("V low", "Camera params") 
            h_up = cv.getTrackbarPos("H upper", "Camera params") 
            s_up = cv.getTrackbarPos("S upper", "Camera params") 
            v_up = cv.getTrackbarPos("V upper", "Camera params") 
            
            dil = cv.getTrackbarPos("Dilation", "Camera params") 
            ero = cv.getTrackbarPos("Erosion", "Camera params") 

            canny_low = cv.getTrackbarPos("Canny", "Camera params") 
            thresh_low = cv.getTrackbarPos("Thresh", "Camera params")

            lower_bound = (h_low, s_low, v_low)
            upper_bound = (h_up, s_up, v_up)

            # ==================== Applying mask ====================
            lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
            l, a, b = cv.split(lab)

            clahe = cv.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            l_clahe = clahe.apply(l)

            lab_clahe = cv.merge((l_clahe, a, b))
            frame_clahe = cv.cvtColor(lab_clahe, cv.COLOR_LAB2BGR)
            hsv = cv.cvtColor(frame_clahe, cv.COLOR_BGR2HSV)
   
            mask_gray = cv.inRange(hsv, lower_bound, upper_bound)  

            mask_gray = cv.morphologyEx(mask_gray, cv.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask_gray = cv.morphologyEx(mask_gray, cv.MORPH_CLOSE, np.ones((5,5), np.uint8))

            gray_region = cv.bitwise_and(frame, frame, mask=mask_gray)
            gray_gray = cv.cvtColor(gray_region, cv.COLOR_BGR2GRAY)


            kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

            # Apply Erosion
            eroded = cv.erode(gray_gray, kernel, iterations=ero)
            
            # Apply Dilation
            dilated = cv.dilate(eroded, kernel, iterations=dil)

            _, thresh = cv.threshold(dilated, 1, 255, cv.THRESH_BINARY)

            edges = cv.Canny(thresh, 150, 200)

            
            # result = cv.bitwise_and(hsv, frame, mask=mask)

            # frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # frame_blurred = cv.GaussianBlur(frame_gray, (9,9), 0)

            #_, frame_thresholded = cv.threshold(frame_blurred, thresh_low, 255, cv.THRESH_BINARY_INV)

            # edges = cv.Canny(frame_blurred, canny_low, 200)

            contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        
        
    
            if contours:
                contour = max(contours, key=cv.contourArea)
                
                # # Get bounding rectangle or mask
                # x, y, w, h = cv.boundingRect(contour)

                # # Crop the original image to the bounding box of the biggest contour
                # cropped = frame[y:y+h, x:x+w]

                # # Show result
                # cv.imshow('Biggest Contour Region', cropped)



                # Calculating contour center x coordinate
                M = cv.moments(contour)

                rect = cv.minAreaRect(contour)  # Get the bounding rectangle
                center_rect, size_rect, angle_rect = rect
                print(center_rect, angle_rect)
                
                center_rect = [int(center_rect[0]), int(center_rect[1])]
                cv.circle(edges, center_rect, 5, (255, 255, 0), 2)
                cv.drawContours(edges, [contour], -1, (0,255,0), 2)

            cv.imshow('Image show', edges)
            cv.waitKey(1)
            cv.imshow('Gray', gray_gray)
            cv.waitKey(1)
            cv.imshow('Dilated', dilated)
            cv.waitKey(1)
            cv.imshow('Thresh', thresh)
            cv.waitKey(1)


    def coords(self, points, points_sorted, frame):
        points_sorted = np.array(points_sorted, dtype=np.float32)

        # Maybe use cv.SOLVEPNP_P3P, but it only works for 4 points cv.SOLVEPNP_EPNP
        if len(points_sorted) == 4:
            success, rotation_vector, translation_vector = cv.solvePnP(points, points_sorted, camera_matrix, distortion_coeffs, flags=cv.SOLVEPNP_P3P)
        else:
            print("Incorrect number of corners")
        
        if success:
            rotation_matrix, _ = cv.Rodrigues(rotation_vector)
            cube_center_object_space = np.mean(points, axis=0)
            cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

            coordinates_camera_space = np.round(cube_center_camera_space.ravel()[:3]).astype(int)

            # Yaw, pitch, roll
            try:
                U, _, Vt = np.linalg.svd(rotation_matrix)
            
                # Compute the new rotation matrix
                # possible alternative is to use Singular Value Decomposition (SVD)
                # on the rotation matrix to extract the rotation angles directly.
                # This method is less susceptible to some of the numerical instability issues that can arise from Euler angle calculations. 
                R = np.dot(U, Vt)
                yaw = math.atan2(R[1, 0], R[0, 0])

                yaw = round(np.degrees(yaw))     
                

                # # Project the 3D points to 2D image points using the pose (rvec, tvec)
                # img_points, _ = cv.projectPoints(points, rotation_vector, translation_vector, camera_matrix, distortion_coeffs)

                # # Visualize the projected points on the image (assuming 'frame' is your video frame)
                # for point in img_points:
                #     x, y = point.ravel()
                #     cv.circle(frame, (int(x), int(y)), 5, (255, 255, 0), -1)  # Draw green dots
                
                
            except Exception as e:
                print(f"Error in angle calculation: {e}")
                return None

            coordinates = np.round(coordinates_camera_space.ravel()[:3]).astype(int).tolist()
            
        
            #Remove z which has index = 2
            coordinates.pop(2)
                    
            return coordinates, yaw
        else:
            print("Solve PNP error")
            return None, None

    def sort_points(self, points, number):
        
        if number == 4:
            # Step 1: Calculate the center of the points (average of all points)
            center = np.mean(points, axis=0)
            
            # Step 2: Calculate the angle of each point relative to the center
            def calculate_angle(point, center):
                dx = point[0] - center[0]
                dy = point[1] - center[1]
                return np.arctan2(dy, dx)

            # Step 3: Sort points based on the angle
            angles = [calculate_angle(point, center) for point in points]
            sorted_indices = np.argsort(angles)
            sorted_points = points[sorted_indices]
            
            # Step 4: Return the sorted points
            return sorted_points.astype(int)

        else:
            print("Invalid number of points")
            return None

    def initSlider(self):
        cv.namedWindow("Camera params",cv.WINDOW_NORMAL)
        cv.createTrackbar("H low", "Camera params", 0, 255, lambda x: None)
        cv.createTrackbar("S low", "Camera params", 0, 255, lambda x: None)
        cv.createTrackbar("V low", "Camera params", 0, 255, lambda x: None)

        cv.setTrackbarPos("H low", "Camera params", 60)
        cv.setTrackbarPos("S low", "Camera params", 40)
        cv.setTrackbarPos("V low", "Camera params", 45)
        
        cv.createTrackbar("H upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("S upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("V upper", "Camera params", 255, 255, lambda x: None)
        
        cv.createTrackbar("Canny", "Camera params", 1, 200, lambda x: None)
        cv.createTrackbar("Thresh", "Camera params", 1, 255, lambda x: None)

        cv.createTrackbar("Erosion", "Camera params", 1, 20, lambda x: None)
        cv.createTrackbar("Dilation", "Camera params", 1, 20, lambda x: None)

        cv.setTrackbarPos("Erosion", "Camera params", 4)
        cv.setTrackbarPos("Dilation", "Camera params", 4)

    def update(self):
        pass