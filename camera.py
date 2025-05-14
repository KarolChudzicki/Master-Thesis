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

            # h_low = cv.getTrackbarPos("H low", "Camera params")
            # s_low = cv.getTrackbarPos("S low", "Camera params") 
            # v_low = cv.getTrackbarPos("V low", "Camera params") 
            # h_up = cv.getTrackbarPos("H upper", "Camera params") 
            # s_up = cv.getTrackbarPos("S upper", "Camera params") 
            # v_up = cv.getTrackbarPos("V upper", "Camera params") 
            
            # dil = cv.getTrackbarPos("Dilation", "Camera params") 
            # ero = cv.getTrackbarPos("Erosion", "Camera params") 

            canny_low = cv.getTrackbarPos("Canny", "Camera params") 
            thresh_low = cv.getTrackbarPos("Thresh", "Camera params")

            # lower_bound = (h_low, s_low, v_low)
            # upper_bound = (h_up, s_up, v_up)

            # ==================== Applying mask ====================
                       
            frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frame_blurred = cv.GaussianBlur(frame_gray, (5,5), 0)

            #_, frame_thresholded = cv.threshold(frame_blurred, thresh_low, 255, cv.THRESH_BINARY_INV)

            edges = cv.Canny(frame_blurred, canny_low, 200)

            contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        
        
    
            if contours:
                contour = max(contours, key=cv.contourArea)
                
                # Calculating contour center x coordinate
                M = cv.moments(contour)

                rect = cv.minAreaRect(contour)  # Get the bounding rectangle
                center_rect, size_rect, angle_rect = rect
                
                center_rect = [int(center_rect[0]), int(center_rect[1])]
                cv.circle(edges, center_rect, 5, (255, 255, 0), 2)

            cv.imshow('Image show', edges)
            cv.waitKey(1)


    def initSlider(self):
        cv.namedWindow("Camera params",cv.WINDOW_NORMAL)
        cv.createTrackbar("H low", "Camera params", 1, 255, lambda x: None)
        cv.createTrackbar("S low", "Camera params", 1, 255, lambda x: None)
        cv.createTrackbar("V low", "Camera params", 1, 255, lambda x: None)
        
        cv.createTrackbar("H upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("S upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("V upper", "Camera params", 255, 255, lambda x: None)
        
        cv.createTrackbar("Canny", "Camera params", 1, 200, lambda x: None)
        cv.createTrackbar("Thresh", "Camera params", 1, 255, lambda x: None)

    def update(self):
        pass