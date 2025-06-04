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

        self.camera_matrix = np.array([
            row1,
            row2,
            row3
        ])

        cleaned = re.sub(r'[\[\]]', '', lines[5])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        self.distortion_coeffs = np.array([np.fromstring(cleaned.strip(), sep=' ')])

        with open('calib_param_hand_eye.txt', 'r') as f:
            lines = f.readlines()

        cleaned = re.sub(r'[\[\]]', '', lines[17])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row1 = np.fromstring(cleaned.strip(), sep=' ')

        cleaned = re.sub(r'[\[\]]', '', lines[18])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row2 = np.fromstring(cleaned.strip(), sep=' ')

        cleaned = re.sub(r'[\[\]]', '', lines[19])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row3 = np.fromstring(cleaned.strip(), sep=' ')

        self.R_cam2gripper = np.array([
            row1,
            row2,
            row3
        ])

        cleaned = re.sub(r'[\[\]]', '', lines[21])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row1 = np.fromstring(cleaned.strip(), sep=' ')

        cleaned = re.sub(r'[\[\]]', '', lines[22])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row2 = np.fromstring(cleaned.strip(), sep=' ')

        cleaned = re.sub(r'[\[\]]', '', lines[23])
        cleaned = re.sub(r'\s+', ' ', cleaned)
        row3 = np.fromstring(cleaned.strip(), sep=' ')

        self.t_cam2gripper = np.array([
            row1,
            row2,
            row3
        ])


        self.T_cam2gripper = np.vstack([
            np.hstack([self.R_cam2gripper, self.t_cam2gripper]),
            np.array([0, 0, 0, 1])
        ])

        IC_width = 60
        IC_length = 80
        self.IC_points = np.array([
            [-IC_length/2, -IC_width/2, 0],  # Corner 1
            [-IC_length/2, IC_width/2, 0],  # Corner 2
            [IC_length/2, IC_width/2, 0],  # Corner 3
            [IC_length/2, -IC_width/2, 0]   # Corner 4
        ], dtype=np.float32)

        bottom_width = 60
        self.bottom_edge = np.array([
            [-bottom_width/2, 0, 0], # End 1
            [bottom_width/2, 0, 0] # End 2
        ])

    def connect(self, camera_id, width, height) -> None:
        self.cap = cv.VideoCapture(camera_id, cv.CAP_DSHOW) 
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
            #undistorted_img = undistorted_img[y:y+h, ((x+w)//2-width//2):((x+w)//2+width//2)]
            undistorted_img = undistorted_img[y:y+h, x:x+w]
            frame = undistorted_img

            # CROPPING THE IMAGE
            cv.rectangle(frame, (0, 0), (width//2, h), (0, 0, 0), -1)
            cv.rectangle(frame, (w - width//2, 0), (w, h), (0, 0, 0), -1)

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


            kernel = cv.getStructuringElement(cv.MORPH_RECT, (11, 11))

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
                
                rect = cv.minAreaRect(contour)  # Get the bounding rectangle
                center_rect, (rect_width, rect_height), angle_rect = rect

                rect = cv.minAreaRect(contour)           # Get the rotated rectangle
                box = cv.boxPoints(rect)                 # Get the 4 corner points
                box = box.astype(int)

                box_sorted = self.sort_points(box,4)

                # Optionally: label the points
                for i, point in enumerate(box_sorted):
                    x, y = point
                    cv.circle(frame, (x, y), 5, (0, 255, 255), -1)
                    cv.putText(frame, f"{i}", (x+5, y-5), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv.LINE_AA)

                center_rect = [int(min(center_rect)), int(max(center_rect))]

                if rect_width < rect_height:
                    rect_width, rect_height = rect_height, rect_width
                    angle_rect = angle_rect - 90

                #print(center_rect, rect_width, rect_height, angle_rect)
                
                cv.circle(edges, center_rect, 5, (255, 255, 0), 2)

                # Given: pt1_2d, pt2_2d (pixels), camera matrix K, known length L, and T_cam_to_robot

                # 1. Convert 2D points to normalized rays
                K_inv = np.linalg.inv(self.camera_matrix)
                P1 = np.array([box_sorted[0][0], box_sorted[0][1], 1])
                P2 = np.array([box_sorted[1][0], box_sorted[1][1], 1])
                #print("POINTS:", P1, P2)

                ray1 = K_inv @ P1
                ray2 = K_inv @ P2
                # ray1 /= np.linalg.norm(ray1)
                # ray2 /= np.linalg.norm(ray2)
                # 2. Calculate scale (depth) d
                d = 0.06 / np.linalg.norm(ray1 - ray2)
                

                # 3. Calculate 3D points in camera frame
                pt1_3d_cam = ray1 * d
                pt2_3d_cam = ray2 * d

                #print("Points camera: ",pt1_3d_cam, pt2_3d_cam)

                middle_3d_cam = (pt1_3d_cam + pt2_3d_cam) / 2
                #print("Middle: ", middle_3d_cam)
                middle_3d_cam = self.R_cam2gripper @ middle_3d_cam
                #print("Middle: ", middle_3d_cam)
                # 4. Convert to robot frame
                #pt1_3d_robot = self.T_cam2gripper @ np.array([middle_3d_cam[0], middle_3d_cam[1], middle_3d_cam[2], 1])
                # pt2_3d_robot = self.T_cam2gripper @ np.array([pt2_3d_cam[0], pt2_3d_cam[1], pt2_3d_cam[2], 1])
                
                pt_3d_robot = self.t_cam2gripper.T - middle_3d_cam
                pt_3d_robot[0][0] += 0.04


    
                # pt1_3d_robot = np.round(pt1_3d_robot, 3)
                # pt2_3d_robot = np.round(pt2_3d_robot, 3)

                # Draw coordinate system
                cv.arrowedLine(frame, (10,10), (10,60), (255,255,0), 2)
                cv.putText(frame, 'Y', (15,60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
                cv.arrowedLine(frame, (10,10), (60,10), (255,0,255), 2)
                cv.putText(frame, 'X', (50,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
                cv.circle(frame, (w//2, h//2), 50, (255, 255, 255), 2)
                cv.imshow('Edges', edges)
                cv.waitKey(1)
                cv.imshow('Edges', gray_gray)
                cv.waitKey(1)
                cv.imshow('Frame', frame)
                cv.waitKey(1)

                area = rect_height*rect_width
                return pt_3d_robot[0], area
                #print("Points: ",pt1_3d_robot, pt2_3d_robot, (pt1_3d_robot+pt2_3d_robot)/2)
                #print("Box sorted: ", box_sorted[0], box_sorted[1])

            else:
                cv.imshow('Frame', frame)
                cv.waitKey(1)
                return np.array([0, 0, 0]), 0
            


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
        cv.setTrackbarPos("S low", "Camera params", 55)
        cv.setTrackbarPos("V low", "Camera params", 70)
        
        cv.createTrackbar("H upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("S upper", "Camera params", 255, 255, lambda x: None)
        cv.createTrackbar("V upper", "Camera params", 255, 255, lambda x: None)
        
        cv.createTrackbar("Canny", "Camera params", 1, 200, lambda x: None)
        cv.createTrackbar("Thresh", "Camera params", 1, 255, lambda x: None)

        cv.createTrackbar("Erosion", "Camera params", 1, 20, lambda x: None)
        cv.createTrackbar("Dilation", "Camera params", 1, 20, lambda x: None)

        cv.setTrackbarPos("Erosion", "Camera params", 3)
        cv.setTrackbarPos("Dilation", "Camera params", 2)

    def update(self):
        pass