import cv2 as cv
import time
import numpy as np
import re
import json
import time
import logging

logging.basicConfig(level=logging.INFO)


class Camera:
    def __init__(self, camera_id, width, height):
        self.cap = cv.VideoCapture(camera_id, cv.CAP_DSHOW) 
        # Set desired resolution
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.width = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))


        # Open a file with camera calibration parameters
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

        # Open a file with hand-eye calibration parameters
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

        self.update_params()
        
        self.prev_box_sorted = None
        self.last_coordinates = None
        self.no_coordinates = 0
        self.max_coords_history_len = 3
        self.coords_history = []
        self.coords_history_on = False
        
        # Sizes of objects for solve PNP
        # Bottom casing
        self.object_points1 = np.array([
            [-0.03, -0.04, 0], # Bottom left
            [-0.03,  0.04, 0], # Bottom right
            [ 0.03,  0.04, 0], # Top right
            [ 0.03,  -0.04, 0] # Top left
        ])
        
        # Integrated circuit
        self.object_points2 = np.array([
            [-0.023, -0.033, 0], # Bottom left
            [-0.023,  0.033, 0], # Bottom right
            [ 0.023,  0.033, 0], # Top right
            [ 0.023, -0.033, 0] # Top left
        ])
        
        # Top cover
        self.object_points1 = np.array([
            [-0.03, -0.04, 0], # Bottom left
            [-0.03,  0.04, 0], # Bottom right
            [ 0.03,  0.04, 0], # Top right
            [ 0.03,  -0.04, 0] # Top left
        ])
        
        self.new_camera_matrix, self.roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coeffs, (self.width, self.height), 1, (self.width, self.height))

    def update_params(self):
        # Load JSON data from file
        self.data_json = [None]*3
        with open("slider_params_TopCover.json", "r") as file:
            self.data_json[2] = json.load(file)

        with open("slider_params_IntegratedCircuit.json", "r") as file:
            self.data_json[1] = json.load(file)

        with open("slider_params_BottomCasing.json", "r") as file:
            self.data_json[0] = json.load(file)
        print("Params updated")
        


    def capture(self, width, part_number, show_or_not, from_json, params) -> np.array:

        (ret, frame) = self.cap.read()
        self.fps = self.cap.get(cv.CAP_PROP_FPS)
        if ret:
            # ==================== Undistort the image ====================
            undistorted_img = cv.undistort(frame, self.camera_matrix, self.distortion_coeffs, None, self.new_camera_matrix)
            
            # ==================== Crop the image to the valid region of interest ====================
            x, y, w, h = self.roi
            undistorted_img = undistorted_img[y:y+h, x:x+w]
            frame = undistorted_img

            # CROPPING THE IMAGE
            cv.rectangle(frame, (0, 0), (width//2, h), (0, 0, 0), -1)
            cv.rectangle(frame, (w - width//2, 0), (w, h), (0, 0, 0), -1)
            
            

            # ==================== Get parameters ====================

            if from_json:
            
                h_low = self.data_json[part_number]["H low"]
                s_low = self.data_json[part_number]["S low"]
                v_low = self.data_json[part_number]["V low"]
                h_up = self.data_json[part_number]["H up"]
                s_up = self.data_json[part_number]["S up"]
                v_up =  self.data_json[part_number]["V up"]
                    
                dil = self.data_json[part_number]["Dilation"]
                ero = self.data_json[part_number]["Erosion"]

            else:
                h_low = params[0]
                s_low = params[1]
                v_low = params[2]
                h_up = params[3]
                s_up = params[4]
                v_up =  params[5]
                    
                dil = params[6]
                ero = params[7]

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
            contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
           
           
            #Filter out unwanted contours
            contours = [cnt for cnt in contours if cv.contourArea(cnt) > 3000]

            
            
            return contours, edges, thresh, frame, gray_gray    
        


    def calculate_coords_rect_center(self, contours, frame, part_number):
        if not contours:
            if self.coords_history_on:
                coords_history_len = len(self.coords_history)
                if coords_history_len > 0 and self.no_coordinates < coords_history_len:
                    coordinates = self.coords_history[self.no_coordinates]
                else:
                    coordinates = np.array([0,0,0])
                
                self.no_coordinates += 1
                return coordinates, 0, frame, 0
            else:
                coordinates = np.array([0,0,0])
                return coordinates, 0, frame, 0
        else:
            self.no_coordinates = 0
        

        contour = max(contours, key=cv.contourArea)
        rect = cv.minAreaRect(contour)
        center_rect, (rect_width, rect_height), angle_rect = rect

        box = cv.boxPoints(rect)
        box = box.astype(int)
        box_sorted = self.sort_points(box, 4)
        
        box_sorted_smoothed = box_sorted
        
        for i, point in enumerate(box_sorted_smoothed):
            x, y = point
            cv.circle(frame, (x, y), 5, (0, 255, 255), -1)
            cv.putText(frame, f"{i}", (x + 5, y - 5), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)

        center_rect = [int(max(center_rect)), int(min(center_rect))]

        if rect_width < rect_height:
            rect_width, rect_height = rect_height, rect_width
            angle_rect -= 90

        image_points = np.array(box_sorted_smoothed, dtype=np.float32)

        # 3D reconstruction
        if part_number == 0: known_Z = 0.3 - 0.003
        elif part_number == 1: known_Z = 0.3 - 0.002
        elif part_number == 2: known_Z = 0.3 - 0.040
        else:
            # Error
            return None
        
        # Convert center to homogeneous coordinates
        u, v = center_rect
        
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32)

        # Invert camera matrix to back-project
        inv_K = np.linalg.inv(self.camera_matrix)
        normalized_coords = inv_K @ pixel_vector

        # Scale by known Z
        X = normalized_coords[0] * known_Z
        Y = normalized_coords[1] * known_Z
        Z = known_Z

        center_3d = np.array([X, Y, Z])

        # Transform to robot coordinates
        center_3d_robot = self.R_cam2gripper @ center_3d.reshape(3, 1)  
        center_3d_robot = self.t_cam2gripper - center_3d_robot
        center_3d_robot = center_3d_robot.flatten()

        coordinates = np.round(center_3d_robot, 7).tolist()


        # Drawing
        h, w = frame.shape[:2]
        cv.arrowedLine(frame, (10, 10), (10, 60), (255, 255, 0), 2)
        cv.putText(frame, 'Y', (15, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv.arrowedLine(frame, (10, 10), (60, 10), (255, 0, 255), 2)
        cv.putText(frame, 'X', (50, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        cv.circle(frame, (w // 2, h // 2), 50, (255, 255, 255), 2)


        area = rect_height * rect_width
        
        
        coordinates = center_3d_robot.tolist()
        
        
        # Keep a small history of coordinates just in case there are 1-3 missing
        self.coords_history.append(coordinates)
        if len(self.coords_history) > self.max_coords_history_len:
            self.coords_history.pop(0)

        
        return coordinates, area, frame, angle_rect        
    
    def get_object_area(self, contours, part_number):
        if not contours:
            return 0, None
        

        contour = max(contours, key=cv.contourArea)
        rect = cv.minAreaRect(contour)
        center_rect, (rect_width, rect_height), angle_rect = rect

        center_rect = [int(max(center_rect)), int(min(center_rect))]
        
        if rect_width < rect_height:
            rect_width, rect_height = rect_height, rect_width
            angle_rect -= 90


        area = rect_height * rect_width

        return area, angle_rect

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
        
    
    def capture_and_get_coords_center(self, part_number):
        contours, edges, thresh, frame, gray_gray = self.capture(width=600, part_number=part_number, show_or_not=False, from_json=True, params=0)
        coords, area, frame, angle = self.calculate_coords_rect_center(contours=contours, frame=frame, part_number=part_number)
        return coords, area, frame, angle

    def capture_and_get_object_area_and_angle(self, part_number):
        contours, edges, thresh, frame, gray_gray = self.capture(width=600, part_number=part_number, show_or_not=False, from_json=True, params=0)
        area, angle = self.get_object_area(contours=contours, part_number=part_number)
        return area, angle

    def identifyingPart(self):
        area, angle = self.capture_and_get_object_area_and_angle(0)
        part_number = 0
        # Loop to check if something is in the image
        while area < 20000:
            part_number += 1
            if part_number >= 3:
                part_number = 0
            area, angle = self.capture_and_get_object_area_and_angle(part_number)
        
        #print(part_number)
        
        area_array = []
        angle_array = []
        area_captures = 3
        # Loop to double check the area
        loopTimeout = time.time()
        while len(area_array) < area_captures:
            area, angle = self.capture_and_get_object_area_and_angle(part_number)
            if area is not None or area > 0:
                area_array.append(area)
                if angle is not None:
                    angle_array.append(angle)
            
            if time.time() - loopTimeout > 15:
                print("Part detection loop timeout")
                return None, None
        
        if area_array and angle_array:
            # Filter out too small values
            max_area = max(area_array)
            filtered_areas = [a for a in area_array if abs(a - max_area) <= 4000] 
            
            # Filter out incorrect
            positive_count_angle = sum(1 for x in angle_array if x > 0)
            negative_count_angle = sum(1 for x in angle_array if x < 0)
            
            if positive_count_angle >= negative_count_angle:
                # Positive
                angle_array = [abs(x) for x in angle_array]
                max_angle = max(angle_array)
                # Filter out angles that are further than 5 deg from max_angle
                filtered_angles = [a for a in angle_array if abs(a - max_angle) <= 5]  
                
            else:
                # Negative
                angle_array = [-abs(x) for x in angle_array]
                # Filter out angles that are further than 5 deg from max_angle
                min_angle = min(angle_array)
                filtered_angles = [a for a in angle_array if abs(a - min_angle) <= 5]  
                
            
            

            angle = sum(filtered_angles) / len(filtered_angles)
            
            area = sum(filtered_areas) / len(filtered_areas)
            
            
            if area > 22000 and area <= 34000:
                part = 1
            elif area > 34000 and area <= 48000:
                part = 0
            elif area > 48000 and area <= 75000:
                part = 2
            else:
                part = None
                        
            return part, angle
        
        else:
            logging.warning("Area or/and angle arrays are empty")
            return None, None
            

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

        cv.createTrackbar("Erosion", "Camera params", 1, 20, lambda x: None)
        cv.createTrackbar("Dilation", "Camera params", 1, 20, lambda x: None)

        cv.setTrackbarPos("Erosion", "Camera params", 3)
        cv.setTrackbarPos("Dilation", "Camera params", 2)

    def update(self):
        pass