import socket
import math
import struct
import logging
import time
import numpy as np
import select

logging.basicConfig(level=logging.INFO)

class URRobot:
    def __init__(self, host: str = '192.38.66.227', port1: int = 30001, port2: int = 30002, port3: int = 30003) -> None:
        # REMEMBER TO CHANGE IP ADDRESS IN THE PROPORTIES OF THE DEVICES CONNECTED VIA ETHERNET (!!! TO A DIFFERENT THAN HOST1 !!!)
        self.s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port1 = port1
        self.port2 = port2
        self.port3 = port3
        try:
            self.s1.connect((host,port1))
            logging.info(f"Successfully connected to {self.host}:{self.port1}")
        except socket.error as e:
            logging.error((f"Failed to connect to {self.host}:{self.port1} - {e}"))

        try:
            self.s2.connect((host,port2))
            logging.info(f"Successfully connected to {self.host}:{self.port2}")
        except socket.error as e:
            logging.error((f"Failed to connect to {self.host}:{self.port2} - {e}"))

        try:
            self.s3.connect((host,port3))
            logging.info(f"Successfully connected to {self.host}:{self.port3}")
        except socket.error as e:
            logging.error((f"Failed to connect to {self.host}:{self.port3} - {e}"))
        
    def movej(self, position, acceleration, velocity) -> None:
        # Convert degrees to radians
        position = [pose * math.pi/180.0 for pose in position]
        COMMAND = 'movej(' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ')\n'
        print(COMMAND.encode('utf-8'))
        self.s1.send(COMMAND.encode('utf-8'))

    def movep(self, position, acceleration, velocity) -> None:
        COMMAND = 'movep(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(0.001) + ')\n'
        print(COMMAND.encode('utf-8'))
        self.s1.send(COMMAND.encode('utf-8'))
        
    def movel(self, position, acceleration, velocity, time)-> None:
        command = 'movel(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(time) +')\n'
        self.s1.send(command.encode('utf-8'))
        
    def speed(self, jointSpeed)-> None:
        COMMAND = 'speed(' + str(jointSpeed) + ')\n'
        self.s1.send(COMMAND.encode('utf-8'))

    def rSleep(self, time)-> None:
        COMMAND = 'sleep(' + str(time) + '.)\n'
        self.s2.send(COMMAND.encode('utf-8'))
        
    def is_steady(self):
        COMMAND = 'is_steady()\n'
        self.s2.send(COMMAND.encode('utf-8'))
        response = self.s2.recv(1)
        print(response)
    
    
    def recv_all(self, n):
        data = b''
        while len(data) < n:
            packet = self.s3.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def flush_socket(self):
        self.s3.setblocking(0)  # non-blocking mode
        try:
            while True:
                data_now = self.recv_all(812)
                if not data_now:
                    break
                else:
                    data_prev = data_now
        except BlockingIOError:
            # No more data available to read
            pass
        finally:
            self.s3.setblocking(1)  # back to blocking mode
            print(len(data_prev))
            
            

    def current_Position(self) -> list[float]:
        """
        Retrieves the current TCP position of the robot.

        Returns:
            list[float]: A list containing the robot's position in decimal format,
                         rounded to a specified number of decimal points.
        """
        STARTING_BYTE = 72
        ENDING_BYTE = 78
        POSITION_DECIMAL_POINTS = 5
        OFFSET = 12
        COMMAND  = 'get_actual_tcp_pose()\n'
        

        
        #self.s1.send(COMMAND.encode('utf-8'))
        # VERY IMPORTANT TO SET THE VALUE TO 812!!

        self.s3.settimeout(0.1)  # wait max 0.5 seconds on recv
        robot_position = []

        try:
            response = self.recv_all(812)
            
            print(f"Received {len(response)} bytes")
            for x in range (STARTING_BYTE, ENDING_BYTE):
                val = OFFSET + 8 * x
                position = struct.unpack('>d', response[val:val+8])
                position = round(float(position[0]),POSITION_DECIMAL_POINTS)
                robot_position.append(position)
        except socket.timeout:
            print("No data received in timeout")
        

    
        return robot_position





