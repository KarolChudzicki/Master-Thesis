import socket
import math
import struct
import logging
import time
import numpy as np
import select

logging.basicConfig(level=logging.INFO)

class URRobot:
    def __init__(self, host: str = '192.38.66.227', port: int = 30003) -> None:
        # REMEMBER TO CHANGE IP ADDRESS IN THE PROPORTIES OF THE DEVICES CONNECTED VIA ETHERNET (!!! TO A DIFFERENT THAN HOST1 !!!)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port
        try:
            self.s.connect((host,port))
            logging.info(f"Successfully connected to {self.host}:{self.port}")
        except socket.error as e:
            logging.error((f"Failed to connect to {self.host}:{self.port} - {e}"))
        
    def movej(self, position, acceleration, velocity) -> None:
        # Convert degrees to radians
        position = [pose * math.pi/180.0 for pose in position]
        COMMAND = 'movej(' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ')\n'
        print(COMMAND.encode('utf-8'))
        self.s.send(COMMAND.encode('utf-8'))

    def movep(self, position, acceleration, velocity) -> None:
        COMMAND = 'movep(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(0.001) + ')\n'
        print(COMMAND.encode('utf-8'))
        self.s.send(COMMAND.encode('utf-8'))
        
    def movel(self, position, acceleration, velocity, time)-> None:
        command = 'movel(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(time) +')\n'
        self.s.send(command.encode('utf-8'))
        
    def speed(self, jointSpeed)-> None:
        COMMAND = 'speed(' + str(jointSpeed) + ')\n'
        self.s.send(COMMAND.encode('utf-8'))

    def rSleep(self, time)-> None:
        COMMAND = 'sleep(' + str(time) + '.)\n'
        self.s.send(COMMAND.encode('utf-8'))
        
    def is_steady(self):
        COMMAND = 'is_steady()\n'
        self.s.send(COMMAND.encode('utf-8'))
        response = self.s.recv(1)
        print(response)
    
    def reset_socket(self):
        try:
            self.s.close()  # Close the current socket
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a new socket
            self.s.connect((self.host, self.port))  # Reconnect to the same address and port
        except socket.error as e:
            print(f"Error resetting socket: {e}")
    
    def flush_socket(self):
        """Flushes any leftover data in the socket buffer."""
        self.s.setblocking(0)  # Non-blocking mode
        try:
            while True:
                ready = select.select([self.s], [], [], 0)
                if ready[0]:
                    _ = self.s.recv(1024)
                else:
                    break  # No more data to read
        except socket.error:
            pass
        finally:
            self.s.setblocking(1)  # Restore blocking mode

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
        
        
        self.flush_socket()
        self.s.sendall(COMMAND.encode('utf-8'))
        # VERY IMPORTANT TO SET THE VALUE TO 812!!!

        response = self.s.recv(812)
        
        robot_position = []

        
        
        
        for x in range (STARTING_BYTE, ENDING_BYTE):
            val = OFFSET + 8 * x
            position = struct.unpack('>d', response[val:val+8])
            position = round(float(position[0]),POSITION_DECIMAL_POINTS)
            robot_position.append(position)
        
    
    
        return robot_position





