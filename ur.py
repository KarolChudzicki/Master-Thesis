import socket
import math
import struct
import logging
import time
import numpy as np
import select

logging.basicConfig(level=logging.INFO)

class URRobot:
    def __init__(self, host: str = '192.38.66.227', port1: int = 30001, port2: int = 30002) -> None:
        # REMEMBER TO CHANGE IP ADDRESS IN THE PROPORTIES OF THE DEVICES CONNECTED VIA ETHERNET (!!! TO A DIFFERENT THAN HOST1 !!!)
        self.s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port1 = port1

        try:
            self.s1.connect((host,port1))
            logging.info(f"Successfully connected to {self.host}:{self.port1}")
        except socket.error as e:
            logging.error((f"Failed to connect to {self.host}:{self.port1} - {e}"))


        
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
        
    def movel(self, position, acceleration, velocity, time_to)-> None:
        command = 'movel(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(time_to) +')\n'
        self.s1.send(command.encode('utf-8'))
        time.sleep(time_to)
        
    def speed(self, jointSpeed)-> None:
        COMMAND = 'speed(' + str(jointSpeed) + ')\n'
        self.s1.send(COMMAND.encode('utf-8'))

    # def rSleep(self, time)-> None:
    #     COMMAND = 'sleep(' + str(time) + '.)\n'
    #     self.s2.send(COMMAND.encode('utf-8'))
        
    # def is_steady(self):
    #     COMMAND = 'is_steady()\n'
    #     self.s2.send(COMMAND.encode('utf-8'))
    #     response = self.s2.recv(1)
    #     print(response)
    
    
    def speedl(self, speed_vector, acceleration, duration) -> None:
        COMMAND = 'speedl(' + str(speed_vector) + ',' + str(acceleration) + ',' + str(duration) + ')\n'
        self.s1.send(COMMAND.encode('utf-8'))
        time.sleep(0.01)




