import socket
import math
import logging
import time
import numpy as np

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

    def movel(self, position, acceleration, velocity, time_to)-> None:
        command = 'movel(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(time_to) +')\n'
        self.s1.send(command.encode('utf-8'))
        time.sleep(time_to + 0.1)
 
 
    def speedl(self, speed_vector, acceleration, duration) -> None:
        COMMAND = 'speedl(' + str(speed_vector) + ',' + str(acceleration) + ',' + str(duration) + ')\n'
        self.s1.send(COMMAND.encode('utf-8'))
        time.sleep(0.02)    

    # def movej(self, position, acceleration, velocity) -> None:
    #     # Convert degrees to radians
    #     position = [pose * math.pi/180.0 for pose in position]
    #     COMMAND = 'movej(' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ')\n'
    #     print(COMMAND.encode('utf-8'))
    #     self.s1.send(COMMAND.encode('utf-8'))

    # def movep(self, position, acceleration, velocity) -> None:
    #     COMMAND = 'movep(' + 'p' + str(position) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(0.001) + ')\n'
    #     print(COMMAND.encode('utf-8'))
    #     self.s1.send(COMMAND.encode('utf-8'))
        

        
    # def movec(self, position_via, position_to, acceleration, velocity, radius) -> None:
    #     COMMAND = 'movec(' + 'p' + str(position_via) + ',' + 'p' + str(position_to) + ',' + str(acceleration) + ',' + str(velocity) + ',' + str(radius) + ')\n'
    #     print(COMMAND.encode('utf-8'))
    #     self.s1.send(COMMAND.encode('utf-8'))
    
    # def forcemode(self, task_frame, selection_vector, wrench, type, limits) -> None:
    #     COMMAND = 'force_mode(' + 'p' + str(task_frame) + ',' + str(selection_vector) + ',' + str(wrench) + ',' + str(type) + ',' + str(limits) + ')\n'
    #     print(COMMAND.encode('utf-8'))
    #     self.s1.send(COMMAND.encode('utf-8'))
    
    # def endforcemode(self) -> None:
    #     COMMAND = 'end_force_mode()\n'
    #     print(COMMAND.encode('utf-8'))
    #     self.s1.send(COMMAND.encode('utf-8'))
        
    # def speed(self, jointSpeed)-> None:
    #     COMMAND = 'speed(' + str(jointSpeed) + ')\n'
    #     self.s1.send(COMMAND.encode('utf-8'))

    
    
    





# # Test
# rob = URRobot()
# p = [0.15, 0.766, 0.081, 3.1415, 0, 0]
# # rob.movel(p, 0.1, 0.1, 3)
# # time.sleep(2)

# frame = [0,0,0,0,0,0]
# vector = [0,0,1,0,0,0]
# wrench = [0,0,-40,0,0,0]
# type = 2
# limits = [0.1,0.1,0.1,0.1,0.1,0.1]

# rob.forcemode(frame, vector, wrench, type, limits)
# time.sleep(3)
# rob.endforcemode()

# print("Finished")