import numpy as np





class Kalman:
    def __init__(self):
        self.time_step = 0.1
        
        # For the state vector [x, y, z, vx, vy, vz]
        self.x = np.zeros((6,1))
        
        # Transition matrix A
        self.A = np.eye(6)
        
        