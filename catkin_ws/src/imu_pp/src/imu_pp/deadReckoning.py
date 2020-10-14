from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

class DeadReckoning:
    def __init__(self):
        self.timestamp = 0.0
        self.dt = 0
        self.position = np.array([0, 0, 0])
        self.velocity = np.array([0, 0, 0])
        self.acceleration = np.array([0, 0, 0])

    def processData(self, linearAccel, angularVel):
        linearAccel = np.array([linearAccel.x, linearAccel.y, linearAccel.z])
        angularVel = np.array([angularVel.x, angularVel.y, angularVel.z])
        return (linearAccel, angularVel)

    def updatePose(self, linearAccel, angularVel, quaternion, tme):
        (linearAccel, angularVel) = self.processData(linearAccel, angularVel)
        if self.timestamp == 0.0:
            self.timestamp = tme
            self.acceleration = linearAccel
        else:
            self.dt = tme - self.timestamp
            self.position = self.position + self.velocity*self.dt + (1/2)*self.acceleration*math.pow(self.dt, 2)
            self.velocity = self.velocity  + (self.acceleration + linearAccel) * (self.dt * 0.5)
            print(self.acceleration)
            
            self.acceleration = linearAccel
            self.timestamp = tme
            


    def get_status(self):
        return 1
