from filterpy.kalman import KalmanFilter
import numpy as np

class DeadReckoning:
    def __init__(self):
        self.kalmanFilter = KalmanFilter(dim_x=12, dim_z=1)


    def updatePose(linearAccel, angularVel, quaternion):
        z = np.stack((linearAccel, angularVel, orientation), axis=0)
        self.kalmanFilter.predict()
        self.kalmanFilter.update(z)


    def get_status(self):
        return 1
