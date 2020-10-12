from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import numpy as np

class DeadReckoning:
    def __init__(self):
        self.kalmanFilter = KalmanFilter(dim_x=12, dim_z=1)


    def updatePose(linearAccel, angularVel, quaternion):
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=False)
        z = np.stack((linearAccel, angularVel, euler), axis=0)
        self.kalmanFilter.predict()
        self.kalmanFilter.update(z)
        self.pose = self.kalmanFilter.x


    def get_status(self):
        return 1
