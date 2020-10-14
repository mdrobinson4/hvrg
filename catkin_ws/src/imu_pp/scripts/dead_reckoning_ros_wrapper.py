#!/usr/bin/env python
import rospy
import message_filters
from imu_pp.deadReckoning import DeadReckoning
from  geometry_msgs.msg import Vector3Stamped, QuaternionStamped, Point

class DeadReckoningROSWRapper:
    def __init__(self):
        self.deadReckoning = DeadReckoning()
        # subscribe to 3d linear acceleration topic
        self.linearAccelSub = message_filters.Subscriber('linear_accel', Vector3Stamped)
        # subscribe to 3d angular velocity topic
        self.angularVelSub = message_filters.Subscriber('angular_vel', Vector3Stamped)
        # synchronize topics to one callback
        self.orientationSub = message_filters.Subscriber('orientation', QuaternionStamped)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.linearAccelSub, self.angularVelSub, self.orientationSub], 10, 0.1)
        self.ts.registerCallback(self.imuCallback)
        print(self.ts)
    
    def stop(self):
        self.deadReckoning.stop()
    
    def imuCallback(self, linearAccel, angularVel, orientation):
        print('linearAccel')
        print(linearAccel)
        print('angularVel')
        print(angularVel)
        print('orientation')
        print(orientation)
        #self.deadReckoning.updatePose(linearAccel, angularVel, orientation)

if __name__ == "__main__":
    # initialize ros node
    rospy.init_node("dead_reckoning")
    # create class for ros wrapper
    deadReckoning = DeadReckoningROSWRapper()
    
    rospy.on_shutdown(deadReckoning.stop)
    rospy.loginfo("deadreckoning is now started, ready to do preprocessing.")
    rospy.spin()