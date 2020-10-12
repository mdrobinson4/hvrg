#!/usr/bin/env python
import rospy
import message_filters
from imu_pp.deadReckoning import DeadReckoning
from  geometry_msgs.msg import (Vector3, Quaternion)

class DeadReckoningROSWRapper:
    def __init__(self):
        self.deadReckoning = DeadReckoning()
        # subscribe to 3d linear acceleration topic
        self.linearAccelSub = message_filters.Subscriber('linearAcceleration', Vector3)
        # subscribe to 3d angular velocity topic
        self.angularVelSub = message_filters.Subscriber('angularVelocity', Vector3)
        # synchronize topics to one callback
        self.quaternionSub = message_filters.Subscriber('quaternion', Quaternion)
        self.ts = message_filters.TimeSynchronizer([self.linearAccelSub, self.angularVelSub, self.quaternionSub], 10)
        self.ts.registerCallback(self.imuCallback)
    
    def stop(self):
        self.deadReckoning.stop()
    
    def imuCallback(linearAccel, angularVel, quaternion):
        print('linear acceleration: {} | angular velocity: {}'.format(linearAccel, angularVel))
        self.deadReckoning.updatePose(linearAccel, angularVel, quaternion)

if __name__ == "__main__":
    # initialize ros node
    rospy.init_node("dead_reckoning")
    # create class for ros wrapper
    deadReckoning = DeadReckoningROSWRapper()
    
    rospy.on_shutdown(deadReckoning.stop)
    rospy.loginfo("deadreckoning is now started, ready to do preprocessing.")
    rospy.spin()