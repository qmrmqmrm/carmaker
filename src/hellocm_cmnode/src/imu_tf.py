#!/usr/bin/env python
import rospy
from hellocm_msgs.msg import UDP, imu_UDP
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math



class Transformation:
    def __init__(self):
        rospy.Subscriber('/imu_udp', imu_UDP, self.callback_imu)
        rospy.Subscriber('/udp', UDP, self.callback_udp)
        self.imu_msg = imu_UDP()
        self.udp_msg = UDP()


    def callback_imu(self, msg):
        self.imu_msg = msg
        br = tf.TransformBroadcaster()
        ori =self.udp_msg.orientation

    def callback_udp(self, msg):
        self.udp_msg = msg


    def transform(self):

        br = tf.TransformBroadcaster()
        ori =self.udp_msg.orientation
        orientation = [self.udp_msg.orientation.x, self.udp_msg.orientation.y, self.udp_msg.orientation.z, self.udp_msg.orientation.w]
        br.sendTransform((2.5226,0,0.5424),
                    (orientation),
                    rospy.Time.now(),
                    "imu",
                    "Fr1A")   


def main():
    rospy.init_node("imu_tf")
    con = Transformation()
    print("imu_tf_node")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        con.transform()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

