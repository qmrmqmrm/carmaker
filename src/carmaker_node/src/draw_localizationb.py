#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from carmaker_node.msg import localization, localization_2

count = 0
mode = 0

class Tf_test:
    def start(self,data):
        self.local_x = data.x
        self.local_y = data.y
        self.local_theta = data.theta


def start(data):
    br = tf.TransformBroadcaster()
    x, y, z, w = tf.transformations.quaternion_from_euler(0,0,data.theta)
    br.sendTransform((data.x,data.y,1.7),
                    (x, y, z, w),
                    rospy.Time.now(),
                    "/gps_link_2",
                    "/localization_2")
    x, y, z, w = tf.transformations.quaternion_from_euler(0,0,0)
    br.sendTransform((-0.9,0.0,-1.7),
                    (x, y, z, w),
                    rospy.Time.now(),
                    "/base_link_2",
                    "/gps_link_2")


def main():
    rospy.init_node('send_mani')
    rospy.Subscriber('/localization_2', localization_2, start, queue_size=1)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
"""
        try:
            (trans, rot) = listener.lookupTransform('/localizaion', '/Wheel_Rear', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            continue
"""
