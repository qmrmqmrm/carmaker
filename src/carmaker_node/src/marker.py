#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import tf
from std_msgs.msg import Int32, Bool
from carmaker_node.msg import localization
from visualization_msgs.msg import Marker
import method as mt

class Marker_Class():
    def __init__(self):
        self.max_markers = 100
        # self._init_markers()
        self.marker_pub = rospy.Publisher('/movo/waypoints',Marker,queue_size=1)



    def init_markers(self):
        
        br = tf.TransformBroadcaster()
    # for i in range(self.max_markers):
        br.sendTransform((0,0,0),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    "/my_frame",
                    "/map")
        pose_dict = mt.get_csv()
        cx = pose_dict['x']
        cy = pose_dict['y']
        print(type(cx),len(cx))
        marker = Marker()
        marker.header.frame_id = "/my_frame"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        
        marker.frame_locked = True
        marker.ns = "basic_shapes"
        sss = list()
        for n, i in zip(cx,cy):
            pppp =  Point()
            # print((n))
            pppp.x = n
            pppp.y = i
            pppp.z = 0
            # print("point",pppp)
            sss.append((pppp))
            # print("sss",sss)
            marker.points = sss
            # print("markerpoint",len(marker.points),marker.points[0],marker.points[-1])

        # print(marker)
        
        # print(len(marker.points),marker.points[0],marker.points[-1])
        self.marker_pub.publish(marker)
        # print("a----------------------------")
        print(marker)


def main():
    rospy.init_node("marker")
    m = Marker_Class()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.init_markers()
        r.sleep()

        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
