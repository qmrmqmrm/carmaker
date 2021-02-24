#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import os
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class yellow_lane():
    def __init__(self):
        print("[start ] Yellow lane detector")
        pass

    def main(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detector(cv_image)
        print("---------------------------")

    def detector(self,cv_image):
        print("[start ] detector func.")
        yellow_lane_pub = rospy.Publisher("/yellow_lane",String,queue_size=1)
        yellow_lane_dist_pub = rospy.Publisher("/yellow_lane_dist",String,queue_size=1)
        stop_line_pub = rospy.Publisher("/stop_line",String,queue_size=1)
        
        Frame = cv2.resize(cv_image,(922,620),interpolation=cv2.INTER_CUBIC)
        copy_img = np.copy(Frame[300:450,300:650,:])
        mark_img = np.copy(Frame[300:450,300:650,:])
        hsv_img = cv2.cvtColor(copy_img,cv2.COLOR_BGR2HSV)

        low_yellow = np.array([18,94,140])
        up_yellow = np.array([48,255,255])
        yellowmask = cv2.inRange(hsv_img,low_yellow,up_yellow)
        yellow_edges = cv2.Canny(yellowmask,75,150)

        yellow_R = np.copy(yellow_edges[:,175:])
        yellow_L = np.copy(yellow_edges[:,:175])

        if 255 in yellow_R  and 255 in yellow_L:
            yellow_lane_pub.publish("all")
            array_idx = np.where(yellow_L == 255)
            idx = array_idx[1][0]

        elif 255 in yellow_R:
            yellow_lane_pub.publish("right")
            array_idx = np.where(yellow_R == 255)
            idx = array_idx[1][0] + 175

        elif 255 in yellow_L:
            yellow_lane_pub.publish("left")
            array_idx = np.where(yellow_L == 255)
            idx = array_idx[1][0]

        else:
            yellow_lane_pub.publish("no")
            idx = 0

        dist = 175 - idx
        dist = abs(dist)

        #-------------------------------- 제어 테스트 해보시면서, ok/close/far 판단 지금은 아래 숫자들로 하시면됩니다.(기준은 87)
        if 44 < dist <130:
            yellow_lane_dist_pub.publish("ok")

        elif dist <= 44:
            yellow_lane_dist_pub.publish("close")
            
        elif 175 > dist >=130:
            yellow_lane_dist_pub.publish("far")

        else:
            yellow_lane_dist_pub.publish("no")

        blue_threshold = 140
        green_threshold = 140
        red_threshold = 140
        bgr_threshold = [blue_threshold, green_threshold, red_threshold]
        thresholds = (copy_img[:,:,0] < bgr_threshold[0]) \
                        | (copy_img[:,:,1] < bgr_threshold[1]) \
                        | (copy_img[:,:,2] < bgr_threshold[2])
        mark_img[thresholds] = [0,0,0]
        white_edges = cv2.Canny(mark_img,75,150)
        cut_white_edges = np.copy(white_edges[:,40:])
        stop_lines = cv2.HoughLinesP(cut_white_edges, 1, math.pi/2, 2, None, 30, 1 )
            
        if stop_lines is not None:
            stop_line_pub.publish("stop")
        else:
            stop_line_pub.publish("no")
        


if __name__ == "__main__":
    rospy.init_node("yellow_lane_detection", anonymous=True)
    Y = yellow_lane()
    rospy.Subscriber("/vds_node_localhost_2218/image_raw",Image, Y.main, queue_size = 2 )
    rospy.spin()
