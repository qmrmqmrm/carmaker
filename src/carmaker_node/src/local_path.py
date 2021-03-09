#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from carmaker_node.msg import *
from carmaker_tracking.msg import *
from config import Config

from draw_marker import Marker_Class
import numpy as np
import pandas as pd
import math

from std_msgs.msg import *

R = 6378.1  # Radius of the Earth


class local_path(object):
    def __init__(self):
        self.val_init()
        print("here local_path class")

    def val_init(self):
        self.ob_count = 0
        self.ob_lidar = []
        self.path_x = []
        self.path_y = []
        self.lidar_path = []
        self.marker = Marker_Class()
        self.em_state = 0
        # =========================================== #
        self.path_start = 0
        self.send_count = 2.0
        self.objec =[]
        self.path_make = None
        self.play_list = []
        self.aroow_mode = 0
        self.ob_right_dect = 0
        self.ob_left_dect = 0
        self.ob_l_dect = 0
        self.ob_r_dect = 0
        self.guard_rail =[]
        
        
    def pub_msg(self):
        pubMsg = lo_point()
        pubMsg.x_point = self.path_x
        pubMsg.y_point = self.path_y
        pubMsg.state = self.send_count
        self.marker.init_markers(self.path_x,self.path_y)
        self.lo_point_pub.publish(pubMsg)
        
        self.path_x= []
        self.path_y =[]
        self.path_start = 0
        self.send_count = 2.0
        self.ob_count = 0

        self.ob_lidar = []
        self.lidar_path = []
        self.aroow_mode = 30
        self.play_list = []
        self.objec =[]
        self.ob_right_dect = 0
        self.ob_left_dect = 0
        self.ob_l_dect = 0
        self.ob_r_dect = 0
        self.guard_rail = []
        

    def yellow(self, data):
        if data.data == "left":
            self.path_make = "right"
        if data.data == "right":
            self.path_make = "left"
        if data.data == "no":
            self.path_make = "no"


    def local_path_planning(self):
        #self.val_init()
        rospy.Subscriber('/localization_2', localization_2, self.locali_sub)
        rospy.Subscriber('/udp', UDP, self.udp_sub)
        rospy.Subscriber('/axis_distance_msg', pointInformationarray, self.lidar_point_op)
        rospy.Subscriber('/Emergency_state', emergency_state, self.emer_state_sub)
        rospy.Subscriber('/Local_state', Tolocal, self.local_end_state)
        rospy.Subscriber('/yellow_lane', String, self.yellow)
        self.lo_point_pub = rospy.Publisher('/lo_point', lo_point, queue_size=10)

        if self.path_start == 1:
            print("start local path")
            self.send_count = 0.0
            
            self.get_csv()
            self.csv_global()
            self.delete_point()

            self.round_x = np.array(self.local_list_x)
            self.round_y = np.array(self.local_list_y)

            self.lidar_real_point()

            self.lidar_path = np.array(self.lidar_path)
            for i in range(len(self.lidar_path)):
                min_x = self.lidar_path[i, 0]
                min_y = self.lidar_path[i, 1]

                self.min_path(min_x, min_y)

            self.pub_msg()
            
            

    
    def delete_point(self):
        for x, y in zip(self.local_list_x, self.local_list_y):
            if self.pule_x > 0 and self.pule_y > 0:  ## x +++ // y +++
                if x < self.car_point_x and y < self.car_point_y:
                    self.mode_state = 1
                    self.local_list_x.remove(x)
                    self.local_list_y.remove(y)
            if self.pule_x > 0 > self.pule_y:  ## x +++ // y ---
                if x < self.car_point_x and y > self.car_point_y:
                    self.mode_state = 2
                    self.local_list_x.remove(x)
                    self.local_list_y.remove(y)
            if self.pule_x < 0 < self.pule_y:  ## x ---// y +++
                if x > self.car_point_x and y < self.car_point_y:
                    self.mode_state = 3
                    self.local_list_x.remove(x)
                    self.local_list_y.remove(y)
            if self.pule_x < 0 and self.pule_y < 0:  ## x ---// y ---
                if x > self.car_point_x and y > self.car_point_y:
                    self.mode_state = 4
                    self.local_list_x.remove(x)
                    self.local_list_y.remove(y)
    
    def math_radians(self, car_x, car_y, x, y):
        x = car_x - x
        y = car_y - y
        dist = math.hypot(x, y)
        return dist

    def get_csv(self):
        cfg = Config()
        df_TM = pd.read_csv(cfg.ORI_PATH)
        df_list = []

        df_list.append(df_TM['x'])
        df_list.append(df_TM['y'])

        x = self.car_point_x
        y = self.car_point_y

        list_cx = df_list[0]
        list_cy = df_list[1]

        self.local_list_x = []
        self.local_list_y = []

        for i in range(len(list_cx)):
            dist = self.math_radians(x, y, list_cx[i], list_cy[i])

            if dist < 20.0:
                self.local_list_x.append(list_cx[i])
                self.local_list_y.append(list_cy[i])

    def csv_global(self):
        cfg = Config()
        df_TM = pd.read_csv(cfg.USEPATH)
        df_list = []

        df_list.append(df_TM['x'])
        df_list.append(df_TM['y'])

        self.list_cx = df_list[0]
        self.list_cy = df_list[1]
        dist_list = []
        global_list_x = np.array(self.list_cx)
        global_list_y = np.array(self.list_cy)
        for i in range(len(global_list_x)):
            dist = self.math_radians(global_list_x[i], global_list_y[i], self.car_point_x, self.car_point_y)
            dist_list.append(dist)
        a = dist_list.index(np.min(dist_list))

        self.gx = global_list_x[a + 3]
        self.gy = global_list_y[a + 3]

        self.pule_x = global_list_x[a] - global_list_x[a - 1]
        self.pule_y = global_list_y[a] - global_list_y[a - 1]
        
        for i in range(9):
            self.guard_xy = [global_list_x[a + i], global_list_x[a + i]]
            self.guard_rail.append(self.guard_xy)

            # print("csv_global",self.guard_rail)
            
            
    def min_path(self, min_x, min_y):
        tngkr = []

        for i in range(len(self.round_x)):
            dx = min_x - self.round_x
            dy = min_y - self.round_y

            tngkr.append(np.hypot(dx, dy))

        a = np.argmin(tngkr)
        best_x = self.round_x[a]
        best_y = self.round_y[a]

        self.path_x.append(best_x)
        self.path_y.append(best_y)



    def lidar_point_op(self, data):
        if self.path_start == 1:
            if data.points[0].distance < 20:
                self.lidar_dis = data.points[0].distance
                self.lidar_op_x = data.points[0].x
                self.lidar_op_y = data.points[0].y
                
                self.xy = [self.lidar_op_x, self.lidar_op_y]
                if self.ob_count < 10:
                    if -5 < self.lidar_op_y <= -2:
                        self.ob_right_dect = self.ob_right_dect + 1
                        
                    if 2 <= self.lidar_op_y < 5:
                        self.ob_left_dect = self.ob_left_dect + 1
                    self.ob_lidar.append(self.xy)

                self.ob_count += 1
                self.objec = np.array(self.ob_lidar)
                if -1.5 <= self.lidar_op_y <= 1.5 and -4.0 <= self.lidar_op_x <= 10.0:
                    self.aroow_mode = 0
                else:
                    self.aroow_mode = 1
                '''
                if -5 < self.lidar_op_y <= -2:
                    self.ob_r_dect = 1 ## right dect
                else :
                    self.ob_r_dect = 0
                if 2 <= self.lidar_op_y < 5:
                    self.ob_l_dect = 1  ## left dect
                else :
                    self.ob_l_dect = 0
                '''
                if self.ob_right_dect > 0:
                    self.ob_r_dect = 1
                if self.ob_left_dect > 0:
                    self.ob_l_dect = 1

                self.path_start = 2
                    
    def lidar_real_point(self):
        print((self.guard_rail))
        if self.path_make == "left" : # path make left
            if self.ob_r_dect == 1 and self.ob_l_dect == 1:
                self.play_list = self.guard_rail
                print("1",self.play_list)
            else :
                self.play_list = [[3.0, 3.0], [5.0, 3.0], [7.0, 3.0], [9.0, 2.0], [11.0, 2.0], [13.0, 2.0], [15.0, 2.0]]
                print("21", self.play_list)
            
        elif self.path_make == "right" : # path make right
            self.play_list = [[3.0, -2.0], [5.0, -2.0], [7.0, -2.0], [9.0, -2.0], [11.0, -2.0], [13.0, -2.0],[15.0, -2.0]]
            print("3", self.play_list)
        elif self.aroow_mode == 1: # guqfh
            self.play_list = [[10.0, 0.0],[10.0, 0.0],[10.0, 0.0],[10.0, 0.0], [10.0, 0.0]]
            print("4", self.play_list)
        elif self.path_make == "no" :
            #if self.ob_r_dect == 1 and self.ob_l_dect == 1:   
            if self.ob_r_dect == 1 :## right dect
                self.play_list = [[3.0, 3.0], [5.0, 3.0], [7.0, 3.0], [9.0, 2.0], [11.0, 2.0], [13.0, 2.0], [15.0, 2.0]]
                print("6", self.play_list)
            else :## left dect
                self.play_list = [[3.0, -1.5], [5.0, -1.5], [7.0, -1.5], [9.0, -1.5], [11.0, -1.5], [13.0, -1.5]]
                print("3434", self.play_list)
        else :
            self.play_list = self.guard_rail
            print("play_list", self.play_list)
        print("play_list_out", self.play_list)
        yaw = self.car_point_theta

        for x, y in self.play_list:
            b = np.array([x, y, 1])
            yaw_matrix = np.array([
                [math.cos(yaw), -math.sin(yaw), self.car_point_x],
                [math.sin(yaw), math.cos(yaw), self.car_point_y],
                [0, 0, 1]
            ])
            result = np.dot(yaw_matrix, b.T)

            real_op_x = result[0]
            real_op_y = result[1]
            a = [real_op_x, real_op_y]

            self.lidar_path.append(a)



    def udp_sub(self, car_op):
        self.car_vel = float(car_op.vx * 3.6)
        self.car_yaw = float(car_op.yaw)

    def locali_sub(self, car_pos):
        self.car_point_x = car_pos.x
        self.car_point_y = car_pos.y
        self.car_point_theta = car_pos.theta

    def emer_state_sub(self, state):
        self.em_state = state.state
        if self.em_state == 3:
            self.path_start = 1
           
        
        if self.em_state < 2 and self.path_start == 2 :
            self.path_start = 0
            self.lidar_path = []
            self.send_count = 1
            self.pub_msg()


    def local_end_state(self, data):
        self.local_end = data.state
        if self.local_end == 1:
            self.path_start = 0
            self.lidar_path = []
            self.send_count = 1
            self.pub_msg()

def main():
    rospy.init_node("local_path_planning")
    local = local_path()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        local.local_path_planning()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
