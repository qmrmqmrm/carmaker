#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from config import Config

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from carmaker_node.msg import *
from carmaker_tracking.msg import emergency_state, Front_vehicle_state


class Controller:
	def __init__(self):
		self.car_sub = rospy.Subscriber('/udp', UDP, self.udp_callback)
		self.car_pub = rospy.Publisher('/sub_udp', sub_udp, queue_size=10)
		self.car_pos = rospy.Subscriber('/odom', Odometry, self.pure_callback)
		self.car_local = rospy.Subscriber('/localization', localization, self.local_callback)

		self.car_lane = rospy.Subscriber('/lane_center_information', lane_center, self.lane_callback)
		self.car_e_stop = rospy.Subscriber('/Emergency_state', emergency_state, self.e_stop_callback)
		self.car_front = rospy.Subscriber('/Front_vehicle_information', Front_vehicle_state, self.Front_callback)

		self.traffic = rospy.Subscriber('/traffic_light', Int32, self.traffic_callback)
		self.local_path = rospy.Subscriber('/lo_point', lo_point, self.l_path_callback)

		self.yellow_lane = rospy.Subscriber('/yellow_lane', String, self.yl_callback)
		self.yellow_lane_dist = rospy.Subscriber('/yellow_lane_dist', String, self.yld_callback)
		self.stop_line = rospy.Subscriber('/stop_line', String, self.sl_callback)



		self.local_path = rospy.Subscriber('/speed_limit', Speed_Limit, self.speed_callback)

		self.loop = 0
		self.car_now_time = 0
		self.car_lasted_time = 0
		self.car_dt_time = 0

		# self.distance_ch = 0

	def udp_callback(self, data_udp):
		self.cur_vel = float(data_udp.vx * 3.6)  # m/s -> km/h
		self.cur_yaw = float(data_udp.yaw)
		self.car_now_time = data_udp.header.stamp.to_sec()
		self.car_acc = float(data_udp.ax)

	def pure_callback(self, data_pure):
		self.car_point_x = data_pure.pose.pose.position.x
		self.car_point_y = data_pure.pose.pose.position.y

	def local_callback(self, data_local):
		self.car_local_point_x = data_local.x
		self.car_local_point_y = data_local.y
		self.car_local_point_theta = data_local.theta

	def lane_callback(self, lane_state):
		self.lane_center_pix = lane_state.center_pix
		self.lane_state_offset = lane_state.offset_

	def e_stop_callback(self, e_state):
		self.emergency_state_info = e_state.state

	def Front_callback(self, Front_state):
		self.Front_car_x = Front_state.x
		self.Front_car_y = Front_state.y
		self.Front_car_dis = Front_state.distance
		self.Front_car_int = Front_state.intensity
		# self.distance_ch = 1

	def traffic_callback(self, traffic_stat):
		self.traffic_state = traffic_stat

	def l_path_callback(self, l_path):
		self.lo_x = l_path.x_point
		self.lo_y = l_path.y_point
		self.lo_state = l_path.state

	def speed_callback(self, speed_state):
		self.speed_distance = speed_state.distance
		self.speed_limit = speed_state.speed_limit

	def yl_callback(self, yl_state):
		self.yellow_lane = yl_state

	def yld_callback(self, yld_state):
		self.yellow_lane_dist = yld_state

	def sl_callback(self, sl_state):
		self.stop_line = sl_state
