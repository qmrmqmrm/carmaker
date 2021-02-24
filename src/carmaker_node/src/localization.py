#!/usr/bin/env python
import rospy
import math
import pyproj
import numpy as np
from matplotlib import pyplot as plt
import message_filters
from sensor_msgs.msg import *
from carmaker_node.msg import *
from nav_msgs.msg import *


class Localization():
	def __init__(self):
		self.val_init()

	def subscribe(self):
		self.localization_pub = rospy.Publisher('/localization', localization, queue_size=10)

		self.gps_data = message_filters.Subscriber('/gps_out', GPS_Out)
		self.imu_data = message_filters.Subscriber('/imu_udp', imu_UDP)
		# self.odom_data = message_filters.Subscriber('/odom_msg', Odometry)
		self.udp_data = message_filters.Subscriber('/udp', UDP)
		ts = message_filters.ApproximateTimeSynchronizer([self.gps_data, self.imu_data, self.udp_data], 2, 0.1,
														 allow_headerless=True)
		ts.registerCallback(self.callback)

	def callback(self, gps, imu, udp):
		print("[start ] Localization process")
		self.val_init()  # value init
		self.gps_set(gps)  # gps data loading
		self.val_set(imu, udp, gps)  # value set
		self.KF()  # start estimate pose using KF
		self.pub_msg()
		print("-------------------------------")

	def val_init(self):
		print("[start ] value init")
		self.Theta = 0.0
		self.v = []
		self.w = []
		self.T = 0.01  # 100000
		self.result_x = []
		self.result_y = []
		self.result_theta = []
		self.gps_tm_x = []
		self.gps_tm_y = []
		self.start_pose_flag = False

	def gps_set(self, gps):
		LATLONG_WGS84 = pyproj.Proj("+proj=latlong +datum=WGS84 +ellps=WGS84")
		TM127 = pyproj.Proj("+proj=tmerc +lat_0=38N +lon_0=127E +ellps=bessel +x_0=200000 +y_0=600000 +k=1.0 ")
		longt = gps.longitude
		latti = gps.latitude
		tm_x, tm_y = pyproj.transform(LATLONG_WGS84, TM127, longt, latti)
		self.gps_tm_x.append(tm_x)
		self.gps_tm_y.append(tm_y)

	def val_set(self, imu, udp, gps):
		print("[start ] value set")
		if self.start_pose_flag is False:
			# self.start_x = odom.pose.pose.position.x
			# self.start_y = odom.pose.pose.position.y
			self.start_x = self.gps_tm_x[0]
			self.start_y = self.gps_tm_y[0]
			# self.start_x = gps.longitude
			# self.start_y = gps.latitude
			self.Theta = udp.yaw  # /(180*math.pi)		# deg -> rad
		# self.start_pose_flag = True
		self.w = imu.Omega_B_z
		self.v = imu.Vel_B_x

	def calc_odometry(self, x, y, theta, v, w):
		x = x - (v / w) * math.sin(theta) + (v / w) * math.sin(theta + w * self.T)
		y = y + (v / w) * math.cos(theta) - (v / w) * math.cos(theta + w * self.T)
		theta = theta + w * self.T
		v = v
		w = w
		return x, y, theta

	def KF(self):
		print("[start ] estimate pose using KF")
		if self.start_pose_flag is False:
			x_old = self.start_x
			y_old = self.start_y
			theta_old = self.Theta
			self.start_pose_flag = True


			x_new, y_new, theta_new = self.calc_odometry(x_old, y_old, theta_old, self.v, self.w)

			self.result_x = x_new
			self.result_y = y_new
			self.result_theta = theta_new
			x_old = x_new
			y_old = y_new
			theta_old = theta_new
			print(x_old, y_old)

	def pub_msg(self):
		pubMsg = localization()
		pubMsg.x = self.result_x
		pubMsg.y = self.result_y
		pubMsg.theta = self.Theta
		self.localization_pub.publish(pubMsg)


def main():
	rospy.init_node('Localization', anonymous=True)
	L = Localization()
	L.subscribe()
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
