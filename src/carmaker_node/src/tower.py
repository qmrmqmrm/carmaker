#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from config import Config
from pid_controller import *
from pure_controller import *
from controller import Controller
import method as mt
import lkas_controller as lc
import acc_controller as ac
import left_and_right as lar
import sys, select, tty, termios
import tf
from marker_idx import Marker_Class

from carmaker_node.msg import sub_udp

import matplotlib.pyplot as plt


def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

	return key


def main():
	global settings
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('controller')
	controller = Controller()
	PID_controller = Pid_Controller()
	cfg = Config()
	udp = sub_udp()
	states = States()

	pose_dict = mt.get_csv()
	cx = pose_dict['x']
	cy = pose_dict['y']
	target_course = TargetCourse(cx, cy)

	udp.GearNo = 1
	udp.VC_SwitchOn = 1
	controller.loop = 0
	v_dt = 0
	distance_old = 0
	count = 0

	acc_mode = 0
	pure_mode = 0

	local_x = None
	local_y = None
	local_course = None
	local_ind = None
	local_now_ind = None
	local_state = 2
	local_stop = 0

	y_line_state = 9

	E_state = None
	distance_new = None

	tar_vel_ = 20
	Lf_k = 1.6
	listener = tf.TransformListener()
	mc = Marker_Class()

	# start -----------------------------------------------
	while 1:
		# cal dt
		if controller.car_lasted_time == 0:
			controller.car_lasted_time = controller.car_now_time
		controller.car_dt_time = controller.car_now_time - controller.car_lasted_time
		controller.car_lasted_time = controller.car_now_time
		key = getKey()

		try:
			(rear_tf, rot_r) = listener.lookupTransform('localization', 'Wheel_Rear', rospy.Time(0))

			(front_tf, rot_f) = listener.lookupTransform('localization', 'Wheel_Front', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		try:
			# use localization
			state = State(rear_tf, front_tf, x=controller.car_local_point_x, y=controller.car_local_point_y,
						  yaw=controller.car_local_point_theta, v=controller.cur_vel)

			last_index = len(cx) - 1
			target_ind, Ld, now_ind = target_course.search_target_index(state, Lf_k)

			# what matter here?
			mc.init_markers([target_ind, now_ind])
			cur_vel_ = controller.cur_vel
			cur_dt = controller.car_dt_time
			cur_acc_ = controller.car_acc

		except (AttributeError, IndexError) as set_err:
			print(set_err)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue
		print("-------------------------------------------------------------")
		print("tar_vel_set", tar_vel_)
		if controller.speed_distance != 0:
			if controller.speed_distance <= 30:
				if controller.speed_limit >= 45:
					tar_vel_ = controller.speed_limit - 10

				elif controller.speed_limit >= 35:
					tar_vel_ = controller.speed_limit - 5

				else:
					tar_vel_ = controller.speed_limit
		print("tar_vel_limit", tar_vel_)
		print("cul_vel", controller.cur_vel)
		print("speed_distance", controller.speed_distance)
		print("speed_limit", controller.speed_limit)

		try:
			E_state = controller.emergency_state_info
			distance_new = controller.Front_car_dis
			print("1 distance : ", distance_new)
			# controller.Front_car_dis = None
		except AttributeError as e_s_err:
			print("No E_state, No distance")
		print("distance_ in : ", distance_new)
		print("mode : ", cfg.mode)
		print("state : ", cfg.state)
		print("E_state : ", E_state)

		if controller.traffic_state.data == 0:
			udp.GearNo = 1
		print("traffic", controller.traffic_state)

		# --------------------------------------------------------------------------------------
		# In global path
		if last_index > target_ind:
			cfg.mode = "waypoint"
			udp.VC_SwitchOn = 1

			try:
				local_x = controller.lo_x
				local_y = controller.lo_y
				local_state = controller.lo_state

				if local_state is 1:
					local_stop = 1

				elif local_x is not None or local_state is 0:
					cfg.mode = "local"
					local_course = TargetCourse(controller.lo_x, controller.lo_y)
					local_last_index = len(local_x) - 1
					local_ind, local_Ld, local_now_ind = local_course.search_target_index(state, Lf_k)
					print(local_x)
					print(local_y)
					if (local_last_index - 2) <= local_ind:
						local_stop = 1
						cfg.mode = "waypoint"
						print("loca_next_waypoint", cfg.mode)

			except (AttributeError, ValueError) as local_err:
				cfg.mode = "waypoint"
				print("No local")

			# 신호등
			# if cfg.mode is "local" and local_x is not None and local_ind is not None:
			if cfg.mode is "local":
				direction, LI = lar.local_get_angle(local_now_ind, local_x, local_y)
				print("local traffic")
			else:
				direction, LI = lar.get_angle(now_ind)
			# print("LI : ", LI)
			# print("direction : ", direction)
			udp.Lights_Indicator = LI

			# 직선 스티어링
			steering_re = lc.lkas(controller.lane_center_pix)

			# mode pick --------------------------------------------------------------------------------------

			if steering_re is not None and cfg.mode is not "turn":
				print("############################################")
				cfg.mode = "line"

			if direction is not None:
				cfg.mode = "turn"

			print("local_state : ", local_state)
			if local_stop == 1:
				print("local_state : ", local_state)
				cfg.mode = "waypoint"
				local_x = None
				local_y = None
				local_course = None
				local_ind = None
				local_state = 2
				local_stop = 0

			#
			Ks = 0.3
			Lf_k = 0.3
			if cfg.mode == "waypoint":
				acc_mode = 1
				pure_mode = 0

			if cfg.mode == "line":
				acc_mode = 1
				pure_mode = 1

			if cfg.mode == "turn":
				acc_mode = 0
				pure_mode = 0
				Ks = 0.6
				Lf_k = 0.5

			if cfg.mode == "local":
				acc_mode = 0
				pure_mode = 2
				Ks = 0.6
				Lf_k = 1
				udp.VC_SwitchOn = 1
				udp.GearNo = 1

			# mode --------------------------------------------------------------------------------------
			# acc mode
			target_velocity = tar_vel_
			if cfg.mode == "local":
				target_velocity = 5
			if cfg.mode == "turn":
				target_velocity = 15

			mov_vel = PID_controller.accel_control(target_velocity, cur_vel_, cur_dt, 1, 0, 1)
			print(distance_new)
			if acc_mode == 1:
				if distance_new is not None and E_state != 0:
					distance_old, cfg, Lights_Hazard, GearNo = ac.accs(distance_new, E_state, tar_vel_, cur_vel_,
																	   cur_dt, cfg, distance_old)
					udp.Lights_Hazard = Lights_Hazard
					# if GearNo == -9:
					# 	cfg.mode = "stop"
					# elif GearNo == 1:
					# 	udp.GearNo = 1
					# 	udp.VC_SwitchOn = 1
					print("front car velocity : ", cfg.acc_tar)

					# if cfg.state == 4:
					# 	if target_velocity < 10:
					# 		target_velocity = target_velocity - 5
					# 	else:
					# 		target_velocity = target_velocity - 9

					# if cfg.state == 3:
					# 	if target_velocity > 10:
					# 		target_velocity = 10
					# 	else:
					# 		target_velocity = 5

					# if cfg.state < 3:
					# 	# pd controller
					
					target_velocity = cfg.acc_tar

					mov_vel = PID_controller.accel_control(target_velocity, cur_vel_, cur_dt, 1, 0, 0)

				else:
					acc_mode = 0
			udp.Ax = mov_vel
			#

			# pure mode
			delta_gain = pure_pursuit_steer_control(state, target_course, target_ind, Ks)
			steering_value = delta_gain
			if pure_mode == 1:
				steering_value = steering_re
				if steering_re is None:
					steering_value = 0
			if pure_mode == 2:
				local_delta_gain = pure_pursuit_steer_control(state, local_course, local_ind, Ks)
				steering_value = local_delta_gain
			#

			# print("controller.yellow_lane", controller.yellow_lane)
			# print("controller.yellow_lane_dist", controller.yellow_lane_dist)

			# if controller.yellow_lane == "left":
			# 	if controller.yellow_lane_dist == "far":
			# 		print("left-far")
			# 		# 왼쪽으로 가야로함 / 차가 오른쪽으로 기울어져 있음
			# 		y_line_state = 1
			# 	elif controller.yellow_lane_dist == "close":
			# 		print("left-close")
			# 		# 오른쪽으로 가야함 / 차가 왼쪽으로 기울어져 있음
			# 		y_line_state = -1

			# elif controller.yellow_lane == "right":
			# 	if controller.yellow_lane_dist == "far":
			# 		print("right-far")
			# 		# 오른쪽으로 가야함 / 차가 왼쪽으로 기울어져 있음
			# 		y_line_state = -1
			# 	elif controller.yellow_lane_dist == "close":
			# 		print("right-close")
			# 		# 왼쪽으로 가야로함 / 차가 오른쪽으로 기울어져 있음
			# 		y_line_state = 1

			# elif controller.yellow_lane == "no" and controller.yellow_lane_dist == "no":
			# 	y_line_state = 9

			# if controller.yellow_lane_dist == "ok":
			# 	y_line_state = 0

			# print("before Yellow", steering_value)

			# if -90 <= controller.yellow_deg < -50:
			# 	# "right"
			# 	steering_value = controller.yellow_deg * (np.pi / 180)
			# 	print("turn right", steering_value)

			# elif 50 <= controller.yellow_deg < 90:
			# 	# "left"
			# 	steering_value = controller.yellow_deg * (np.pi / 180)
			# 	print("turn left", steering_value)

			# else:
			# 	steering_value = steering_value

			print("last steering_value : ", steering_value)
			print("udp GearNo  : ", udp.GearNo)
			print("udp mode  : ", cfg.mode)

			udp.SteeringWheel = steering_value

			# ----------------------------------------------------------------------------------
			controller.loop += 1

			# draw plot
			v_dt += cur_dt
			states.append(v_dt, state)

		else:
			print("lastIndex < target_ind")
			print("*-*-*-*-*-*-END-*-*-*-*-*-*")
		# ----------------------------------------------------

		# traffic light | red, yellow : stop, green : go
		if E_state == 0 and controller.cur_vel < 0:
			cfg.mode = "waypoint"
			distance_new = 100
			tar_vel_ = 20

		if controller.traffic_state.data == 1:
			cfg.mode = "stop"
		if distance_new is not None and cfg.mode is not "local":
			if distance_new <= 5:
				print(cfg.mode)
				cfg.mode = "stop"
			if E_state == 0 and distance_new is not None:
				cfg.mode = "waypoint"
				distance_new = None
		if cfg.mode == "stop":
			udp.GearNo = -9

		# ----------------------------------------------------

		# Brake
		if key == 'x':
			udp.VC_SwitchOn = 1
			udp.GearNo = -9
			udp.Ax = 0
			udp.SteeringWheel = 0
			print('stop - 1')

		else:
			if (key == '\x03'):
				break

		controller.car_pub.publish(udp)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass