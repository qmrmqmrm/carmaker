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
import numpy as np
import tf

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

	udp.GearNo = -9
	udp.VC_SwitchOn = 1
	controller.loop = 0
	v_dt = 0
	distance_old = 0
	count = 0

	y_line_state = 9

	tar_vel_ = 0
	Lf_k = 1.6
	listener = tf.TransformListener()

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
			print(rear_tf)

			(front_tf, rot_f) = listener.lookupTransform('localization', 'Wheel_Front', rospy.Time(0))
			print(front_tf)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		try:
			# use localization
			state = State(rear_tf, front_tf, x=controller.car_local_point_x, y=controller.car_local_point_y,
						  yaw=controller.car_local_point_theta, v=controller.cur_vel)

			print(state.WB)
			print(3.49-0.79)

			last_index = len(cx) - 1
			target_ind, Ld, now_ind = target_course.search_target_index(state, Lf_k)

			cur_vel_ = controller.cur_vel
			cur_dt = controller.car_dt_time

		except (AttributeError, IndexError) as set_err:
			print(set_err)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue

		if last_index > target_ind:

			Ks = 0.4

			mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt, 1, 0, 1)
			udp.Ax = mov_vel

			nx = target_course.cx[now_ind]
			zx = target_course.cx[0]
			ny = target_course.cy[now_ind]
			zy = target_course.cy[0]

			# pure mode
			# delta_gain, tx, ty = pure_pursuit_steer_control(state, target_course, target_ind, Ks)
			delta_gain, tx, ty = pure_pursuit_steer_control(state, target_course, target_ind, Ks)
			udp.SteeringWheel = delta_gain
			"""
			print("------------------------------------------------------------------------------")
			print("yaw : ", (controller.cur_yaw) % (np.pi*2))
			print("car v : ", state.v)
			print("delta_gain : ", delta_gain)
			print("")

			print("tx - : ", tx, "ty - : ", ty)
			print("Ld _ origin  : ", Ld, "Ld _ cal  : ", Ld_m)
			print("alpha _ origin  : ", alpha, "alpha _ cal  : ", alpha_m)
			print("delta rad _ origin  : ", delta_rad, "delta rad _ cal  : ", delta_m_rad)
			print("delta _ origin  : ", delta, "delta _ cal  : ", delta_m)

			print("yaw _ origin  : ", controller.cur_yaw)
			print("yaw _ 360 ", (controller.cur_yaw) % (np.pi * 2))
			print("yaw _ - tar  : ", car_pr, "yaw _ - cal  : ", car_fr, "yaw - ", car_pr-car_fr)
			print("yaw _ - cal  : ", car_alpha_m)

			tx_tar = target_course.cx[target_ind]
			tx_now = target_course.cx[now_ind]
			ty_tar = target_course.cy[target_ind]
			ty_now = target_course.cy[now_ind]

			tx_tar_1 = target_course.cx[now_ind + 1]
			ty_tar_1 = target_course.cy[now_ind + 1]
			tx_tar_2 = target_course.cx[now_ind + 2]
			ty_tar_2 = target_course.cy[now_ind + 2]
			tx_tar_3 = target_course.cx[now_ind + 3]
			ty_tar_3 = target_course.cy[now_ind + 3]

			print("test mode", test_mode,"\n test ind 1", test_ind_1, "test in 2", test_ind_2)

			print(" tx_tar : ", tx_tar, "  ty_tar : ", ty_tar )
			print(" tx_tar_1 : ", tx_tar_1, "  ty_tar_1 : ", ty_tar_1 )
			print(" tx_tar_2 : ", tx_tar_2, "  ty_tar_2 : ", ty_tar_2 )
			print(" tx_now : ", tx_now, "  ty_now : ", ty_now )
			print(target_ind-now_ind,"",last_index)
			print("")

			print("localization1 x : ", controller.car_local_point_x, "localization1 y : ", controller.car_local_point_y)
			print("rear x : ", state.rear_x, "rear y : ", state.rear_y)
			print("front x : ", state.front_x, "front y : ", state.front_y)


			print("lo_1 dx : ", controller.car_local_point_x - tx, "lo_1 dy : ", controller.car_local_point_y - ty)

			dlo_1 = np.hypot(tx - controller.car_local_point_x, ty - controller.car_local_point_y)
			dlo_1_0 = np.hypot(tx_tar - controller.car_local_point_x, ty_tar - controller.car_local_point_y)
			dlo_1_1 = np.hypot(tx_tar_1 - controller.car_local_point_x, ty_tar_1 - controller.car_local_point_y)
			dlo_1_2 = np.hypot(tx_tar_2 - controller.car_local_point_x, ty_tar_2 - controller.car_local_point_y)
			dlo_1_3 = np.hypot(tx_tar_3 - controller.car_local_point_x, ty_tar_3 - controller.car_local_point_y)
			dlo_1_now = np.hypot(tx_now - controller.car_local_point_x, ty_now - controller.car_local_point_y)

			print("lo_1 d : ", dlo_1_now, dlo_1_0, dlo_1_1, dlo_1_2, dlo_1_3)
			print(" target ind : ", target_ind," | ",last_index)
			print(" now ind : ", now_ind," | ",last_index)
			print(" pure_ cal ind : ", pu_ind, "pu ind : ", pu_pind)


			print("")
			print("tx      : ", tx, "ty      : ", ty)
			print("nx      : ", nx, "ny      : ", ny)
			print("state x : ", state.x, "state y : ", state.y)
			print("distance target x : ", tx - state.x, "distance target y : ", ty - state.y)
			dtar = np.hypot(state.x - tx, state.y - ty)
			dnow = np.hypot(state.x - nx, state.y - ny)
			print("")
			print("distance target : ", dtar, "distance now : ", dnow)
			print("Ld _ origin  : ", Ld, "Ld _ cal  : ", Ld_m)
			"""
			# ----------------------------------------------------------------------------------
			controller.loop += 1

			# draw plot
			v_dt += cur_dt
			states.append(v_dt, state)

			# -------------------------------------------------------------------------------------
			# drawing plot
			if cfg.show_animation:  # pragma: no cover
				plt.cla()
				# for stopping simulation with the esc key.

				plt.gcf().canvas.mpl_connect(
					'key_release_event',
					lambda event: [exit(0) if event.key == 'escape' else None])
				plot_arrow(state.x, state.y, state.yaw)
				plt.plot(cx, cy, "-r", label="course")
				plt.plot(states.x, states.y, "-b", label="trajectory")
				plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
				plt.axis("equal")
				plt.grid(True)
				plt.title("Speed[km/h]:" + str(state.v)[:4])
				plt.pause(0.001)

			if cfg.show_animation_gain:
				plt.cla()
				plt.gcf().canvas.mpl_connect(
					'key_release_event',
					lambda event: [exit(0) if event.key == 'escape' else None])
				plt.grid(True)
				plt.plot(states.t, [iv for iv in states.v], "-r")
				plt.legend()
				plt.xlabel("Time[s]")
				plt.ylabel("Speed[km/h]")
				plt.axis("equal")
				plt.title("Speed[km/h]:" + str(state.v)[:4])
				plt.grid(True)
				plt.pause(0.00001)
			#

		else:
			print("lastIndex < target_ind")
			print("*-*-*-*-*-*-END-*-*-*-*-*-*")
		# ----------------------------------------------------


		# set target V
		if key == '0':
			udp.VC_SwitchOn = 0
			print('maneuver')

		if key == '1':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 10

		if key == '2':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 20

		if key == '3':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 30

		if key == '4':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 40

		if key == '5':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 50

		if key == '9':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 1
		if key == '8':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 3
		if key == '7':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			tar_vel_ = 5

		if key == 'w':
			udp.VC_SwitchOn = 1
			if udp.Ax >= 0:
				udp.GearNo = 1
			else:
				udp.GearNo = -1
			udp.Ax += 1

		if key == 'a':
			udp.VC_SwitchOn = 1
			udp.SteeringWheel += 0.1
			# mode_pub.publish(udp)
			print('SteeringWheel +')

		if key == 'd':
			udp.VC_SwitchOn = 1
			udp.SteeringWheel -= 0.1
			# mode_pub.publish(udp)
			print('SteeringWheel -')

		if key == 'e':
			udp.VC_SwitchOn = 1
			udp.SteeringWheel = 0.0
			# mode_pub.publish(udp)
			print('SteeringWheel 0')

		if key == 's':
			udp.VC_SwitchOn = 1
			if udp.Ax >= 0:
				udp.GearNo = 1
			else:
				udp.GearNo = -1
			udp.Ax -= 1
			# mode_pub.publish(udp)
			print('Ax -')



		# Brake
		if key == 'c':
			udp.VC_SwitchOn = 1
			udp.GearNo = 0
			udp.Ax = 0
			udp.SteeringWheel = 0
			print('stop - 0.5')

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

	# draw plot
	try:
		if controller.show_animation:  # pragma: no cover
			plt.cla()
			plt.plot(cx, cy, ".r", label="course")
			plt.plot(states.x, states.y, "-b", label="trajectory")
			plt.legend()
			plt.xlabel("x[m]")
			plt.ylabel("y[m]")
			plt.axis("equal")
			plt.grid(True)

			plt.subplots(1)
			plt.plot(states.t, [iv for iv in states.v], "-r")
			plt.xlabel("Time[s]")
			plt.ylabel("Speed[km/h]")
			plt.grid(True)
			plt.show()

	except AttributeError:
		print("*-*-*-n-o-d-a-t-a-*-*-*")


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
