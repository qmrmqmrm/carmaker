#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from config import Config
from pid_controller import *
from pure_controller import *
from controller import Controller
import method as mt

from carmaker_node.msg import sub_udp

import sys, select, tty, termios
import math
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

	udp.GearNo = 1

	tar_vel_ = 0
	controller.loop = 0

	pose_dict = mt.get_csv()
	cx = pose_dict['x']
	cy = pose_dict['y']

	target_course = TargetCourse(cx, cy)
	states = States()

	v_dt = 0
	distance_old = 0
	count = 0

	# start -----------------------------------------------
	while 1:
		key = getKey()
		# cal dt
		if controller.car_lasted_time == 0:
			controller.car_lasted_time = controller.car_now_time
		controller.car_dt_time = controller.car_now_time - controller.car_lasted_time
		controller.car_lasted_time = controller.car_now_time

		# set target target V
		if key == 't':
			udp.VC_SwitchOn = 1
			udp.GearNo = 1
			try:
				tar_vel_ = float(input("target velue : "))
				print("set", tar_vel_, "km/h")
			except NameError:
				print("try again")
			except SyntaxError:
				print("try again")

		try:
			# use localization
			state = State(x=controller.car_local_point_x, y=controller.car_local_point_y, yaw=controller.car_local_point_theta, v=controller.cur_vel)
		except (AttributeError, IndexError) as e:
			print(e)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue

		try:

			last_index = len(cx) - 1
			target_ind, Lf, Ld = target_course.search_target_index(state)
		except (AttributeError, IndexError) as e:
			print(e)
			print("*-*-*-L-O-D-I-N-G-*-*-*")
			continue
		# print(target_ind)

		# # Loop in index, untill last index
		# ## 만약 Wapoint
		# if cfg.mode == "waypoint":
		#




		## 만약 lane_tracking



		## 만약 trun



		## 만약 local




		if last_index > target_ind:
			cur_vel_ = controller.cur_vel
			cur_dt = controller.car_dt_time
			cur_acc_ = controller.car_acc

			if controller.traffic_stat == 1:
				udp.GearNo = -9
			else:
				udp.GearNo = 1

			try:
				distance_new = controller.Front_car_dis
				E_state = controller.emergency_state_info
				if cfg.state == 0:
					cfg.acc_v = tar_vel_
					mov_vel = PID_controller.accel_control(cfg.acc_v, cur_vel_, cur_dt)

				if distance_new <= cfg.safe_distance / 2:
					cfg.state = 5
					# udp.GearNo = -9
					print("Danger! Stop!")
					udp.Lights_Hazard = 1

				elif cfg.safe_distance / 2 < distance_new < cfg.safe_distance:
					cfg.state = 4
					print("Deceleration")
					udp.Lights_Hazard = 0

				elif cfg.safe_distance <= distance_new <= cfg.safe_distance + 5:
					cfg.state = 3
					print("Safe distance")
					udp.Lights_Hazard = 0

				elif cfg.safe_distance + 5 < distance_new < cfg.safe_distance + 15:
					cfg.state = 2
					print("Acceleration")
					udp.Lights_Hazard = 0

				# else:
				# 	cfg.state = 1
				# 	cfg.acc_v = tar_vel_
				# 	print("Nothing front")
				# cfg.acc_v = math.sqrt((cur_vel_ ** 2) + 2 * cur_acc_ * (cfg.safe_distance - distance))

				try:
					cfg.acc_v = cur_vel_ + (distance_new - distance_old) / cur_dt
				except ZeroDivisionError as z:
					print(z)

				if controller.emergency_state_info == 0:
					cfg.state = 1
					cfg.acc_v = tar_vel_
					print("Nothing front")

				# pd controller
				mov_vel = PID_controller.accel_control(cfg.acc_v, cur_vel_, cur_dt, "p")
				udp.Ax = mov_vel  # m/s^2
				print("acc_vel : ", cfg.acc_v)

				distance_old = distance_new

			except AttributeError:
				# Nothing front
				# pd controller
				mov_vel = PID_controller.accel_control(tar_vel_, cur_vel_, cur_dt)
				udp.Ax = mov_vel  # m/s^2
				cfg.state = 0
				print("Nothing--------------------------")

			try:
				top_pix, low_pix, mid_pix = controller.lane_center_pix

				t_l_dx = top_pix - low_pix
				t_l_dy = 0 - 800
				t_l_alpha = math.atan2(t_l_dy, t_l_dx)

				m_l_dx = mid_pix - low_pix
				m_l_dy = 400 - 800
				m_l_alpha = math.atan2(m_l_dy, m_l_dx)

				# rad
				pix_alpha = t_l_alpha - m_l_alpha
				# deg
				pix_alpha_d = pix_alpha * (180 / math.pi)

				print("pix_alpha : ", pix_alpha)
				print("pix_alpha_d : ", pix_alpha_d)

				if -1.5 <= pix_alpha_d <= 1.5:
					print("############################################")
					# pix_theta = math.pi/2 - t_l_alpha
					count += 1
					if count >= 5:
						if -0.05 <= t_l_dx <= 0.05:
							udp.SteeringWheel = 0

						else:
							udp.SteeringWheel = t_l_dx / -1000

				else:
					# # steering wheel ang! = deg
					delta, target_ind, delta_rad, delta_gain, alpha, alpha_m, car_alpha_m, delta_m, Ld_m = pure_pursuit_steer_control(
						state, target_course, target_ind)
					udp.SteeringWheel = delta_gain
					count = 0

			except AttributeError as k:
				print(k)

			v_dt += cur_dt
			# state.update(mov_vel, delta_rad, cur_dt)
			states.append(v_dt, state)

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

			# loop check
			controller.loop += 1

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
