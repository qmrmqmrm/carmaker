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


def accs(distance_new, E_state, tar_vel_, cur_vel_, cur_dt, cfg, distance_old):
	"""
	:param distance_new: 현재 앞차와의 거리
	:param E_state: 라이다의 상태 (0 : 앞에 없음, 1 : 15~25m에 장애물, 2 : 15m 안에 장애물, 3 : 2가 계속 지속될 경우)
	:param tar_vel_: 타겟 속도
	:param cur_vel_: v0 현재 속도
	:param cur_dt: t1 - t0 시간의 차이
	:param cfg: config에 있는 변수로 조절
	:param distance_old: t0일때의 앞차와의 거리

	Lights_Hazard : 비상등 off = 0, on = 1
	e_distance : 비상거리
	a_v : km/h -> m/s
	cfg.safe_distance : 안전거리
		기본 안전거리는 7m + 현재 속도/3.6 (m/s)
		0km로 주행 시 7 + 0 = 7m
		10km로 주행 시 7 + 2.8 = 9.8m
		20km로 주행 시 7 + 5.5 = 12.5m
		30km로 주행 시 7 + 8.3 = 15.3m
		40km로 주행 시 7 + 11.1 = 18.1m
	cfg.set_distance : 라이다가 검출 시작하는 거리
	cfg.state = 0				# 차량의 상태
	cfg.acc_v = 0				# v1 앞 차의 속도
	cfg.acc_t = 0				# t1 앞 차의 안전거리에 도달하는 시간
	cfg.acc_a = 0				# a 그 안전거리에 도달 했을 시 앞차와 속도가 동일하게 만들 가속도, 이때 등가속도 운동을 진행.
	t0 는 현재 시간이기 때문에 항상 0이 나와야 하여 제외.
	cfg.acc_tar = 0 			# 그 가속도를 가지고 등가속도 운동을 할 때의 차의 속도 (타겟값)

	:return:
	distance_old_ :
	cfg :
	Lights_Hazard :
	"""

	Lights_Hazard = 0					# 비상등 off = 0, on = 1
	e_distance = 7						# 비상거리
	a_v = cur_vel_ / 3.6				# km/h -> m/s
	cfg.safe_distance = 13 		# 안전거리
	GearNo = 1

	# 5m 안에 물체가 있을 경우 비상정지 == 안전거리를 침범
	if distance_new <= e_distance:
		cfg.state = 5
		GearNo = -9
		print("E_Stop!")
		Lights_Hazard = 1
	elif e_distance < distance_new < cfg.safe_distance + 5:
		cfg.state = 4
		print("Deceleration")
		GearNo = 1

	elif cfg.safe_distance + 5 <= distance_new <= cfg.safe_distance + 15:
		cfg.state = 3
		print("Safe distance")
		GearNo = 1
	elif cfg.safe_distance + 15 < distance_new < cfg.set_distance:		# set_distance : 50
		cfg.state = 2
		print("Acceleration")
		GearNo = 1
	else:
		cfg.state = 0
		GearNo = 1

	if cur_dt == 0:
		cur_dt = 0.1

	# v1, 앞차의 속도를 구함
	cfg.acc_v = cur_vel_ + (distance_new - distance_old) / cur_dt
	try:
		# t1, 안전거리까지 갈때의 t1을 구함.
		cfg.acc_t = ((distance_new - cfg.safe_distance) - distance_old)/(cfg.acc_v - cur_vel_)
		# s1, t1에 갈 때의 가속도를 구함, 이 때 현재 속도가 더 커서 -가속도가 나오면 감속을 수행해야 함.
		cfg.acc_a = (cfg.acc_v - cur_vel_)/cfg.acc_t
	except ZeroDivisionError as z:
		cfg.acc_a = 0

	cfg.acc_tar = cur_vel_ + cfg.acc_a * cur_dt

	#
	# if (cfg.acc_v - cur_vel_) == 0:
	# 	if cfg.safe_distance - 1 <= distance_new <= cfg.safe_distance + 1:
	# 		cfg.acc_a = cfg.acc_a
	# 	elif cfg.safe_distance + 1 < distance_new:
	# 		cfg.acc_a += 0.1
	# 	elif distance_new < cfg.safe_distance - 1:
	# 		cfg.acc_a -= 0.1

	# 해당 가속도로 등가속 운동을 할 때의 속도를 타겟값으로 잡아서 pid 제어를 진행
	# 여기 들어오는 건 속도가 0, 0일때 혹은 앞차와 속도가 동일할시에 진행되므로 0일때가 많다고 가정,
	# 현재속도가 0일때는 로컬패스 실행

	# state 1은 앞에 아무것도 없을 때...
	if E_state == 0:
		cfg.state = 1
		cfg.acc_v = tar_vel_
		print("Nothing front")

	# state = 0 처음 시작할 때 값
	if cfg.state == 0:
		cfg.acc_v = tar_vel_

	# 이전 값
	distance_old_ = distance_new

	return distance_old_, cfg, Lights_Hazard, GearNo
