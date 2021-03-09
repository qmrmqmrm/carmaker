#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import os.path as op


class Config():
	def __init__(self):
		# self.DATAROOT = "/home/kang/catkin_ws/src/carmaker_node"
		self.SRCPATH = os.path.dirname(os.path.realpath(__file__))
		self.P_PATH = os.path.join(self.SRCPATH, "path")

		# select test file
		self.USEPATH = os.path.join(self.P_PATH, self.num_file(0))

		self.ORI_PATH = os.path.join(self.P_PATH, self.num_file(999))

		# option - default
		self.safe_distance = 7  	# 안전거리 기본값
		self.set_distance = 45		# 라이다 검출 시작 기본 거리
		self.state = 0				# 차량의 상태
		self.acc_v = 0				# 앞 차의 속도
		self.acc_t = 0				# 앞 차의 안전거리에 도달하는 시간
		self.acc_a = 0				# 그 안전거리에 도달 했을 시 앞차와 속도가 동일하게 만들 가속도
		self.acc_tar = 0 			# 그 가속도를 가지고 등가속도 운동을 할 때의 차의 속도 (타겟값)

		# mode
		self.mode = None

		# draw option
		# self.show_animation = True
		self.show_animation = False

		# self.show_animation_gain = True
		self.show_animation_gain = False

		self.show_print = True

	# self.show_print = False

	def num_file(self, num):
		if num == 0:
			file = "global_path_TM.csv"
		elif num == 1:
			file = "fxp_s1_TM.csv"
		elif num == 2:
			file = "fxp_s2_TM.csv"
		elif num == 3:
			file = "fxp_s3_TM.csv"
		elif num == 4:
			file = "fxp_s4_TM.csv"

		elif num == 999:
			file = "cut_origin_TM.csv"

		else:
			file = ""
		return file
