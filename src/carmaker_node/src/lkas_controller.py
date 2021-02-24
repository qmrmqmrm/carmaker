#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math


def lkas(lane_center_pix):
	top_pix, low_pix, mid_pix = lane_center_pix

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

	# print("pix_alpha : ", pix_alpha)
	# print("pix_alpha_d : ", pix_alpha_d)
	steering_re = None
	if -1.5 <= pix_alpha_d <= 1.5:
		# pix_theta = math.pi/2 - t_l_alpha
		if -0.05 <= t_l_dx <= 0.05:
			SteeringWheel = 0
		else:
			SteeringWheel = t_l_dx / -1000
		steering_re = SteeringWheel
	else:
		pass

	return steering_re
