#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate
import pandas as pd
from numpy.linalg import inv
############for rad to degree####################
PI = 3.141592
############for kalman filter####################
A = np.array([[ 1, 1,  0,  0],
              [ 0,  1,  0,  0],
              [ 0,  0,  1, 1],
              [ 0,  0,  0,  1]])
H = np.array([[ 1,  0,  0,  0],
              [ 0,  0,  1,  0]])
Q = 1.0 * np.eye(4)
R = np.array([[100,  0],
              [ 0, 100]])
############for find center point################
center = pd.read_csv('./cut_origin_wgs84.csv')
center_list = []
interpolate_list = []
this_pose = None
to_center = None
waypoint = []
new_waypoint = []

####get waypoint####
def filter_data():
	alld3 = [[0.6559432745,2.214497566],[0.65594697,2.214504957],[0.655945003,2.21451664],[0.6559343338,2.214534998],[0.6559246182,2.214551449],[0.6559170485,2.214564323],[0.6558943391,2.214602947],[0.6558870673,2.214615107],[0.6558845043,2.214619637]]
	alld2 = [[0.6559187174,2.214563608],[0.6559081078,2.214581728],[0.6558954716,2.214604139],[0.65589571,2.214615822],[0.6559137106,2.214633942],[0.6559199691,2.214640379],[0.6559277177,2.214648247]]
	alld1 = [[0.6558065414,2.214757442],[0.6558124423,2.214747429],[0.6558184624,2.214736938],[0.6558237076,2.214728117],[0.6558326483,2.214713097],[0.6558418274,2.214697599],[0.6558497548,2.21468401],[0.655859828,2.214666843],[0.655868113,2.214652777],[0.6558885574,2.214617968]]
	alld4 = [[0.6558749676,2.214668036],[0.6558699012,2.21466279],[0.6558681726,2.214654922],[0.6558733582,2.214644909],[0.6558892131,2.214617968]]
	return alld4
####rad to degree####
def to_degree(x,y):
	x2 = x*180/PI
	y2 = y*180/PI
	return x2,y2

####interpolate####
def interpolate_b_spline_path(x: list, y: list, n_path_points: int,degree: int = 3) -> tuple:

    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)

    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)
    
####kalman_filter####
def kalman_filter(z_meas, x_esti, P):
	x_pred = A @ x_esti
	P_pred = A @ P @ A.T + Q 
	K = P_pred @ H.T @ inv(H @ P_pred @ H.T +R)
	x_esti = x_pred + K @ (z_meas - H @ x_pred)
	P = P_pred - K @ H @ P_pred
	return x_esti, P

def kalman_gps(spline_list):
	x_0 = np.array([spline_list[0][0], 0, spline_list[0][1], 0])  # (x-pos, x-vel, y-pos, y-vel) by definition in book.
	P_0 = 100 * np.eye(4)
	n_samples = len(spline_list)
	xpos_meas_save = np.zeros(n_samples)
	ypos_meas_save = np.zeros(n_samples)
	xpos_esti_save = np.zeros(n_samples)
	ypos_esti_save = np.zeros(n_samples)

	x_esti, P = None, None
	for i in range(n_samples):
		pos = spline_list
		z_meas = pos[i]
		if i == 0:
			x_esti, P = x_0, P_0
		else:
			x_esti, P = kalman_filter(z_meas, x_esti, P)
		xpos_meas_save[i] = z_meas[0]
		ypos_meas_save[i] = z_meas[1]
		xpos_esti_save[i] = x_esti[0]
		ypos_esti_save[i] = x_esti[2]
	kalman_data = []
	for k_x,k_y in zip(xpos_esti_save,ypos_esti_save):
		kalman_data.append([k_x,k_y])
	return kalman_data	
	
####make [x,x2],[y,y2] to list[[x,y],[x2,y2]]
def get_list(xlist,ylist):
	get_list = []
	for x,y in zip(xlist,ylist):
		data = [x,y]
		get_list.append(data)
	return get_list

####main####
def main():
	fordata = filter_data()
	deg_list = []
	data_x = []
	data_y = []
	this_distance = 10
    ####rad to degree####
	for i in fordata:
		todeg_x,todeg_y = to_degree(i[1],i[0])
		todegree = [todeg_x,todeg_y]
		deg_list.append(todegree)
	for i in deg_list:
		data_x.append(i[0])
		data_y.append(i[1])
	####bspline interpolate####
	n_course_point = 500
	rix, riy = interpolate_b_spline_path(data_x, data_y,
                                         n_course_point)
	interpolate_list = get_list(rix,riy)
	final = pd.DataFrame(interpolate_list)
	#final.to_csv("./fix_interpolate_500_kalman.csv")
	after_kalman_list = kalman_gps(interpolate_list)
	####find close center point algorithm####
	for a,b in zip(center['latitude'],center['longitude']):
		center_list.append([a,b])
	before_center = [0,0]
	for this in after_kalman_list:
		this_pose = this
		for center_point in center_list:
			distance = np.hypot(this_pose[0]-center_point[0],this_pose[1]-center_point[1])
			if this_distance > distance:
				this_distance = distance
				to_center = center_point
		
		waypoint.append(to_center)
		this_distance = 10
	'''		
	print(waypoint[0][0])
	for i in waypoint:
		w = i-np.array([0.00001,-0.0001])
		new_waypoint.append(w)
	print(new_waypoint[0][0])
	'''
	link_dataframe = pd.DataFrame(waypoint)
	link_dataframe.to_csv("./scenario4_wgs84.csv")	

if __name__ == '__main__':
    main()
