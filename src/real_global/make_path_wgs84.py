import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate
import pandas as pd
import glob
import os
import sys
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
center = pd.read_csv('/home/j/catkin_ws/src/cut_origin_wgs84.csv')
center_list = []
interpolate_list = []
this_pose = None
to_center = None
waypoint = []
new_waypoint = []
gps_data = []
####get waypoint####
def filter_data():
	load_point = glob.glob(os.path.join('./','*.csv'))
	print(load_point[0])
#	gps_file = pd.read_csv('./GPS_Template1.csv')
	gps_file = pd.read_csv(load_point[0])
	print(gps_file)
	Lat = list(gps_file['Lat(rad)'].values)
	Long = list(gps_file['Long(rad)'].values)
	
	for la,lo in zip(Lat,Long):
		gps_data.append([la,lo])
	return gps_data
	
####rad to degree####
def to_degree(x,y):
	x2 = x*180/PI
	y2 = y*180/PI
	return x2,y2

####interpolate####
def interpolate_b_spline_path(x: list, y: list, n_path_points: int,
                              degree: int = 3) -> tuple:

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

def delete_same_point(data):
	before_data = pd.DataFrame(data)
	delete_data = before_data.drop_duplicates(subset=None,keep='first',inplace=False)
	delete_data_list = delete_data.values.tolist()
	return delete_data_list
	
def angle_between(p1, p2):  
	ang1 = np.arctan2(*p1[::-1])
	ang2 = np.arctan2(*p2[::-1])
	res = np.rad2deg((ang1 - ang2) % (2 * np.pi))
	return res
	
def getangle(global_path):
	for i in range(0,500):
		if i<len(global_path)-4:
			pt1 = (global_path[i+1][0] - global_path[i][0], global_path[i+1][1] - global_path[i][1])
			pt2 = (global_path[i+2][0] - global_path[i][0], global_path[i+2][1] - global_path[i][1])
			pt1b = (global_path[i+2][0] - global_path[i][0], global_path[i+2][1] - global_path[i][1])
			pt2b = (global_path[i+3][0] - global_path[i][0], global_path[i+3][1] - global_path[i][1])
			
			res = angle_between(pt1, pt2)
			resb = angle_between(pt1b, pt2b)
			res = (res + 360) % 360
			resb = (resb + 360) % 360

			if 2<res<358 and (0<=resb<=2 or 358<=resb<=360):		
				global_path.remove(global_path[i+1])

	return global_path
	
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
	delete_same_path = delete_same_point(waypoint)
	link_dataframe = pd.DataFrame(getangle(delete_same_path))
	link_dataframe.to_csv("/home/j/catkin_ws/src/carmaker_node/src/path/global_path_TM.csv")

if __name__ == '__main__':
	main()
