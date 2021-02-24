import pyproj
import numpy as np
import csv
import pandas as pd

def read_pandas():
	gps_list = []
	data = pd.read_csv("/home/j/catkin_ws/src/carmaker_node/src/path/global_path_TM.csv")
	for longitude, latitude in zip(data["0"],data["1"]):
		gps = [longitude, latitude]
		gps_list.append(gps)
	return gps_list

	
def WGS84_to_TM():
	trans = read_pandas()
	TM_list = []
	for t in trans:
		wgs84 = [t[1],t[0]]
		LATLONG_WGS84 = pyproj.Proj("+proj=latlong +datum=WGS84 +ellps=WGS84")
		TM127 = pyproj.Proj("+proj=tmerc +lat_0=38N +lon_0=127E +ellps=bessel +x_0=200000 +y_0=600000 +k=1.0 ")
		longt = (float)(t[0])
		lat = (float)(t[1])
		x,y = pyproj.transform(LATLONG_WGS84, TM127, longt, lat)
		TM = [x,y]
		TM_list.append(TM)
	print(len(TM_list))
	return TM_list
		
def save_csv(gps_TM):
	data = gps_TM
	dataframe = pd.DataFrame(data)
	dataframe.to_csv("/home/j/catkin_ws/src/carmaker_node/src/path/global_path_TM.csv",header = ["x","y"],index = False)
	
if __name__ == '__main__':
	gps_TM = WGS84_to_TM()
	save_csv(gps_TM)

