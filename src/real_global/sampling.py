import numpy as np
import pandas as pd


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

data = pd.read_csv("/home/j/catkin_ws/src/carmaker_node/src/path/global_path_TM.csv")
sampling = data.sample(frac = 0.9,random_state = 1)
print(sampling)

ds = sampling.sort_index()
ds = ds.drop([ds.columns[0]],axis=1)
ds = ds.values.tolist()
#ds.to_csv("./sampled_70per.csv",index = False)

#ds = sampling.sort_values(by = ['0','1'],axis=0)
print(ds)
dsp = getangle(ds)
print(dsp)
#link_dataframe = pd.DataFrame(getangle(delete_same_path))
dsp = pd.DataFrame(dsp)
dsp.to_csv("/home/j/catkin_ws/src/carmaker_node/src/path/global_path_TM.csv",index=False)
#delete_data = data.drop_duplicates(subset=['0','1'],keep='first',inplace=False)
#sampling.to_csv("./scenario1_300_deleted.csv")

