#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu



class test_sub():
	def __init__(self):

		self.tw_sub = rospy.Subscriber('/imu', Imu, self.callback)

	def callback(self,d):
		print("-----------------------------------------------")
		print(d)
		
		
def main():
	rospy.init_node('get_imu_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
