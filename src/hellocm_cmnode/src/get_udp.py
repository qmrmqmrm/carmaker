#!/usr/bin/env python
import rospy
from hellocm_msgs.msg import UDP



class test_sub():
	def __init__(self):

		self.tw_sub = rospy.Subscriber('/udp', UDP, self.callback)

	def callback(self,d):
		print("-----------------------------------------------")
		print(d)
		
		
def main():
	rospy.init_node('get_udp_sub')
	test = test_sub()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
