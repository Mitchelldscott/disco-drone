#! /usr/env python3

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray

mag_data = None

def magCallback(msg):
	


def main():
	mag_sub = rospy.subscriber('/mag', Float64MultiArray, magCallback)

if __name__ =='__main__':
	main()