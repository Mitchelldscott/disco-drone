#!/usr/bin/env python3

import rospy

# because of transformations
import tf
import quaternion
import numpy as np

import tf2_ros
import geometry_msgs.msg

from scipy.spatial.transform import Rotation as R

class Kwad_Controller():
	def __init__(self):

		self.BN = R.from_quat([0, 0, 0, 1])

		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)

		rate = 100
		self.rate = rospy.Rate(rate)

	def spin(self):
		while not rospy.is_shutdown():
			try:
				trans = tfBuffer.lookup_transform('map', 'baselink1', rospy.Time())
				self.BN = R.from_quat([trans.transform.rotation.x, 
										trans.transform.rotation.y, 
										trans.transform.rotation.z, 
										trans.transform.rotation.w])

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				continue

			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('Kwad_Controller')
	controller = Kwad_controller()
	controller.spin()
	
