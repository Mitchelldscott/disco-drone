#!/usr/bin/env python3

import rospy

# because of transformations
import tf
import numpy as np

import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

def eulers_from_quaternion(q):
	"""
	Convert a quaternion into euler angles (roll, pitch, yaw)
	"""
	q0, q1, q2, q3 = q
	
	roll = np.arctan(2 * ((q0*q1) + (q2*q3)) / (1 - (2 * (q1**2 + q2**2))))
	pitch = np.arcsin(2 * ((q0*q2) - (q3*q1)))
	yaw = np.arctan(2 * ((q0*q3) + (q1*q2)) / (1 - (2 * (q2**2 + q3**2))))

	return np.array([roll, pitch, yaw]) 

class Kwad_Controller():
	def __init__(self):

		# self.tfBuffer = tf2_ros.Buffer()
		# self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.teensy_publisher = rospy.Publisher('/serial_out', String, queue_size=1)

		self.tf_sub = rospy.Subscriber("/tf_static", TFMessage, callback=self._onNewTransformSet, queue_size=10)
		self.transform = None

		rate = 50
		self.rate = rospy.Rate(rate)

	def _onNewTransformSet(self, msg):
		for transform in msg.transforms:
			if transform.child_frame_id == 'base_link1' and transform.header.frame_id == 'map':
				self.transform = transform

	def spin(self):
		while not rospy.is_shutdown():
			try:
				
				if self.transform is None:
					self.rate.sleep()
					continue
				
				rpy = eulers_from_quaternion([self.transform.transform.rotation.w,
										self.transform.transform.rotation.x, 
										self.transform.transform.rotation.y, 
										self.transform.transform.rotation.z])

				self.control = np.array([[0.0, -1.0, -0.1],
									[-1.0, 0.0, 0.1],
									[0.0, 1.0, -0.1],
									[1.0, 0.0, 0.1]]) @ rpy.T

				# print(rpy,self.control)
				msg = String()
				control = self.control.T
				msg.data = f'UU,{control[0]},{control[1]},{control[2]},{control[3]}'
				self.teensy_publisher.publish(msg)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				continue

			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('Kwad_Controller')
	controller = Kwad_Controller()
	controller.spin()
	
