#!/usr/bin/env python3

import rospy

# because of transformations
import tf
import numpy as np

import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import ByteMultiArray, Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion


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

		self.dbg_publisher = rospy.Publisher('/teensy_command', Float64MultiArray, queue_size=1)
		# self.teensy_publisher = rospy.Publisher('/serial_out', ByteMultiArray, queue_size=1)

		self.tf_sub = rospy.Subscriber("/tf_static", TFMessage, callback=self.tf_callback, queue_size=1)
		self.transform = None

		rate = 100
		self.rate = rospy.Rate(rate)

	def tf_callback(self, msg):
		for transform in msg.transforms:
			if transform.child_frame_id == 'base_link1' and transform.header.frame_id == 'map':
				self.transform = transform

	def spin(self):
		scale = 1
		ctr = 0

		while not rospy.is_shutdown():
			try:
				
				# if self.transform is None:
				# 	self.rate.sleep()
				# 	continue
				
				# rpy = np.array(euler_from_quaternion([self.transform.transform.rotation.w,
				# 														self.transform.transform.rotation.x, 
				# 														self.transform.transform.rotation.y, 
				# 														self.transform.transform.rotation.z]))

				# self.control = np.array([[0.0, 0.01, -0.01],
				# 					[-0.01, 0.0, 0.01],
				# 					[0.0, 0.01, -0.01],
				# 					[-0.01, 0.0, 0.01]]) @ rpy.T

				self.control = np.ones((4,)) * scale

				ctr += 1
				if ctr == 400:
					scale = 0.01
				# if ctr == 100:
				# 	scale += 256
				# 	scale %= 4096
				# 	ctr = 0

				control = []
				for c in self.control:
					control.append(np.min([1.0,np.max([0.0,c])]))

				msg = Float64MultiArray()
				msg.data = control
				self.dbg_publisher.publish(msg)

				# msg = ByteMultiArray()
				# data = [85, 85, 58]
				# print(rpy, control)
				# for u in control.tobytes():
				# 	print(u)
				# 	# if u > 127:
				# 	# 	data.append(127-u)

				# 	# else:
				# 	data.append(u-128)

				# msg.data = data
				# self.teensy_publisher.publish(msg)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				continue

			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('Kwad_Controller')
	controller = Kwad_Controller()
	controller.spin()
	
