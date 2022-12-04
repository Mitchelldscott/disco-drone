#!/usr/bin/env python3

import rospy

# because of transformations
import tf
import numpy as np
from std_msgs.msg import ByteMultiArray, Float64MultiArray


class Kwad_Controller():
	def __init__(self):

		# self.tfBuffer = tf2_ros.Buffer()
		# self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.update_timer = 0

		self.dbg_publisher = rospy.Publisher('/teensy_command', Float64MultiArray, queue_size=1)

		self.state_sub = rospy.Subscriber("/state", Float64MultiArray, callback=self.ea_callback, queue_size=1)
		self.state = np.zeros(6)

		self.euler_history = np.zeros((5,3))

		rate = 60
		self.dt = 1 / rate
		self.rate = rospy.Rate(rate)

	def ea_callback(self, msg):
		self.state = np.array(msg.data)
		self.update_timer = 0


	def spin(self):
		throttle = 0
		ctr = 0

		while not rospy.is_shutdown():
				
			# if self.transform is None:
			# 	self.rate.sleep()
			# 	continue

			if self.update_timer < 2:

				s_control = [0, 0, 0, 0]
				if ctr >  1 / self.dt:
					if abs(self.state[0]) < 0.15:
						s_control = np.array([[   0.0,   0.0,  0.0,   0.0,  -0.0,  -0.0],
											  [   0.0,   2e-1,   0.0,   0.0,   0.0,   0.0],
											  [   0.0,   0.0, -0.0,  -0.0,  -0.0,  -0.0],
											  [  -0.0,  -2e-1,   0.0,   0.0,   0.0,   0.0]]) @ self.state

					elif abs(self.state[0]) < 0.45:
						s_control = np.array([[   0.0,   0.0,  0.0,   0.0,  -0.0,  -0.0],
											  [   0.2,   2e-4,   0.0,   0.0,   0.0,   0.0],
											  [   0.0,   0.0, -0.0,  -0.0,  -0.0,  -0.0],
											  [  -0.2,  -2e-4,   0.0,   0.0,   0.0,   0.0]]) @ self.state
					else:
						s_control = np.array([[   0.0,   0.0,  0.0,   0.0,  -0.0,  -0.0],
											  [   0.3,   0.0,   0.0,   0.0,   0.0,   0.0],
											  [   0.0,   0.0, -0.0,  -0.0,  -0.0,  -0.0],
											  [  -0.3,  -0.0,   0.0,   0.0,   0.0,   0.0]]) @ self.state

				if ctr == 1.5 / self.dt:
					throttle = 0

				control = []
				for c in s_control:
					control.append(np.min([1.0, np.max([0.0, c + throttle])]))

				ctr += 1

				msg = Float64MultiArray()
				msg.data = control
				self.dbg_publisher.publish(msg)


			self.update_timer += 1
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('Kwad_Controller')
	controller = Kwad_Controller()
	controller.spin()
	
