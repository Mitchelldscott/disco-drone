#!/usr/bin/env python3

import math
import rospy
import tf
# because of transformations
import numpy as np

import tf2_ros
import geometry_msgs.msg

from ahrs.filters import Madgwick, Complementary, EKF
from std_msgs.msg import String, Float64, Float64MultiArray
from tf.transformations import euler_from_quaternion


# def euler_from_quaternion(q):
# 	"""
# 	Convert a quaternion into euler angles (roll, pitch, yaw)
# 	"""
# 	q0, q1, q2, q3 = q
	
# 	roll = np.arctan(2 * ((q0*q1) + (q2*q3)) / (1 - (2 * (q1**2 + q2**2))))
# 	pitch = np.arcsin(2 * ((q0*q2) - (q3*q1)))
# 	yaw = np.arctan(2 * ((q0*q3) + (q1*q2)) / (1 - (2 * (q2**2 + q3**2))))

# 	return np.array([roll, pitch, yaw]) 

def omega_from_quaternions(q1, q2, dt):
	"""
	Convert a quaternion into euler angle rates (droll, dpitch, dyaw)
	"""
	rpy1 = np.array(euler_from_quaternion(q1))
	# print(rpy1)
	rpy2 = np.array(euler_from_quaternion(q2))

	return (rpy1 - rpy2) / dt


class IMU_Odometry():
	def __init__(self):
		self.sensor_flags = np.zeros(3)

		self.madgwick = Madgwick(gain=0.45)
		self.complementary = Complementary(gain=0.25)
		self.ekf = EKF()

		self.sensor_mag = np.zeros(3)
		self.sensor_gyro = np.zeros(3)
		self.sensor_accel = np.zeros(3)

		self.omega = np.zeros(3)
		self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
		self.q_prop = np.array([1.0, 0.0, 0.0, 0.0])

		self.broadcaster = tf2_ros.StaticTransformBroadcaster()

		self.mag_sub = rospy.Subscriber('/mag_raw', Float64MultiArray, self.mag_callback)
		self.gyro_sub = rospy.Subscriber('/gyro_raw', Float64MultiArray, self.gyro_callback)
		self.accel_sub = rospy.Subscriber('/accel_raw', Float64MultiArray, self.accel_callback)

		self.state_publisher = rospy.Publisher('/state', Float64MultiArray, queue_size=1)

		self.valid_bias = False
		self.state_bias = np.zeros(3)

		rate = 384
		self.dt = 1 / rate
		self.rate = rospy.Rate(rate)
		self.madgwick.Dt = 1 / rate
		self.complementary.DT = 1 / rate
		self.ekf.DT = 1 / rate


	def mag_callback(self, msg):
		self.sensor_flags[0] = 1
		self.sensor_mag = np.array(msg.data)
	
	def gyro_callback(self, msg):
		self.sensor_flags[1] = 1
		self.sensor_gyro = np.array(msg.data) * np.pi / 180.0
	
	def accel_callback(self, msg):
		self.sensor_flags[2] = 1
		self.sensor_accel = np.array(msg.data) / 9.81

	def broadcast_tf(self):
		static_transformStamped = geometry_msgs.msg.TransformStamped()
		static_transformStamped.header.frame_id = "map"
		static_transformStamped.child_frame_id = "base_link1" 
		static_transformStamped.header.stamp = rospy.Time.now()
			
		static_transformStamped.transform.translation.x = 0.0
		static_transformStamped.transform.translation.y = 0.0
		static_transformStamped.transform.translation.z = 0.0

		static_transformStamped.transform.rotation.w = self.quaternion[0]
		static_transformStamped.transform.rotation.x = self.quaternion[1]
		static_transformStamped.transform.rotation.y = self.quaternion[2]
		static_transformStamped.transform.rotation.z = self.quaternion[3]
		self.broadcaster.sendTransform(static_transformStamped)

	def broadcast_state(self):
		msg = Float64MultiArray()
		msg.data = np.array(list(zip(self.euler_angles, self.omega))).reshape(6)
		self.state_publisher.publish(msg)


	def madgwick_filter(self):		
		self.quaternion = self.madgwick.updateMARG(self.quaternion, 
													mag=self.sensor_mag,
													gyr=self.sensor_gyro, 
													acc=self.sensor_accel)

	def ek_filter(self):		
		self.quaternion = self.ekf.update(self.quaternion, 
													mag=self.sensor_mag,
													gyr=self.sensor_gyro, 
													acc=self.sensor_accel)

	def complementary_filter(self):
		try:
			new_quaternion = self.complementary.update(self.quaternion, 
													mag=self.sensor_mag,
													gyr=self.sensor_gyro, 
													acc=self.sensor_accel)

			euler_angles = euler_from_quaternion(new_quaternion).flip() - self.state_bias
			self.omega = (euler_angles - self.euler_angles) / self.dt
			self.quaternion = new_quaternion
			self.euler_angles = euler_angles


		except ValueError:
			print('bad value dropping accel/mag frame')


	def spin(self):
		bias_count = 0

		while not rospy.is_shutdown():
			if np.sum(self.sensor_flags) > 0:
				self.sensor_flags *= 0
				self.complementary_filter()

				if self.valid_bias:
					self.broadcast_state()

				else:
					self.state_bias += self.euler_angles
					bias_count += 1
					if bias_count >= 50:
						self.valid_bias = True
						self.state_bias /= bias_count
				
			self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('localization')
	odom = IMU_Odometry()
	odom.spin()
	
