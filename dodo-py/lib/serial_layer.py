#! /usr/bin/env python3


"""
	Arduino reader/writer script
	Author : Mitchell Scott
		- misc4432@colorado.edu
	Project : Rufous
"""
import os
import sys
import yaml
import time
import rospy
import serial
import traceback
import numpy as np
import traceback as tb
from std_msgs.msg import String, Float64, Float64MultiArray, ByteMultiArray

class SerialAgent:
	def __init__(self):
		"""
		  A node that listens to an arduino sending one-liners.
		The first element of each msg is the id, the second is the 
		type of message. All messages are comma seperated sequences.
		There are a few optional parameters that can change 
		how you publish the data.
		"""
		self.port_num = 0
		self.device = None
		self.sensors = ['accel', 'gyro', 'mag']
		self.publishers = {}
		rospy.init_node('serial_layer')
		self.sub = rospy.Subscriber('/teensy_command', Float64MultiArray, self.writerCallback)
		self.initPublishers()

		self.tryConnect()

	def tryConnect(self, alt=0):
		"""
		  Try to connect with the device.
		Returns:
		  status: bool - True if successful
		"""
		try:
			if self.device:
				self.device.close()

			self.device = serial.Serial(f'/dev/ttyACM{self.port_num}', 1000000, timeout=10)
			self.log('Device Connected: Reading...')


		except Exception as e:
			self.log(e)
			# ACM# likes to switch after reset, just check all of em (0..2)
			self.port_num += 1
			if self.port_num >= 5:
				exit(0)

			time.sleep(2)

			self.tryConnect()


	def initPublishers(self):
		"""
		  Creates publishers.
		"""
		for topic in self.sensors:
			self.publishers[topic] = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
			self.publishers[f'{topic}_raw'] = rospy.Publisher(f'{topic}_raw', Float64MultiArray, queue_size=1)


	def writeBack(self, msg):
		"""
		  Write a message back to the arduino
		"""
		if not self.device is None:
			self.device.write(msg)
		

	def writerCallback(self, msg):
		"""
		  Callback for writing messages to the arduino
		  Ros message float 0-1, writer uses byte 0:255
		"""

		# convert to uint16 then bytes
		data = []
		for c in msg.data:
			data.append(np.max([0, (c * 2**16) - 1]))

		byte_array = b'UU:' + np.array(data).astype(np.uint16).tobytes()
		self.writeBack(byte_array)

	def log(self, text):
		"""
		  Wrapper to print a value through /rosout.
			Params:
			  text: value - prints this
		"""
		rospy.loginfo(f'[Serial Info]: {text}')

	def parsePacket(self, packet, ts):
		"""
		  Create a new msg to publish.
			Params:
				topic: string - destination for the publisher
				msgType: string - the datatype of the msg
				data: ??? - the data to fill msg
			Returns:
			  msg: ??? - the msg to send
		"""
		data = packet.split(',')

		if data[0] == 'II' and len(data[1]) > 0:
			accel_raw = data[1:4]
			gyro_raw = data[4:7]
			mag_raw = data[7:]

			msg = Float64MultiArray()
			msg.data = np.array(accel_raw, dtype=np.float64)
			self.publishers['accel_raw'].publish(msg)

			msg = Float64MultiArray()
			msg.data = np.array(gyro_raw, dtype=np.float64)
			self.publishers['gyro_raw'].publish(msg)
			
			msg = Float64MultiArray()
			msg.data = np.array(mag_raw, dtype=np.float64)
			self.publishers['mag_raw'].publish(msg)

		else:
			self.log(packet)

	def spin(self):
		"""
	
		"""
		try:
			while not rospy.is_shutdown():
				if self.device is None:
					self.device.open()


				elif self.device.in_waiting:
					packet = self.device.readline().decode().rstrip()
					self.parsePacket(packet, rospy.Time.now())
					

		except Exception as e:
			traceback.print_exc()
			self.log(e)
			time.sleep(1)
			self.tryConnect()
			self.spin()

if __name__=='__main__':
	try:
		arduino_relay = SerialAgent()
		arduino_relay.spin()

	except KeyboardInterrupt:
		rospy.loginfo('[Serial_Agent]: Exiting ...')
		if arduino_relay.device:
			arduino_relay.device.close()

	except Exception as e:
		exc_type, exc_value, exc_traceback = sys.exc_info()
		tb.print_exc()
		if arduino_relay.device:
			arduino_relay.device.close()