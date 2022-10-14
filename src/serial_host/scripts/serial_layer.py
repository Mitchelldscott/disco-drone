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
from std_msgs.msg import String, Float64, Float64MultiArray

class SerialAgent:
	def __init__(self):
		"""
		  A node that listens to an arduino sending one-liners.
		The first element of each msg is the id, the second is the 
		type of message. All messages are comma seperated sequences.
		There are a few optional parameters that can change 
		how you publish the data.
		"""
		self.nPorts = 0
		self.config = None
		self.device = None
		self.sensors = ['accel', 'gyro', 'mag']
		self.publishers = {}
		# self.loadDevice(config)
		rospy.init_node('agent', anonymous=True)
		self.sub = rospy.Subscriber('/serial_out', String, self.writerCallback)
		self.initPublishers()

		self.tryConnect()

	def loadDevice(self, config):
		"""
		  Try to load the conf file for a device.
		"""
		kill = 0
		with open(config, 'r') as stream:
			self.config = yaml.safe_load(stream)

		self.log(self.config)
		if not 'State' in self.config:
			kill = 1
			self.log('Missing state description')
		elif not self.config['State']['WHOAMI']:
			self.config['State']['WHOAMI'] = 'Serial_Agent'
		if not 'Baudrate' in self.config:
			kill = 1
			self.log('Missing Baudrate')
		if not 'Ports' in self.config:
			kill = 1
			self.log('Missing Ports')
		else:
			 self.nPorts = len(self.config['Ports'])

		if kill:
			self.log('Check your config file')
			sys.exit(0)

	def tryConnect(self, alt=0):
		"""
		  Try to connect with the device.
		Returns:
		  status: bool - True if successful
		"""
		try:
			if self.device:
				self.device.close()

			self.device = serial.Serial('/dev/ttyACM0', 1000000, timeout=10)
			self.device.flush()
			self.connected = True

		except Exception as e:
			self.log(e)
			self.log('No Device')
			time.sleep(5)
			exit(0)

		self.log('Device Connected: Reading...')

		return self.connected

	def initPublishers(self):
		"""
		  Creates publishers.
		"""
		for topic in self.sensors:
			self.publishers[topic] = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
			self.publishers[f'{topic}_raw'] = rospy.Publisher(f'{topic}_raw', Float64MultiArray, queue_size=1)


	def writeBack(self, mesg):
		"""
		  Write a message back to the arduino
		"""
		self.device.write(mesg)

	def writerCallback(self, msg):
		"""
		  Callback for writing messages to the arduino
		"""
		self.writeBack(bytes(msg.data, 'utf-8'))

	def log(self, text):
		"""
		  Wrapper to print a value through /rosout.
			Params:
			  text: value - prints this
		"""
		rospy.loginfo(f'[Serial_Agent]: {text}')

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

		if data[0] == 'II':
			accel_raw = data[1:4]
			gyro_raw = data[4:7]
			accel = data[7:10]
			gyro = data[10:13]
			mag = data[13:]

			
			msg = Float64MultiArray()
			msg.data = np.array(data[1:4], dtype=np.float64)
			self.publishers['accel_raw'].publish(msg)

			msg = Float64MultiArray()
			msg.data = np.array(data[4:7], dtype=np.float64)
			self.publishers['gyro_raw'].publish(msg)

			msg = Float64MultiArray()
			msg.data = np.array(data[7:10], dtype=np.float64)
			self.publishers['accel'].publish(msg)

			msg = Float64MultiArray()
			msg.data = np.array(data[10:13], dtype=np.float64)
			self.publishers['gyro'].publish(msg)
			
			msg = Float64MultiArray()
			msg.data = np.array(data[13:], dtype=np.float64)
			self.publishers['mag'].publish(msg)

		else:
			self.log(packet)

	def spin(self):
		"""
	
		"""
		try:
			while not rospy.is_shutdown():
				if not self.connected:
					self.tryConnect()

				elif self.device.in_waiting:
					packet = self.device.readline().decode().rstrip()
					self.parsePacket(packet, rospy.Time.now())

		except Exception as e:
			traceback.print_exc()
			self.log(e)
			self.log('Possible I/O Error: Restarting...')
			time.sleep(2)
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