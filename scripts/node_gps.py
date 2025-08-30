#!/usr/bin/env python3

import math
import pynmea2
import rclpy
import serial
import sys
import time

from rclpy.node import Node
from visionconnect.serial import SerialException

from ac_msgs.msg import Box

class SerialGps():

	RMC = "$GPRMC"	
	DEFAULT = "$GPRMC,191938.826,A,2835.344,N,08111.877,W,000.0,000.0,220822,000.0,W*69"
	
	def __init__(self):
		"""
			Initializes the serial port where the GPS module
			is located at
		"""

		try:
			self._gps = serial.Serial(
					port='/dev/ttyTHS0',
					baudrate=9600,
					bytesize=serial.EIGHTBITS,
					parity=serial.PARITY_NONE,
					stopbits=serial.STOPBITS_ONE,
					timeout=10
				)
				
		except SerialException as se:
			print(f'Serial Error: {str(se)}')
			print(f'Defaulting to none serial object')
			self._gps = None
		
	def gps_reading(self):
		"""
			Obtains a GPS reading, decoding it to an
			ASCII string
		"""
		
		return self._gps.readline().decode('ascii')
		
	def get_gps_buffer(self):
		"""
			Returns the number of bytes that are in the
			receive buffer
		"""
		
		return self._gps.in_waiting		
		
	def is_init(self):
		"""
			Checks to see if the serial port has been opened
			and initialized
		"""
		
		return self._gps is not None

# TODO: try to set it up in such a way that if a proper reading/sentence can't be
# processed, to default to a previous known location to avoid issues
class NodeGps(Node):

	def __init__(self):
	
		_timer_period = 0.5
		
		# Giving identifying name to node
		super().__init__('gps_node')		
		
		self.get_logger().info("Initalizing GPS node")
		
		# Creating publisher
		self._publisher = self.create_publisher(Box, 'gps', 1)
		
		# Create timer
		self.timer = self.create_timer(_timer_period, self._timer_callback)
		
		# Initialize the GPS Module
		self.gps = SerialGps()
		
		# Initializing previous location to default
		self._prev_loc = SerialGps.DEFAULT
		
	def _timer_callback(self):
		"""
			Callback function. Periodicity is set by the timer period
		"""
		
		# Initialize the message to be sent
		msg = Box()
		
		# Start with the previous location
		data = self._prev_loc
		
		# Check to see if serial object has been initialized
		if (self.gps.is_init()):
		
			# Keep reading until RMC message is found
			# What happens if input buffer runs out?
			while (self.gps.get_gps_buffer() > 0):
				
				# This is an encoded byte string
				data = self.gps.gps_reading()
				
				data_split = data.split(",")
				
				print(data)
				
				# RMC message has been found
				if (data_split[0] == SerialGps.RMC):
					# If location fix is invalid, default to previous location
					#if (data_split[7] == '0.00' and data_split[8] == '0.00'):
					#	data = self._prev_loc
					break
		else:
			data = self._prev_loc
		
		# Parse the sentence
		result = pynmea2.parse(data)
		
		# Final check for RMC
		if (isinstance(result, pynmea2.types.talker.RMC)):
			self._prev_loc = data
		else:
			result = pynmea2.parse(self._prev_loc)
		
		# Prepare the message
		msg.data[0] = result.latitude
		msg.data[1] = result.longitude
		msg.data[2] = result.spd_over_grnd * 1.15
		msg.data[3] = 0
		
		# Publish
		self._publisher.publish(msg)
		
		# Display the message to the console
		self.get_logger().info(f'Lat: {msg.data[0]}, Lon: {msg.data[1]}, Speed: {msg.data[2]}')

#		except ParseError as pe:
#			print(f'Parse error: {str(pe)}')
#			print(f'Defaulting to not publishing anything')
#			self.get_logger().info(f'No published message...')
#			pass

def main(args=None):

	# Initialize client library
	rclpy.init(args=args)
	
	# Create node
	gps_publisher = NodeGps()
	
	#self.get_logger().info("STaring node initialization")
	
	# Will a spin be needed here?
	rclpy.spin(gps_publisher)
	
	# Shut down
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
