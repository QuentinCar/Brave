#!/usr/bin/env python

#http://www.cnbr13.com/2015/03/protocole-nmea-et-logiciels-de-navigation.html

import os, sys
import serial
import time
import rospy
from std_msgs.msg import String

def getFrameType(frame):
	return frame[1:6]

def ublox_node():

	ublox = serial.Serial('/dev/ttyUSB0',4800, timeout = 5)

	raw_ublox = rospy.Publisher('raw_ublox', String, queue_size=10)

	GPRMC = rospy.Publisher('ublox_GPRMC', String, queue_size=10)#GPS frame
	HCHDG = rospy.Publisher('ublox_HCHDG', String, queue_size=10)#Compass frame
	WIMDA = rospy.Publisher('ublox_WIMDA', String, queue_size=10)# Pressure Frame ??
	WIMWV = rospy.Publisher('ublox_WIMWV', String, queue_size=10)# Wind Frame ??

	pubDict = {}
	pubDict["GPRMC"] = GPRMC
	pubDict["HCHDG"] = HCHDG
	pubDict["WIMDA"] = WIMDA
	pubDict["WIMWV"] = WIMWV

	rospy.init_node('ublox_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		line = ublox.readline()

		if len(line) == 0:
			print("Time out! Exit.\n")
			sys.exit()

		frameType = getFrameType(line)

		if frameType in pubDict:

			raw_ublox.publish(line)
			pubDict[frameType].publish(line[6::])
			rate.sleep()


if __name__ == '__main__':

	ublox_node()
