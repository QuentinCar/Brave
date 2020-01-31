#!/usr/bin/env python

#http://www.cnbr13.com/2015/03/protocole-nmea-et-logiciels-de-navigation.html

import os, sys
import serial
import time
import rospy
from std_msgs.msg import String

from ublox.msg import Gps
from ublox.msg import Compass



def getFrameType(frame):
	return frame[1:6]

def fillPublisher(frameType, line):

	data = line.split(",")

	if frameType == "GPRMC":#GPS
		pub = Gps()
		#rospy.loginfo(data)
		pub.timeStamp = float(data[1])
		pub.validation = data[2]
		pub.latitude = float(data[3])
		pub.latitude_indic = data[4]
		pub.longitude = float(data[5])
		pub.longitude_indic = data[6]
		pub.speed = float(data[7])
		pub.heading = float(data[8])
		pub.date = float(data[9])
		pub.magnetic_declination = float(data[10])
		pub.declination_indic = data[11]
		pub.positionning_mode = data[12][0]

	elif frameType == "HCHDG":#Compass
		pub= Compass()
		rospy.loginfo(data)
		pub.heading = float(data[1])
		pub.heading_indic = data[3]
		pub.magnetic_declination = float(data[4])

	elif frameType == "WIMDA":
		rospy.loginfo(data)

	elif frameType == "WIMWV":#Wind
		rospy.loginfo(data)

	else:
		rospy.loginfo("Not a good frame")
		return None

	return pub



def ublox_node():

	ublox = serial.Serial('/dev/ttyUSB0',4800, timeout = 5)

	GPRMC_raw = rospy.Publisher('/ublox/raw_GPRMC', String, queue_size=10)#GPS frame
	HCHDG_raw = rospy.Publisher('/ublox/raw_HCHDG', String, queue_size=10)#Compass frame
	WIMDA_raw = rospy.Publisher('/ublox/raw_WIMDA', String, queue_size=10)
	WIMWV_raw = rospy.Publisher('/ublox/raw_WIMWV', String, queue_size=10)# Wind Frame ??

	GPRMC = rospy.Publisher('/ublox/GPRMC', Gps, queue_size=10)#GPS frame
	HCHDG = rospy.Publisher('/ublox/HCHDG', Compass, queue_size=10)#Compass frame
	WIMDA = rospy.Publisher('/ublox/WIMDA', Compass, queue_size=10)#Compass frame
	WIMWV = rospy.Publisher('/ublox/WIMWV', Compass, queue_size=10)#Compass frame

	pubDict_raw = {}
	pubDict_raw["GPRMC"] = GPRMC_raw
	pubDict_raw["HCHDG"] = HCHDG_raw
	pubDict_raw["WIMDA"] = WIMDA_raw
	pubDict_raw["WIMWV"] = WIMWV_raw

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

		if frameType in pubDict_raw:

			pubDict_raw[frameType].publish(line[6::])

			if frameType == "GPRMC" or frameType == "HCHDG":
				pubDict[frameType].publish(fillPublisher(frameType, line[6::]))


		rate.sleep()


if __name__ == '__main__':

	ublox_node()
