#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import the necessary packages
import rospy
import rospkg
from std_msgs.msg import Float32
import serial
import matplotlib.pyplot as plt
from numpy import pi

class Imu_9dof_m0():

    def __init__(self, port):
        self.buffer_string = ''
        self.ser = serial.Serial(port,baudrate=300)


    def receiving(self):
        self.buffer_string += self.ser.read(self.ser.inWaiting())
        if '\n' in self.buffer_string:
            lines = self.buffer_string.split('\n') # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            self.buffer_string = lines[-1]
            return last_received
        else:
            return None


    def getter(self):
        line = self.receiving()
        if line is not None:
            yaw = eval(line.split('=')[1].split(',')[0])
            return yaw
        else:
            return 0




if __name__ == "__main__":
# Using orientation convention North-East-Down

    rospy.init_node('image_visualisation', anonymous=True)
    pub_yaw = rospy.Publisher("heading", Float32, queue_size=2)

    rate = rospy.Rate(20)

    imu = Imu_9dof_m0("/dev/ttyACM0")


    while not rospy.is_shutdown():
        yaw = imu.getter()
        pub_yaw.publish(Float32(data=pi*yaw/180))
        rate.sleep()

    imu.ser.close()
