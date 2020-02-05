#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import the necessary packages
import rospy
import rospkg
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import tf
import serial
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy import optimize

def IMU_callback(data):
    global roll, pitch, yaw
    q = data.orientation
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


def run():
    global roll, pitch, yaw
    roll, pitch, yaw = 999,999,999
    rospy.init_node('imu_getter', anonymous=True)

    pub_roll = rospy.Publisher('roll', Float32, queue_size = 2)
    pub_pitch = rospy.Publisher('pitch', Float32, queue_size = 2)
    pub_yaw = rospy.Publisher('yaw', Float32, queue_size = 2)
    rospy.Subscriber('/imu/data', Imu, IMU_callback)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        pub_roll.publish(roll)
        pub_pitch.publish(pitch)
        pub_yaw.publish(yaw)

        rate.sleep()



class Imu_9dof_m0():
    # Inspired from https://github.com/corentin-j/wrsc_plymouth_2019/blob/master/src/sensors/imu/imu_9dof.py

    def __init__(self, calibrate = True):
        r = rospkg.RosPack()
        self.package_path = r.get_path('test_bench')

        self.buffer_string = ''
        self.ser = serial.Serial("/dev/ttyACM0",baudrate=115200, timeout=0.1)
            # (SPACE) -- Pause/resume serial port printing
            # t -- Turn time readings on or off
            # a -- Turn accelerometer readings on or off
            # g -- Turn gyroscope readings on or off
            # m -- Turn magnetometer readings on or off
            # c -- Switch to/from calculated values from/to raw readings
            # q -- Turn quaternion readings on or off (qw, qx, qy, and qz are printed after mag readings)
            # e -- Turn Euler angle calculations (pitch, roll, yaw) on or off (printed after quaternions)
            # h -- Turn heading readings on or off
            # r -- Adjust log rate in 10Hz increments between 1-100Hz (1, 10, 20, ..., 100)
            # A -- Adjust accelerometer full-scale range. Cycles between ± 2, 4, 8, and 16g.
            # G -- Adjust gyroscope full-scale range. Cycles between ± 250, 500, 1000, 2000 dps.
            # s -- Enable/disable SD card logging

        if calibrate:   
            self.offset_x, self.offset_y, self.offset_z, self.max_amp = self.calibration_magnetometer(number_of_sec = 30)
            offsets = np.array([self.offset_x, self.offset_y, self.offset_z, self.max_amp])
            np.save(self.package_path+'/src/calibration.npy', offsets)
        else:
            offsets = np.load(self.package_path+'/src/calibration.npy')
            self.offset_x, self.offset_y, self.offset_z, self.max_amp = offsets[0], offsets[1], offsets[2], offsets[3]



    def calibration_magnetometer(self, number_of_sec=60):
        """
        Return the offset of the magnetometer
        """
        rospy.loginfo("\tMagnetometer calibration begins")
        rospy.loginfo("\t\tPlease rotate the imu around all directions")
        rospy.sleep(3)
        rospy.loginfo("\t\tReady ?")
        rospy.sleep(1)
        rospy.loginfo("\t\tGetting data, keep turning")
        mx,my,mz = [],[],[]
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - t0) < number_of_sec:
            line = self.ser.readline()
            [mag_x, mag_y, mag_z] = map(eval,line.split(", "))

            mx.append(mag_x)
            my.append(mag_y)
            mz.append(mag_z)

        rospy.loginfo("\t\tStop turning")

        params = [0.,0.,0.,0.]
        myResult = optimize.leastsq(self.res_sphere, params, args=(np.array(mx),np.array(my),np.array(mz)) )
        ox, oy, oz, r = myResult[0]
        max_amplitude = max([abs(np.min(my)-oy), 
                             abs(np.max(my)-oy)])

        rospy.loginfo("\t\tOffsets : {},{},{}".format(ox,oy,oz))
        rospy.loginfo("\tMagnetometer calibration ends")
        return ox,oy,oz,max_amplitude


    def res_sphere(self, p,x,y,z):
        """ residuals from sphere fit """
        a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
        distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
        err = distance - r                 # err is distance from input point to current fitted surface
        return err


    def get_angles(self):
        line = self.receiving()
        [mag_x, mag_y, mag_z] = map(eval,line.split(", "))
        mag_x, mag_y, mag_z = mag_x-self.offset_x, mag_y-self.offset_y, mag_z-self.offset_z
        
        side = mag_x

        if side < 0:
            yaw = 3*np.pi/2 + np.pi*mag_y/(2*self.max_amp)  # ok only of working without pitch/roll

        else:
            yaw = np.pi/2 - np.pi*mag_y/(2*self.max_amp)


        return yaw


    def receiving(self):
        self.buffer_string += self.ser.read(self.ser.inWaiting())
        if '\n' in self.buffer_string:
            lines = self.buffer_string.split('\n') # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            self.buffer_string = lines[-1]
        return last_received




def getter():
    rospy.init_node("IMU_getter")
    pub_send_euler_angles = rospy.Publisher("IMU_filtered_heading", Vector3, queue_size=2)
    imu = Imu_9dof_m0(calibrate = False)
    time.sleep(2)
    alpha = 0.75
    heading = 0

    print "Starting test"
    while not rospy.is_shutdown():
        yaw = imu.get_angles()

        heading = alpha*heading + (1-alpha)*yaw
        # print "Heading = ", heading
        pub_send_euler_angles.publish(Vector3(x=heading))
        time.sleep(0.1)


if __name__ == "__main__":
    # run()
    getter()


