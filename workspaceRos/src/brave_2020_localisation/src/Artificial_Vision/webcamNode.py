#!/usr/bin/env python
# -*- coding: utf-8 -*-


# import the necessary packages
import rospy
import rospkg

import cv2
import time
from numpy import array, tan
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

from chessboard_calibration import getCamDistortData



def getScaleFactor(resolution=(640, 480)):
    scale_factor = tan(0.398)/(resolution[0]*203.55/640)   #Camera scale factor, rad/pixel
    return (scale_factor, resolution)


def run():
    rospy.init_node('image_getter', anonymous=True)

    rate = rospy.Rate(5)

    r = rospkg.RosPack()
    package_path = r.get_path('brave_2020_localisation')

    # camera = cv2.VideoCapture(0)
    camera = cv2.VideoCapture("/dev/video2")

    scale_factor, resolution = getScaleFactor()

    image_pub = rospy.Publisher("camera/image_calibrated",Image, queue_size = 0)
    info_pub = rospy.Publisher("camera/camera_info",Vector3, queue_size = 0)
    bridge = CvBridge()
    camInfo = Vector3(x=resolution[0],y=resolution[1],z=scale_factor)

    # Read the camera matrix from calibration file
    calibration_matrix, calibration_dist = getCamDistortData(package_path+'/src/Artificial_Vision/calibration_data.txt')


    while camera.isOpened() and not rospy.is_shutdown():
        ret, frame = camera.read()

        # grab the frame from the stream
        frame = cv2.resize(frame, resolution)
        frame = cv2.undistort(frame, calibration_matrix, calibration_dist, None)

        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            info_pub.publish(camInfo)
        except Exception as e:
            rospy.loginfo('Error cam: {0}'.format(e))

        rate.sleep()


    camera.release()





if __name__ == "__main__":
    run()