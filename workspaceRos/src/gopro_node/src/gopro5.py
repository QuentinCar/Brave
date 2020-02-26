#!/usr/bin/env python

import cv2
import time
import socket
import constants

from goprolib import GoPro

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import rospy
import numpy as np
import rospkg



def undistort(img,mapx,mapy):
    
    #dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    return dst

# Load undistortion data
rospack = rospkg.RosPack()
path = rospack.get_path('gopro_node')
data = np.load(path + '/src/calib/calibration_data.npz')
distCoeff,intrinsic_matrix,mapx,mapy, = data['distCoeff'], data['intrinsic_matrix'], data['mapx'], data['mapy']

#launch gopro stream
WRITE = False
gpCam = GoPro()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time.time()
gpCam.livestream("start")
gpCam.video_settings(res='1080p', fps='30')
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
counter = 0

#create bridge opencv -> ros publisher
bridge = CvBridge()
raw_image_pub = rospy.Publisher("/gopro/image_raw",Image,queue_size=2)
und_image_pub = rospy.Publisher("/gopro/image_undistort",Image,queue_size=2)

rospy.init_node('gopro_node', anonymous=True)

while not rospy.is_shutdown():
    nmat, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)
    if WRITE == True:
        cv2.imwrite(str(counter)+".jpg", frame)
        counter += 1
        if counter >= 10:
            break
    if cv2.waitKey(1) & 0xFF == 27:
        break
    if time.time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time.time()


    raw_image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    und_image_pub.publish(bridge.cv2_to_imgmsg(undistort(frame,mapx,mapy), "bgr8"))
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
