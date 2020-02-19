#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

import cv2
import time
import numpy as np
from numpy import pi, cos, sin, tan, array, shape, zeros

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from skimage.segmentation import active_contour




def detectBuoy(image, roll, dataCam):
    Sf, resolution = dataCam[0], dataCam[1]
    colorRange = getColorRange()

    lower, upper = colorRange[0], colorRange[1]

    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only yellow/green colors
    mask1 = cv2.inRange(hsv, lower, upper)
    mask1 = cv2.medianBlur(mask1, 5)


    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image,image, mask= mask1)

    ret1,thresh1 = cv2.threshold(mask1,127,255,0)
    im2,contours1,hierarchy1 = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, 2)

    contours = sorted(contours1, key = cv2.contourArea, reverse = True)

    bearings = []
    for cnt in contours[:3]:
        if cv2.contourArea(cnt) > 15:  

            (x1,y1),radius1 = cv2.minEnclosingCircle(cnt)
            center1 = (int(x1),int(y1))
            radius1 = int(radius1)

            cv2.circle(image,center1,radius1,(255,0,0),1)
            cv2.circle(image,center1,1,(255,0,0),2)

            xBuoy = center1[0]*cos(roll)+center1[1]*sin(roll)  # roll in radians
            headingBuoy = (xBuoy-resolution[0]/2)*Sf

            bearings.append(headingBuoy)

    return [], image
    # return bearings[:1], image     ## WHILE DETECTION NOT RELIABLE
    # return bearings, image





def detectBuoy2(input_image, roll, dataCam):
    Sf, resolution = dataCam[0], dataCam[1]
    bearings = []

    # Convert BGR to HSV
    hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only red colors
    lower_low, upper_low = getColorRange_low_red()
    lower_high, upper_high = getColorRange_high_red()

    # Combine the masks
    mask1 = cv2.inRange(hsv, lower_low, upper_low)
    mask2 = cv2.inRange(hsv, lower_high, upper_high)

    # Remove noise
    mask = cv2.medianBlur(mask1+mask2, 5)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Identify distinct of red color
    im,contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, 2)

    # Initialize part of visualisation
    buoy_detected = zeros(input_image.shape).astype(np.uint8)

    if contours != []:
        for cnt in contours:
            # Find the most representative circle for each area
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            centre = (int(x), int(y))
            radius = int((1+0.)*radius)
            # Color is corresponding: draw in blue
            cv2.circle(input_image, centre, radius, (255,0,0))
            # Highlight the contour in the image
            cv2.drawContours(buoy_detected, [cnt], 0, (255,255,255), -1)

            # Compute the similarity of the area with a disc
            if cv2.contourArea(cnt) > (np.pi*(0.01/Sf)**2) and cv2.contourArea(cnt)/(np.pi*radius**2) > 0.45:
                # Similar to a disc : consider it is a buoy and draw in green
                cv2.circle(input_image, centre, radius, (0,255,0))

                # Compute direction from camera data
                xBuoy = centre[0]*cos(roll)+centre[1]*sin(roll)  # roll in radians
                headingBuoy = (xBuoy-resolution[0]/2)*Sf
                bearings.append(headingBuoy)

    # Highlight the contour in the image
    res_image = cv2.addWeighted(input_image, 0.6, buoy_detected, 0.4, 0.)

    return bearings, res_image



def getColorRange_low_red():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 0
    hue_max = 20
    sat_min = 30
    sat_max = 100
    val_min = 30
    val_max = 100

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)



def getColorRange_high_red():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 325
    hue_max = 360
    sat_min = 30
    sat_max = 100
    val_min = 30
    val_max = 100

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)




def image_callback(data):
    global image, bridge
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except:
        pass


def image_info_callback(data):
    global dataCam
    dataCam = (data.z, (data.x, data.y))


def run():
    global image, bridge, dataCam

    rospy.init_node('image_visualisation', anonymous=True)

    rate = rospy.Rate(5)

    display = rospy.get_param('display', False)

    pub_bearings = rospy.Publisher('buoys_directions', String, queue_size = 2)

    if display:
        cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)



# ###############          VIDEO           #############################
# ######################################################################

#     ##    Running on test video
#     r = rospkg.RosPack()
#     package_path = r.get_path('brave_2020_localisation')

#     cap = cv2.VideoCapture(package_path+'/src/Artificial_Vision/test_images/bouee_et_drone.MP4')

#     # resolution = (1280, 720)
#     resolution = (640, 480)
#     # resolution = (320, 240)
#     dataCam = (tan(0.398)/(resolution[0]*203.55/640), resolution)    
#     rate = rospy.Rate(32)

#     image = None

#     while(cap.isOpened()) and not rospy.is_shutdown():

#         # Capture frame-by-frame
#         ret, image_vid = cap.read()

#         if ret:
#             image = cv2.resize(image_vid, resolution)

#################     CAMERA     ####################################
#####################################################################

    rospy.Subscriber("camera/image_calibrated", Image, image_callback)
    rospy.Subscriber("camera/camera_info", Vector3, image_info_callback)

    image, dataCam = None, None
    bridge = CvBridge()


    while (image is None or dataCam is None) and not rospy.is_shutdown():
        rospy.sleep(1)

    while not rospy.is_shutdown():


######################################################################
######################################################################

        bearings, detection_image = detectBuoy2(image, 0, dataCam)

        buoys_bearings = String(data = str(bearings))
        pub_bearings.publish(buoys_bearings)

        #show the frame
        if display:
            cv2.imshow('Webcam',detection_image)
            # cv2.imshow('Webcam',cv2.resize(image,(240,180)))
            cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()




if __name__ == "__main__":
    run()

