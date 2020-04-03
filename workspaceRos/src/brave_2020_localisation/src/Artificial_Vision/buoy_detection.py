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



def detectBuoy(input_image, roll, dataCam):
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




def undistort_raw_image(image, calibration_data):
    mtx,newcameramtx,dist,roi = calibration_data['Mi'], calibration_data['newMi'], calibration_data['dist'], calibration_data['roi']
    dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
    return dst



def image_callback(data):
    global image, bridge, calibration_data, dataCam, horizon_prev
    image_raw = bridge.imgmsg_to_cv2(data, "bgr8")
    # image = undistort_raw_image(image_raw, calibration_data) 
    image, horizon_height, horizon_prev = horizonArea(image_raw, None)
    dataCam = (tan(0.398)/(image.shape[1]*203.55/640), (image.shape[1], image.shape[0]))   


def image_info_callback(data):
    global dataCam
    dataCam = (data.z, (data.x, data.y))



def horizonArea(image, horizon_prev):
    if horizon_prev is None:
        init = True
    else:
        init = False
    try:
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    except:
        return None, None, None
    rows,cols = grey.shape
    rows_origin,cols_origin = rows,cols
    top_grey, rotation_prev = 0, 0

    if not init:
        #horizon_prev = (rotation, x0, y0, rotationMatrix)

        rotation_prev = horizon_prev[0]
        x0_prev = horizon_prev[1]
        y0_prev = horizon_prev[2]
        M_prev = horizon_prev[3]

        rotated = cv2.warpAffine(grey,M_prev,(cols,rows))

        rows_rotated, cols_rotated = shape(rotated)[0], shape(rotated)[1]

        horizon_prev_line = int(M_prev[1,0]*x0_prev + M_prev[1,1]*y0_prev + M_prev[1,2])
        bottom_margin, top_margin = 0.15, 0.15

        bottom_grey = min(rows_rotated,int(horizon_prev_line + bottom_margin*rows_rotated))
        top_grey = max(0, int(horizon_prev_line - top_margin*rows_rotated))

        grey = rotated[top_grey:bottom_grey, :]
        rows,cols = grey.shape

        # if __name__ == "__main__":
        #     cv2.imshow('Test', grey)


    kernel = np.zeros((7,7))
    kernel_side = np.ones((3,7))
    kernel[:3] = -(1./(4*7))*kernel_side
    kernel[-3:] = (1./(4*7))*kernel_side
    grad_y = cv2.filter2D(grey,cv2.CV_16S,kernel)
    grad_y = np.uint8(np.absolute(grad_y))


    ret, bin_y = cv2.threshold(grad_y,10,255,0)

    horizontalLines = cv2.HoughLines(bin_y,1,np.pi/180,100)

    if horizontalLines is not None:
        for rho,theta in horizontalLines[0]:
            rotation = (theta-np.pi/2)*180/np.pi + rotation_prev

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho


    else:
        rotation = horizon_prev[0]
        x0 = horizon_prev[1]
        y0 = horizon_prev[2]

    M = cv2.getRotationMatrix2D((x0, y0),rotation,1)
    horizon_prev = (rotation, x0, y0+top_grey, M)

    rotated = cv2.warpAffine(image,M,(cols_origin, rows_origin))

    rows_rotated, cols_rotated = shape(rotated)[0], shape(rotated)[1]

    horizon = int(M[1,0]*x0 + M[1,1]*y0 + M[1,2]) + top_grey

    # if __name__ == "__main__":
    cv2.line(rotated, (0, horizon), (640, horizon), (0,255,0), 1)

    left = max( int(M[0,0]*0 + M[0,1]*0 + M[0,2]), int(M[0,0]*0 + M[0,1]*rows_rotated + M[0,2]))+1
    right = min( int(M[0,0]*cols_rotated + M[0,1]*0 + M[0,2]), int(M[0,0]*cols_rotated + M[0,1]*rows_rotated + M[0,2]))-1

    bottom_margin, top_margin = 0.2, 0.01
    bottom = min(rows_rotated,int(horizon + bottom_margin*rows_rotated))
    top = max(0, int(horizon - top_margin*rows_rotated))

    cropped = rotated[top:bottom, left:right]

    horizon_height = int(top_margin*rows_rotated)

    # if __name__ == "__main__":
    #     cv2.imshow('Result', cropped)


    return cropped, horizon_height, horizon_prev




def run():
    global image, bridge, dataCam, calibration_data, horizon_prev

    rospy.init_node('buoy_detection', anonymous=True)

    rate = rospy.Rate(5)

    display = rospy.get_param('display', False)

    pub_bearings = rospy.Publisher('buoys_directions', String, queue_size = 2)


    r = rospkg.RosPack()
    calibration_path = r.get_path('brave_2020_localisation')+'/src/Artificial_Vision/param.npz'
    calibration_data = np.load(calibration_path)

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

    rospy.Subscriber("gopro/image_raw", Image, image_callback)
    rospy.Subscriber("camera/camera_info", Vector3, image_info_callback)

    image, dataCam, horizon_prev = None, None, None
    bridge = CvBridge()

    # wait for the topics to be published
    while (image is None or dataCam is None) and not rospy.is_shutdown():
        rospy.sleep(1)


    while not rospy.is_shutdown():

######################################################################
######################################################################

        bearings, detection_image = detectBuoy(image, 0, dataCam) #if available, replace 0 by roll

        buoys_bearings = String(data = str(bearings))
        pub_bearings.publish(buoys_bearings)

        #show the frame
        if display:
            cv2.imshow('Webcam',detection_image)
            # cv2.imshow('Webcam',cv2.resize(image,(640,480)))
            cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()




if __name__ == "__main__":
    run()

