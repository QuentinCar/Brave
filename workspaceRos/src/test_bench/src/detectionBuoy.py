#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import cv2
import time
import numpy as np
from numpy import pi, cos, sin, array, shape

from std_msgs.msg import String

from webcamThread import WebVideoStream


def run():
    rospy.init_node('image_visualisation', anonymous=True)

    rate = rospy.Rate(10)

    vs = WebVideoStream(resolution=(320, 240), record = False).start()

    time.sleep(2)

    dataCam = vs.getScaleFactor()

    cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)

    pub_bearings = rospy.Publisher('CAMERA_bearings', String, queue_size = 2)

    while not rospy.is_shutdown():
        image = vs.read()
        if image is not None:
            bearings, image = detectBuoy(image, 0, dataCam)

            buoys_bearings = String(data = str(bearings))
            pub_bearings.publish(buoys_bearings)

            #show the frame
            cv2.imshow('Webcam',image)
            time.sleep(0.03)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        if key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF

        elif key == ord('c'):
            cv2.imwrite('sample.png',image)
            print("Picture saved")

        rate.sleep()

    cv2.destroyAllWindows()





def getColorRange():
    # define range of buoy color in HSV
    # voir https://www.google.com/search?client=firefox-b&q=%23D9E80F

    hue_min = 345
    hue_max = 400
    sat_min = 45
    sat_max = 100
    val_min = 40
    val_max = 100

    lower = np.array([int(hue_min/2),int(sat_min*255/100),int(val_min*255/100)])
    upper = np.array([int(hue_max/2),int(sat_max*255/100),int(val_max*255/100)])

    return (lower, upper)




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


    return bearings, image




if __name__ == "__main__":
    run()
