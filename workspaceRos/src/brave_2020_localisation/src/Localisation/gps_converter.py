#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rospkg

import cv2

import pyautogui as pg

import utm

from numpy import sqrt


# mouse callback function
def set_origin(event,x,y,flags,param):
    """ 
    Creates a point ((x,y),(utmX,utmY,...)) from a click on the image.
    """
    global nb_clicks, marks, marks_coordinates, full
    if event == cv2.EVENT_LBUTTONDBLCLK and (nb_clicks < len(marks_coordinates) or marks_coordinates ==[]):

        if marks_coordinates == []:
            user_input = pg.prompt(text="Prompt lat/long of the point (Google Map decimal style, comma separated). Not Minute angles.\
                                        \n Type 'END' if last point.\n Ex: 48.198797, -3.013792 END",
                                  title='GPS coordinates', 
                                  default='')
            if "END" in user_input:
                user_input = user_input[:-3]
                full = True
        else:
            user_input = marks_coordinates[nb_clicks]

        latitude, longitude = map(eval,user_input.split(','))
        res = utm.from_latlon(latitude, longitude)
        point = ((x,y),res)
        marks.append(point)

        nb_clicks += 1



def wait_click():
    global nb_clicks
    entry = nb_clicks
    while (not rospy.is_shutdown()) and nb_clicks == entry:        
        cv2.waitKey(1000)
    return



def draw_axis(image):    
    rows, cols, _ = image.shape        
    cv2.arrowedLine(image, (10, rows-10), (50, rows-10), (0,0,255), thickness = 3)
    cv2.arrowedLine(image, (10, rows-10), (10, rows-50), (0,0,255), thickness = 3)
    cv2.putText(image, "N", 
                (12, rows-50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1, cv2.LINE_AA)
    cv2.putText(image, "E", 
                (50, rows-12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1, cv2.LINE_AA)
    return image





def marks_acquisition(display):
    """
    Returns the correspondance between image points and GPS coordinates.
    First given coordinate will become the origin of local frame.
    Convention: 
                X: East
                Y: North
                Z: Up
    """

    global nb_clicks, marks, marks_coordinates, full

#################################################################################################################
########################################      USER SETUP       ##################################################
    marks_coordinates =  eval(rospy.get_param('marks_coordinates', "[]"))

#################################################################################################################
#####################################      ROS INITIALISATION       #############################################
    r = rospkg.RosPack()
    package_path = r.get_path('brave_2020_localisation')

    marks =[]

    if not display:
        for user_input in marks_coordinates:
            latitude, longitude = map(eval,user_input.split(','))
            res = utm.from_latlon(latitude, longitude)
            marks.append(res)
        base_map, base_map_original = None, None

    else:
        cv2.setMouseCallback('base_map',set_origin)
        confirm = False

        while not confirm and not rospy.is_shutdown():
            base_map_original = cv2.imread(package_path+'/src/Localisation/base_map/base_map.png')
            base_map = draw_axis(base_map_original)

            nb_clicks = 0
            full = False
            cv2.imshow("base_map", base_map)

    #################################################################################################################
    #####################################      WAITING USER ACTIONS     #############################################

            while not full and not rospy.is_shutdown():

                wait_click()
                rospy.loginfo("New point : \t Pixel "+str(marks[-1][0])+"\t Global "+str(marks[-1][1]))
                if marks_coordinates != []:
                    full = (len(marks) == len(marks_coordinates))

                if len(marks)>1:
                    point1, point2 = marks[-2:]
                    distance_NS = point2[1][0]-point1[1][0]
                    distance_WE = point2[1][1]-point1[1][1]
                    distance_absolute = sqrt(distance_WE**2+distance_NS**2)

                    cv2.line(base_map, point1[0], point2[0], (255,0,0))
                    cv2.putText(base_map, "Distance: "+str(round(distance_absolute,1))+" m", 
                                ((point1[0][0]+point2[0][0])//2, (point1[0][1]+point2[0][1])//2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 1, cv2.LINE_AA)


            cv2.imshow("base_map", base_map)

            rospy.loginfo("Hit enter to confirm, suppr to redo, echap to cancel.")
            key = cv2.waitKey(0) & 0xFF
            while key not in [10,8,27] and not rospy.is_shutdown(): # hit enter to confirm, suppr to redo, echap to cancel
                key = cv2.waitKey(0) & 0xFF
            if key == 10:
                confirm = True
                rospy.loginfo("Marks coordinates confirmed.\n")
            elif key == 8:
                confirm = False
                rospy.loginfo("New acquisition of marks coordinates.\n")
            elif key == 27:
                rospy.loginfo("Marks coordinates acquisition cancelled.\n")
                break

    return base_map_original, base_map, marks



#################################################################################################################
#########################################      MAIN FUNCTION     ################################################


def get_local_coordinates(display):
    if display:
        cv2.namedWindow("base_map", cv2.WINDOW_NORMAL)
        
    base_map_original, base_map, marks = marks_acquisition(display)


    if display:
        origin = (marks[0][1][0], marks[0][1][1])
        origin_px = (marks[0][0][0], marks[0][0][1])
    else:
        origin = (marks[0][0], marks[0][1])

    local_marks = []
    for mark in marks:
        if display:
            local_marks.append([mark[1][0]-origin[0], mark[1][1]-origin[1]])
        else:
            local_marks.append([mark[0]-origin[0], mark[1]-origin[1]])


    rospy.loginfo("Got local coordinates of marks:\n"+str(local_marks)+'\n')

    if display:
        cv2.imshow("base_map", base_map)

    #################################################################################################################
    #######################      FROM GIVEN MARKS, COMPUTE THE LIMITS OF SEARCH FIELD   #############################

        list_WE = sorted(marks, key = lambda mark: mark[1][0])
        list_SN = sorted(marks, key = lambda mark: mark[1][1])

        most_north = list_SN[-1]
        most_south = list_SN[0]
        most_east = list_WE[-1]
        most_west = list_WE[0]

        dx_global = most_east[1][0]-most_west[1][0]   #m
        dy_global = most_north[1][1]-most_south[1][1] #m

        dx_pixel = most_east[0][0]-most_west[0][0]    #px
        dy_pixel = -(most_north[0][1]-most_south[0][1])  #px

        rows,cols,_ = base_map.shape #px


        map_y_low = dy_global/dy_pixel*(origin_px[1]-rows)
        map_y_high = dy_global/dy_pixel*(origin_px[1]-0) 
        map_x_low = dx_global/dx_pixel*(0-origin_px[0])   
        map_x_high = dx_global/dx_pixel*(cols-origin_px[0])  #m

    else:
        map_x_low, map_x_high, map_y_low, map_y_high = eval(rospy.get_param('field_limits', "[-100,100,-100,100]"))
        origin_px, dx_global, dx_pixel, dy_global, dy_pixel = None, 0, 1, 0, 1

    rospy.loginfo("\nMap limits: "+str((map_x_low, map_x_high, map_y_low, map_y_high)))

    if display:
        rospy.loginfo("Hit enter to confirm.")
        key = cv2.waitKey(0) & 0xFF
        while key != 10 and not rospy.is_shutdown():
            key = cv2.waitKey(0) & 0xFF
        cv2.destroyWindow("base_map")


    return base_map_original, (origin_px, dx_global/dx_pixel, dy_global/dy_pixel),\
           (map_x_low, map_x_high, map_y_low, map_y_high), origin, local_marks





if __name__ == "__main__":
    rospy.init_node("gps_converter")
    get_local_coordinates()