#!/usr/bin/env python2
############################################################
# Localisation simulation12: 
#     mobile: tank style
#     landmarks: any
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed
#     noise: yes
#     limited range: yes
# Remarks: Relies on (noisy) measure of speed. Would be better to get rid of it if possible.
#          No measure of distance to different landmarks
#          Real time version, not a simulation
############################################################     

import rospy

from pyibex import *
from pyibex.geometry import SepPolarXY
from vibes import *

from roblib import *

import numpy as np
from numpy import pi, array, asarray, zeros, ones, uint8, arange
import random
from itertools import permutations
from time import time, sleep
import cv2

from std_msgs.msg import Float32, String

from gps_converter import get_local_coordinates

##################################################################################################
#########################   IA computation functions     #########################################


def compute_all_positions(X_boxes, landmarks, directions, speed, heading, pos_wanted_accuracy, lateral_speed, field, range_of_vision, dt):
    """
    Based on Interval Analysis methods, computes all the positions where the boat can possibly be.

    Inputs:
    - X_boxes: list of boxes representing the positions where the boat could have been at previous iteration
    - landmarks: list of lists of float, coordinates of every landmark in the local frame
    - directions: list of intervals corresponding to the angle between the heading line of the boat and the line of sight to
                   every visible mark, ie camera measure
    - speed: interval, measure of speed of the boat
    - heading: interval, compass measure
    - pos_wanted_accuracy: float, precision of the frontier delimiting the areas where the boat can be
    - lateral_speed: float, maximum lateral speed not basically taken into account into Dubin model
    - field: interval vector, field of research, from which the boat must never escape
    - range_of_vision: float, maximum distance at which a mark can be detected
    - dt: float, integration step

    Returns:
    - The list of all boxes [X_Interval, Y_Interval] in which the boat can be.
    """

    ### Static part ###

    # First case: some marks are visible. We first compute a "static estimation" from the knowledge of the position of the boat
    # relatively to a buoy. At this step we do not take into account the knowledge of the position of the boat at precedent iteration.
    # To sum up: when at least one mark is seen, we compute boxes. When no mark is visible, we only move the previously computed ones 
    # according to the state equation.
    if len(directions) != 0:   

        # From the compass and camera measures, compute the azimuths of lines linking the boat and visible marks.
        azimuths = []
        for direction in directions:
            azimuths.append(direction+heading)  

        # when at least one mark is detected, improve position estimation
        # the line below associates each measure successively to each landmark to check wether this association is possible or not
        # in the function compute_boxes (see below)
        anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(landmarks, azimuths, range_of_vision)        
        inner_boxes, outer_boxes, frontier_boxes = compute_boxes(field, anonymised_marks, anonymised_distances, anonymised_angles, pos_wanted_accuracy)
        static_estimations = inner_boxes + frontier_boxes
    
    # Second case: no mark is currently visible. Then the only thing we "staticly" know is that the boat is in the field of research.
    else:
        static_estimations = [field]
    



    ### Dynamic part ###

    # Noise modelisation: lateral movements
    lat_moves = Interval(0.).inflate(lateral_speed)

    # Modelisation of the movement of the boat: noisy Dubin model. Approximations are contained in the noise.
    # The positions where the boat can possibly be are represented by rectangles of different sizes.
    # From this model, we can move the boxes and increase their sizes to take into account the movement of the boat
    # between two iterations.

    f_evol = Function("x[2]", "(x[0] + %f*(%s*cos(%s) + %s*sin(%s)), x[1] + %f*(%s*sin(%s) + %s*cos(%s)))"%(2*(dt, str(speed), str(heading), str(lat_moves), str(heading))))
    dynamic_estimations = [f_evol.eval_vector(X_box) for X_box in X_boxes]
    



    ### Fusion of static and dynamic parts ###

    # The position of the boat must be simultaneously:
    #       - in the field of research
    #       - compatible with current observations
    #       - compatible with the position of the boat at the previous iteration
    # possible positions = field & static estimation & dynamic estimation
    X_boxes = []
    
    print "Static boxes: ", len(static_estimations)
    print "Dynamic boxes: ", len(dynamic_estimations)
    print
    
    if len(static_estimations) > 1: # meaning at least one mark is detected
        dynamic_union = dynamic_estimations[0]      # then we can afford to loose some precision on dynamics given
        for dynamic_box in dynamic_estimations[1:]: # that static estimations are better, in order to reduce computing time
            dynamic_union |= dynamic_box    # union         ## can be improved to keep knowledge of possible different areas
    
        for static_box in static_estimations:
            intersection_box = field & dynamic_union & static_box  # intersection
            if not intersection_box.is_empty():
                X_boxes.append(intersection_box)
                
    else: # In case no marks are detected, all estimations are computed based on system dynamics
        for dynamic_box in dynamic_estimations:
            intersection_box = field & dynamic_box
            if not intersection_box.is_empty():
                X_boxes.append(intersection_box)
    # List of boxes containing all possible positions of the boat
    return X_boxes




def compute_boxes(field, anonymised_marks, anonymised_distances, anonymised_angles, pos_wanted_accuracy):
    """
    Static part of the function above. Computes all the positions that correspond to the measures.
    Called only when a mark is detected.

    Inputs:
    - field: IntervalVector, the field of research from which the boat cannot escape
    - anonymised_marks: list of IntervalVectors containing multiple times the coordinates of the marks
    - anonymised_distances: list of Intervals containing multiple times the distances between the boat and the detected marks
    - anonymised_angles: list of Intervals containing multiple times the bearings meaured by the camera
    - pos_wanted_accuracy: float, size of the frontier boxes

    anonymised_marks/distances/angles come from teh function anonymise_measures.

    Returns:
    - the boxes that satisfy the measures, the ones that do not, and the frontier between inner and outer.
    """

    ### Definition of the constraints
    separators = []
    for i in range(len(anonymised_marks)):
        seps =[]
        n = len(anonymised_marks[i])
        # Set constraints for each possible association (mark, distance, azimuth). 
        # Most of them will result in no solution when several marks are seen at the same time.
        for m,d,alpha in zip(anonymised_marks[i], anonymised_distances[n*i:n*(i+1)], anonymised_angles[n*i:n*(i+1)]):
            sep = SepPolarXY(d, alpha)
            fforw = Function("v1", "v2", "(%s-v1;%s-v2)" %(str(m[0]), str(m[1])))
            fback = Function("p1", "p2", "(%s-p1;%s-p2)" %(str(m[0]), str(m[1])))
            sep = SepTransform(sep, fback, fforw)
            seps.append(sep)
            
        sep = SepQInterProjF(seps)
        sep.q = 0      # How many measures can be considered as outliers. Here we consider all measures to be correct.
        separators.append(sep)
    
    ### Compute all the boxes that comply with the constraints, given a frontier accuracy (recursive bisections of boxes)
    inner_boxes, outer_boxes, frontier_boxes = [], [], []
    for sep in separators:
        # Compute all possible positions. Factor 4 in accuracy is due to bissections effects.
        inner, outer, frontier = pySIVIA(field, sep, 4*pos_wanted_accuracy, draw_boxes = False)
        inner_boxes += inner
        outer_boxes += outer
        frontier_boxes += frontier    
    return inner_boxes, outer_boxes, frontier_boxes 



def compute_gathered_positions(inner_boxes, field, pos_wanted_accuracy):    
    """
    From the list of boxes in which the boat can be, computes the center of the different possible areas and the shapes of their
    bounding rectangles. Can be upgraded to return more moments of the diffent areas.

    Inputs:
    - inner_boxes: list of IntervalVectors, boxes that satisfy the measures and movements of the boat
    - field: IntervalVector, the field of research from which the boat cannont escape
    - pos_wanted_accuracy: float, size of the frontier boxes. Corresponds to the resolution of hte binary map.

    Returns:
    - binary_map: binary image, 255 if the boat can be in the pixel, else 0. Resolution = pos_wanted_accuracy, borders are those of the field
    - possible_positions: 3d-array,
    [[[x1,y1], [x_size1, y_size1]],
        ...
     [[xn,yn], [x_sizen, y_sizen]]] , where [xi,yi] and [x_sizei,y_sizei] are respectively the center and shape of the bounding rectangle of the ith area
     of the binary map (found with cv2.findContours).
    """

    # Create a binary map of the positions where the boat can be
    field_x_low, field_y_low = field[0][0], field[1][0]
    field_x_high, field_y_high = field[0][1], field[1][1]
    binary_map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
    for box in inner_boxes :#+frontier_boxes:
        x_low = int((box[0][0]-field_x_low)/pos_wanted_accuracy)   # Translate to begin index at 0 with positive values. 1 pixel = pos_wanted_accuracy m2.
        x_high = int((box[0][1]-field_x_low)/pos_wanted_accuracy)        
        y_low = int((box[1][0]-field_y_low)/pos_wanted_accuracy)
        y_high =  int((box[1][1]-field_y_low)/pos_wanted_accuracy)
        
        binary_map[y_low:y_high, x_low:x_high] = 255*ones((y_high-y_low, x_high-x_low), uint8)    
    im,contours,hierarchy = cv2.findContours(binary_map, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Fit the different areas into rectangles
    possible_positions = []
    for cnt in contours:
        x,y,w,h  = cv2.boundingRect(cnt)
        # x, y, w, h = int(x), int(y), int(w), int(h)            
#        cv2.rectangle(binary_map, (x,y),(x+w,y+h),(255,0,0),1)        
        centre_real = ((x + w/2)*pos_wanted_accuracy + field_x_low ,(y + h/2)*pos_wanted_accuracy + field_y_low)
        shape_real = (w*pos_wanted_accuracy, h*pos_wanted_accuracy)
        possible_positions.append([centre_real, shape_real])    
    return binary_map, possible_positions



def anonymise_measures(landmarks, azimuths, range_of_vision):
    """
    Function that assigns each combination of marks to every combination of measures. Necessary to test all associations
    and find the possible ones in function compute_boxes.
    """
    anonymised_marks = list(permutations(landmarks, len(azimuths)))
    anonymised_angles = azimuths*len(anonymised_marks)
    anonymised_distances = [Interval(0,range_of_vision)]*len(azimuths) *len(anonymised_marks)
    return anonymised_marks, anonymised_angles, anonymised_distances 




##################################################################################################
#############################   ROS callblacks     ###############################################
def sub_heading(data):
    global heading, heading_accuracy
    heading = Interval(sawtooth(pi/2-data.data)).inflate(heading_accuracy) # conversion to local frame ENU.


def sub_speed(data):
    global speed, speed_accuracy
    speed = Interval(data.data).inflate(speed_accuracy)


def sub_directions(data):
    global marks_directions, direction_accuracy
    directions = eval(data.data)
    marks_directions = map(lambda x: Interval(x).inflate(direction_accuracy), directions)


def sub_state(data):
    global real_boat_state_vector
    real_boat_state_vector = asarray(eval(data.data))

##################################################################################################
##################################   Main Loop    ################################################
##################################################################################################



def run():
    rospy.init_node("interval_localiser")
    global marks_directions, speed, heading, heading_accuracy, direction_accuracy, speed_accuracy, real_boat_state_vector
##################################################################################################
#########################         Setup section          #########################################

    # integration step
    dt = rospy.get_param('integration_step', 0.2)
    display = rospy.get_param('display', False)
    mark_position_accuracy = rospy.get_param('mark_position_accuracy', 5)        # Accuracy on marks positions

    # Field of research
    base_map, pixel_map_data, field_limits, origin_utm, landmarks_local = get_local_coordinates(display)

    landmarks = []
    for mark in landmarks_local:
        landmarks.append(IntervalVector([[mark[0]]*2,[mark[1]]*2]).inflate(mark_position_accuracy))

    field_x_low, field_x_high, field_y_low, field_y_high  = field_limits
    origin_px, x_scale, y_scale = pixel_map_data   #location of the origin in the matrix, conversion m/px

    search_field = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
    

    # Estimated specifications of sensors
    heading_accuracy = rospy.get_param('heading_accuracy', 5) *pi/180        # Compass accuracy
    direction_accuracy = rospy.get_param('direction_accuracy', 5) *pi/180    # Camera accuracy
    range_of_vision = rospy.get_param('range_of_vision', 15)                 # maximum distance for mark detection
    angle_of_perception = rospy.get_param('angle_of_perception', 50) *pi/180 # angle of perception of the camera
    speed_accuracy = rospy.get_param('speed_accuracy', 0.3)                  # accuracy on speed measurement.
    
    # To adapt the Dubins' model, we add a term that represents lateral variations of position
    max_lateral_speed = rospy.get_param('max_lateral_speed', 0.5)
    
    # Wanted accuracy on position
    pos_wanted_accuracy = rospy.get_param('pos_wanted_accuracy', 0.3)
    
       

    print "\n######## INIT ##############"
    print
    print "Integration step = ", dt
    print "Search field = ", search_field
    print "Heading acuracy = ", heading_accuracy
    print "Marks direction accuracy = ", direction_accuracy
    print "Range of vision = ", range_of_vision
    print "Angle of perception = ", angle_of_perception
    print "Speed accuracy = ", speed_accuracy
    print "Max lateral speed = ", max_lateral_speed
    print "Wanted accuracy = ", pos_wanted_accuracy
    print
    print "############################"


    # Initialising variables
    boat_possible_positions = [search_field]
    marks_directions = []
    speed = Interval(1,3)
    heading = Interval(-pi,pi)
    real_boat_state_vector = array([[0,0,0,0]]).T
    sleep(2)
##################################################################################################
#########################      ROS initialisation        #########################################


    pub_positions = rospy.Publisher("boat_possible_positions", String, queue_size = 2)
    pub_local_landmarks = rospy.Publisher("local_landmarks_coordinates", String, queue_size = 2)
    rospy.Subscriber("buoys_directions", String, sub_directions)
    rospy.Subscriber("heading", Float32, sub_heading)
    rospy.Subscriber("speed", Float32, sub_speed)


    rate = rospy.Rate(1/dt)
    rospy.loginfo("Initiated localisation")


##################################################################################################
#################################      Drawing        ############################################
    if display:
        ## Drawing
        cv2.namedWindow("position_map", cv2.WINDOW_NORMAL)
        vibes.beginDrawing()
        vibes.newFigure("Localization")
        vibes.setFigureProperties({'x':800, 'y':100, 'width':800, 'height': 800})        
        vibes.axisLimits(field_x_low, field_x_high, field_y_low, field_y_high)   
        rospy.Subscriber("state_truth", String, sub_state)

    # cv2.namedWindow("Test_no_display", cv2.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        
        # 1st interval function
        boat_possible_positions = compute_all_positions(boat_possible_positions, 
                                                        landmarks, 
                                                        marks_directions, 
                                                        speed, 
                                                        heading, 
                                                        pos_wanted_accuracy,
                                                        max_lateral_speed, 
                                                        search_field, 
                                                        range_of_vision, dt)

        #2nd interval function
        binary_map, possible_positions = compute_gathered_positions(boat_possible_positions, 
                                                             search_field , 
                                                             pos_wanted_accuracy)

        # cv2.imshow("Test_no_display", cv2.resize(binary_map,(480,360)))
        # cv2.waitKey(1)

        pub_positions.publish(String(data=str(possible_positions))) # still in local frame
        pub_local_landmarks.publish(String(data=str(landmarks_local)))

##################################################################################################
############################    Drawing     ############################################    

        if display:
            #Compare with real position
            X = real_boat_state_vector


            print '\n'
            print "Pos truth: ", X[0,0], X[1,0]
            print "Heading truth: ", X[2,0]
            print "Heading interval: ", heading
            print "Speed truth: ", X[3,0]
            print "Speed interval: ", speed
            print '\n'

            vibes.clearFigure()        
            scale = (field_y_high-field_y_low)/100. 
            for X_box in boat_possible_positions:
                vibes.drawBox(X_box[0][0], X_box[0][1], X_box[1][0], X_box[1][1], color='[blue]')
            vibes.drawCircle(X[0,0], X[1,0], 1*scale, 'blue[black]') 
            for b in marks_directions:
                vibes.drawPie((X[0,0],X[1,0]), (0.,range_of_vision), (X[2,0]+b[0],X[2,0]+b[1]), color='black',use_radian=True)             
            for m in landmarks_local:
                vibes.drawCircle(m[0], m[1], 1*scale, 'yellow[black]')  

        # if display:
            ## Display binary map
            position_mask = cv2.flip(binary_map, 0)
            resized_position_mask = cv2.resize(position_mask, (base_map.shape[1], base_map.shape[0]))
            # cv2.imshow("Possible positions", position_mask)
            colored_position_mask = cv2.bitwise_and(base_map,base_map, mask = resized_position_mask)
            display_map = cv2.addWeighted(base_map, 0.5, colored_position_mask, 0.5, 0)

            x_boat_px, y_boat_px = origin_px[0]+int(X[0,0]/x_scale), origin_px[1]-int(X[1,0]/y_scale)
            
            cv2.circle(display_map, 
                       (x_boat_px, y_boat_px), 
                       5, (0,0,255), thickness=5)
            cv2.line(display_map,
                     (x_boat_px, y_boat_px), 
                     (int(x_boat_px+15*cos((heading[0]+heading[1])/2)), int(y_boat_px-15*sin((heading[0]+heading[1])/2))),
                     (0,0,255), thickness = 3)

            cv2.imshow("position_map", cv2.resize(display_map,(480,360)))
            cv2.waitKey(1)

        rate.sleep()



if __name__ == "__main__":
    run()