#!/usr/bin/env python2
############################################################
# Localisation simulation6: 
#     mobile: immobile
#     landmarks: 3
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#     noise: yes
#     limited range: no
############################################################     


from vibes import *
from pyibex import *
from pyibex.geometry import SepPolarXY

from numpy import pi, array, asarray, zeros, ones, uint8
from math import factorial
from itertools import permutations
import cv2
from time import time



if __name__ == "__main__":
##################################################################################################
#########################     Creating observation       #########################################

    
    Marks = [[35, 70], [75,95], [25,120]]
    
    detection_range = oo
    
    heading = pi/180*60
    accuracy = pi/180*3 # accuracy of compass + accuracy of camera
    
    Alpha = [Interval(heading+0.44-accuracy, heading+0.44+accuracy), 
             Interval(heading+0.-accuracy, heading+0.+accuracy), 
             Interval(heading+0.61-accuracy, heading+0.61+accuracy)]
    
    
    #Field of research
    field_x_low, field_x_high = 0, 80
    field_y_low, field_y_high = 0, 130
    field = [field_x_low, field_x_high, field_y_low, field_y_high]
    
    pos_wanted_accuracy = 0.5


##################################################################################################
#########################       Useful functions         #########################################


def compute_boxes(P, anonymised_marks, anonymised_distances, anonymised_angles):
    seps = []
    for m,d,alpha in zip(anonymised_marks, anonymised_distances, anonymised_angles):
        sep = SepPolarXY(d, alpha)
        fforw = Function("v1", "v2", "(%f-v1;%f-v2)" %(m[0], m[1]))
        fback = Function("p1", "p2", "(%f-p1;%f-p2)" %(m[0], m[1]))
        sep = SepTransform(sep, fback, fforw)
        seps.append(sep)
        
    sep = SepQInterProjF(seps)
    sep.q = len(anonymised_marks) - 3      # How many measures can be considered as outliers. Here we want 3 correct measures.
    
    # Compute all possible positions
    inner_boxes, outer_boxes, frontier_boxes = pySIVIA(P, sep, pos_wanted_accuracy)#, draw_boxes = False)
    return inner_boxes, outer_boxes, frontier_boxes 



def compute_gathered_positions(map, inner_boxes, field, pos_wanted_accuracy):    
    # Create a binary map of the positions where the boat can be
    field_x_low, field_y_low = field[0], field[2]
    for box in inner_boxes :#+frontier_boxes:
        x_low = int((box[0][0]-field_x_low)/pos_wanted_accuracy)   # Translate to begin index at 0 with positive values. 1 pixel = pos_wanted_accuracy m2.
        x_high = int((box[0][1]-field_x_low)/pos_wanted_accuracy)        
        y_low = int((box[1][0]-field_y_low)/pos_wanted_accuracy)
        y_high =  int((box[1][1]-field_y_low)/pos_wanted_accuracy)
        
        map[y_low:y_high, x_low:x_high] = 255*ones((y_high-y_low, x_high-x_low), uint8)    
    im,contours,hierarchy = cv2.findContours(map, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Fit the different areas into rectangles
    possible_positions = []
    for cnt in contours:
        x,y,w,h  = cv2.boundingRect(cnt)
        x, y, w, h = int(x), int(y), int(w), int(h)            
#        cv2.rectangle(map, (x,y),(x+w,y+h),(255,0,0),1)        
        centre_real, shape_real = ((x + w/2)*pos_wanted_accuracy, (y + h/2)*pos_wanted_accuracy), (w*pos_wanted_accuracy, h*pos_wanted_accuracy)
        possible_positions.append([centre_real, shape_real])    
    return map, possible_positions




def anonymise_measures(landmarks, bearings, detection_range):
    anonymised_distances = [Interval(0,detection_range)] * len(landmarks)**2
    anonymised_angles = bearings*len(landmarks)
    anonymised_marks = landmarks
    for i in range(1,len(landmarks)):
        anonymised_marks = anonymised_marks + landmarks[i:] + landmarks[:i]
    return anonymised_marks, anonymised_angles, anonymised_distances 
    



##################################################################################################
######################### Compute all possible positions #########################################

if __name__ == "__main__":
    
    ### Anonymising measures to simulate the fact that buoys are not recognisable
    
    anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(Marks, Alpha, detection_range)
    
    
    
    t0 = time()
    
    ## Drawing
    vibes.beginDrawing()
    vibes.newFigure("Localization")
    vibes.setFigureProperties({'x':130, 'y':100, 'width':800, 'height': 800})
    
    
    
    
    
    # Set constraints
    P = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
    inner_boxes, outer_boxes, frontier_boxes = compute_boxes(P, anonymised_marks, anonymised_distances, anonymised_angles)
        
    
    
    ## Drawing
    for m in Marks:
    	vibes.drawCircle(m[0], m[1], 1, 'yellow[black]')
    
    vibes.axisEqual()
     
    vibes.saveImage("boat_positions.png")
    
    vibes.endDrawing()
    
    
    t1 = time()
    print "Temps de calcul: "
    print t1-t0
    print
    
    
    
    
    
    ##################################################################################################
    ############################     Gather in areas      ############################################
    
    
    
    cv2.namedWindow("Test", cv2.WINDOW_NORMAL)
    
    map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
    map, possible_positions =  compute_gathered_positions(map, inner_boxes, field)
    
    print "Positions possibles:"
    print asarray(possible_positions)
    print
    
    print "Temps de rassemblement: "
    print time()-t1
    
    cv2.imshow("Test", cv2.flip(map, 0))
    key = cv2.waitKey(0)
    if key & 0xFF == 27:    
        cv2.destroyAllWindows()
    











