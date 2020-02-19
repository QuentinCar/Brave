#!/usr/bin/env python2
############################################################
# Localisation simulation7: 
#     mobile: tank style
#     landmarks: 3
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#     noise: yes
#     limited range: no
# Remarks: cf "compute all possible positions" section. Limited accuracy on position.
############################################################     

from vibes import *
from pyibex import *
from pyibex.geometry import SepPolarXY

from numpy import pi, array, asarray, zeros, ones, uint8, arange
from math import factorial
from itertools import permutations
import cv2
from time import time

from roblib import *
from simu6_intervals import anonymise_measures, compute_gathered_positions


if __name__ == "__main__":
##################################################################################################
#########################         Setup section          #########################################

    # Field of research
    field_x_low, field_x_high = 0, 150
    field_y_low, field_y_high = 0, 150
    field = [field_x_low, field_x_high, field_y_low, field_y_high]
    
    range_of_vision = oo
    
    # Wanted accuracy on position
    pos_wanted_accuracy = 0.5
    
    # Time step
    dt = 0.7              # Integration step
    slowing_ratio = 3
    
    
    # Landmarks
    landmarks = [[35, 70], [75,95], [25,120]]
    
    
    # Initial state of the boat
    Xinit = array([[10], [50], [-1], [2]])



##################################################################################################
#########################   IA computation functions     #########################################



##################################################################################################
################################  SIMULATION FUNCTIONS   #########################################

# Evolution equation
def evolX(X, u):
    """Returns the derivative of X."""
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX


def getBearings(X, landmarks, accuracy = pi/180*5):
    """Returns the bearing for the designated landmark."""
    x, y = X[0, 0], X[1, 0]
    bearings = []
    for landmark in landmarks:
        xl, yl = landmark[0], landmark[1]        
        camera_measure = sawtooth(atan2(yl-y, xl-x) - X[2,0])
        compass_measure = X[2,0]
        bearings.append(Interval(camera_measure+compass_measure-accuracy, camera_measure+compass_measure+accuracy))
    return bearings


def inRange(X, landmarks):
    """Returns the list of the landmarks that the boat is really able to see."""
    perceptionAngle = 45  #degrees
    spotted = []
    for i in range(len(landmarks)):
        dist = norm(X[:2]-landmarks[i])
        if dist < 15 and abs(getBearing(X, landmarks[i])) < perceptionAngle*pi/180:
            spotted.append(landmarks[i])
    return spotted



##################################################################################################
##################################  Simulation    ################################################

# Initialising variables
X = Xinit


# Main loop
if __name__ == "__main__":
    
    for t in arange(0, 30, dt):

##################################################################################################
############################  Simulate the measure    ############################################
        bearings = getBearings(X, landmarks)        
        anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(landmarks, bearings, range_of_vision)        
        
        
        
        
        ## Drawing
        vibes.beginDrawing()
        vibes.newFigure("Localization")
        vibes.setFigureProperties({'x':130, 'y':100, 'width':800, 'height': 800})
        
        
##################################################################################################
######################### Compute all possible positions #########################################        
        # Set constraints        
        P = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
        seps = []
        for m,d,alpha in zip(anonymised_marks, anonymised_distances, anonymised_angles):
            sep = SepPolarXY(d, alpha)
            fforw = Function("v1", "v2", "(%f-v1;%f-v2)" %(m[0], m[1]))
            fback = Function("p1", "p2", "(%f-p1;%f-p2)" %(m[0], m[1]))
            sep = SepTransform(sep, fback, fforw)
            seps.append(sep)
            
        sep = SepQInterProjF(seps)
        sep.q = len(anonymised_marks) - 3      # How many measures can be considered as outliers. Here we want 3 correct measures.
# Solution pas totalement satisfaisante car creation de faux-positifs.
# En effet, une meme mesure peut etre consideree comme correcte pour deux bouees distinctes et "compte double" dans ce cas.
# Cela se produit lorsque la mesure et la droite passant par deux bouees partagent le meme azimut. 
# On a dans ce cas une zone qui se cree sur cet azimut de sorte a satisfaire la troisieme contrainte.
        
        
        
        # Compute all possible positions. Factor 4 in accuracy is due to bissections effects.
        inner_boxes, outer_boxes, frontier_boxes = pySIVIA(P, sep, 4*pos_wanted_accuracy, display_stats=False)#, draw_boxes = False)



        ## Drawing
        for m, b in zip(landmarks, bearings):
            vibes.drawCircle(m[0], m[1], 1, 'yellow[black]')  
            vibes.drawPie((X[0,0],X[1,0]), (0.,35.), (b[0],b[1]), color='black',use_radian=True) 
        vibes.drawCircle(X[0,0], X[1,0], 1, 'blue[black]')           
        vibes.axisEqual()                    
        vibes.endDrawing()
           
##################################################################################################
############################     Gather in areas      ############################################



        cv2.namedWindow("Test", cv2.WINDOW_NORMAL)
        
        map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
        map, possible_positions =  compute_gathered_positions(map, inner_boxes, field, pos_wanted_accuracy)
        
        
        cv2.imshow("Test", cv2.flip(map, 0))
        
        key = cv2.waitKey(int(1000*slowing_ratio*dt))
        if key & 0xFF == 27:    
            cv2.destroyAllWindows()
            break
        elif  key & 0xFF == 32: 
            pause(0.5)
            key = cv2.waitKey(1)
            while key & 0xFF != 32:    
                pause(0.5)
                key = cv2.waitKey(1)



##################################################################################################
################################     Evolution      ##############################################

        # command
        u = array([[0.], [0.02]])  #(speed_command, rotation_command)

        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        # end display
#        pause(slowing_ratio* dt)





