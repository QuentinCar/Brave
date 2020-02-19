#!/usr/bin/env python2
############################################################
# Localisation simulation10: 
#     mobile: tank style
#     landmarks: any (not taken into account here)
#     measures: 
#         speed of the boat (to be estimated in a next version)
#         heading of the boat
#     noise: yes
#     limited range: yes
# Remarks: Dynamic computing: considers (only) knowledge from t(k-1) to compute position at t(k). Marks not considered.
############################################################     

from vibes import *
from pyibex import *
from pyibex.geometry import SepPolarXY

from roblib import *

from numpy import pi, array, asarray, zeros, ones, uint8, arange
from math import factorial
import random
from itertools import permutations
from time import time
import cv2


if __name__ == "__main__":
##################################################################################################
#########################         Setup section          #########################################

    # Field of research
    field_x_low, field_x_high = 0, 130
    field_y_low, field_y_high = 0, 130
    field = IntervalVector([[field_x_low, field_x_high], [field_y_low, field_y_high]])
    
    
    # Estimated specification of sensors
    azimuth_accuracy = 5 *pi/180     # Compass + camera accuracy to compute the azimuth of a buoy
    range_of_vision = 15              # maximum distance for mark detection
    angle_of_perception = 50 *pi/180   # angle of perception of the camera
    speed_accuracy = 0.5              # accuracy on speed measurement. Will neeed to be deleted !!!
    
    
    # Wanted accuracy on position
    pos_wanted_accuracy = 0.3
    
    # Time step
    dt = 0.7              # Integration step
    slowing_ratio = 1.5
    
    # Noise that represents unpredictable variations of positions due to the low accuracy of the model used here 
    move_noise = 0*1*dt
    
    # Landmarks
    landmarks = [[35, 70], [75,95], [25,120]]
#    landmarks = [[15, 15], [5, 15], [2, 29], [13, 17], [20, 20]]
    
    nb_marks = len(landmarks)
    
    # Initial state of the boat
    Xinit = array([[30], [30], [1], [2]])
#    Xinit = array([[20], [0], [pi/2], [1]])




##################################################################################################
################################  SIMULATION FUNCTIONS   #########################################

# Evolution equation
def evolX(X, u):
    """Returns the derivative of X."""
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX


##################################################################################################
##################################  Simulation    ################################################
##################################################################################################



# Main loop
if __name__ == "__main__":

    # Initialising variables
    X = Xinit
    X_boxes = [IntervalVector([[X[0,0]-1, X[0,0]+1], [X[1,0]-1, X[1,0]+1]]), IntervalVector(2,[60,61])]
    cv2.namedWindow("Test", cv2.WINDOW_NORMAL)
    
    
    ## Drawing
    vibes.beginDrawing()
    vibes.newFigure("Localization")
    vibes.setFigureProperties({'x':800, 'y':100, 'width':800, 'height': 800})        
    vibes.axisLimits(field_x_low, field_x_high, field_y_low, field_y_high)   


    for t in arange(0, 30, dt):
        
##################################################################################################
############################    Optional features     ############################################    

        ## Drawing   
        vibes.clearFigure()
        scale = (field_y_high-field_y_low)/100.  
        for X_box in X_boxes:
            vibes.drawBox(X_box[0][0], X_box[0][1], X_box[1][0], X_box[1][1], color='[blue]')
        vibes.drawCircle(X[0,0], X[1,0], 0.5*scale, 'blue[black]')  
        for m in landmarks:
            vibes.drawCircle(m[0], m[1], 1*scale, 'yellow[black]')  
            
            
##################################################################################################
################################     Estimation     ##############################################
        speed = Interval(X[3,0]).inflate(speed_accuracy)
        heading = Interval(X[2,0]).inflate(azimuth_accuracy)
        
        f_evol = Function("x[2]", "(x[0] + %f*%s*cos(%s), x[1] + %f*%s*sin(%s))"%(2*(dt, str(speed), str(heading))))        
        
        X_boxes = [f_evol.eval_vector(X_box).inflate(move_noise) for X_box in X_boxes]
        

##################################################################################################
################################     Evolution      ##############################################

        # command
        u = array([[0.], [0.05]])  #(speed_command, rotation_command)

        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        pause(slowing_ratio*dt)


vibes.endDrawing()


























