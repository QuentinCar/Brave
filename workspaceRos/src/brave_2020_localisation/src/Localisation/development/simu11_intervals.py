#!/usr/bin/env python2
############################################################
# Localisation simulation11: 
#     mobile: tank style
#     landmarks: any
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed
#     noise: yes
#     limited range: yes
# Remarks: Relies on (noisy) measure of speed. Would be better to get rid of it if possible.
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
    field_x_low, field_x_high = -60, 130
    field_y_low, field_y_high = 0, 230
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
    slowing_ratio = 0.5
    
    # Noise that represents unpredictable variations of positions due to the low accuracy of the model used here 
    move_noise = 0*1*dt    
    
    # Landmarks
    landmarks = [[35, 70], [75,95], [25,120]]
#    landmarks = [[15, 15], [5, 15], [2, 29], [13, 17], [20, 20]]
    
    nb_marks = len(landmarks)
    
    # Initial state of the boat
    Xinit = array([[30], [30], [pi/2], [2]])
#    Xinit = array([[20], [0], [pi/2], [1]])


##################################################################################################
#########################   IA computation functions     #########################################

def compute_boxes(P, anonymised_marks, anonymised_distances, anonymised_angles):
    # Set constraints 
    separators = []
    for i in range(len(anonymised_marks)):
        seps =[]
        n = len(anonymised_marks[i])
        for m,d,alpha in zip(anonymised_marks[i], anonymised_distances[n*i:n*(i+1)], anonymised_angles[n*i:n*(i+1)]):
            sep = SepPolarXY(d, alpha)
            fforw = Function("v1", "v2", "(%f-v1;%f-v2)" %(m[0], m[1]))
            fback = Function("p1", "p2", "(%f-p1;%f-p2)" %(m[0], m[1]))
            sep = SepTransform(sep, fback, fforw)
            seps.append(sep)
            
        sep = SepQInterProjF(seps)
        sep.q = 0      # How many measures can be considered as outliers. Here we want 3 correct measures.
        separators.append(sep)
    
    inner_boxes, outer_boxes, frontier_boxes = [], [], []
    for sep in separators:
        # Compute all possible positions. Factor 4 in accuracy is due to bissections effects.
        inner, outer, frontier = pySIVIA(field, sep, 4*pos_wanted_accuracy, draw_boxes = False)
        inner_boxes += inner
        outer_boxes += outer
        frontier_boxes += frontier    
    return inner_boxes, outer_boxes, frontier_boxes 



def compute_gathered_positions(map, inner_boxes, field, pos_wanted_accuracy):    
    # Create a binary map of the positions where the boat can be
    field_x_low, field_y_low = field[0][0], field[1][0]
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



def anonymise_measures(landmarks, azimuths, range_of_vision):
    anonymised_marks = list(permutations(landmarks, len(azimuths)))
    random.shuffle(anonymised_marks)
    anonymised_angles = azimuths*len(anonymised_marks)
    anonymised_distances = [Interval(0,range_of_vision)]*len(azimuths) *len(anonymised_marks)
    return anonymised_marks, anonymised_angles, anonymised_distances 


##################################################################################################
################################  SIMULATION FUNCTIONS   #########################################

# Evolution equation
def evolX(X, u):
    """Returns the derivative of X."""
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX


def getAzimuths(X, landmarks, accuracy = pi/180*5):
    """Returns the bearing for the designated landmark."""
    x, y = X[0, 0], X[1, 0]
    azimuths = []
    for landmark in landmarks:
        xl, yl = landmark[0], landmark[1]        
        camera_measure = sawtooth(atan2(yl-y, xl-x) - X[2,0])
        compass_measure = X[2,0]
        azimuths.append(Interval(camera_measure+compass_measure-accuracy, camera_measure+compass_measure+accuracy))
    return azimuths


def inRange(X, landmarks, perceptionAngle, visionRange):
    """Returns the list of the landmarks that the boat is really able to see."""
    spotted = []
    for m in landmarks:
        dist = norm(X[:2]-asarray(m).reshape((2,1)))
        bearing = sawtooth(atan2(m[1]-X[1,0], m[0]-X[0,0]) - X[2,0])
        if dist < visionRange and np.abs(bearing) < perceptionAngle:
            spotted.append(m)            
    return spotted



##################################################################################################
##################################  Simulation    ################################################
##################################################################################################

## Drawing
vibes.beginDrawing()
vibes.newFigure("Localization")
vibes.setFigureProperties({'x':800, 'y':100, 'width':800, 'height': 800})        
vibes.axisLimits(field_x_low, field_x_high, field_y_low, field_y_high)   



# Main loop
if __name__ == "__main__":

    # Initialising variables
    X = Xinit
#    X_boxes = [IntervalVector([[X[0,0]-1, X[0,0]+1], [X[1,0]-1, X[1,0]+1]])
    X_boxes = [field]
#    cv2.namedWindow("Test", cv2.WINDOW_NORMAL)
    
    
    for t in arange(0, 300, dt):
        print
        print "########   "+str(t)+"  ########" 
        t0 = time()
##################################################################################################
############################  Simulate the measure    ############################################
        spotted = inRange(X, landmarks, angle_of_perception, range_of_vision)
        azimuths = getAzimuths(X, spotted, accuracy = azimuth_accuracy) 
        speed = Interval(X[3,0]).inflate(speed_accuracy)
        heading = Interval(X[2,0]).inflate(azimuth_accuracy)
        
##################################################################################################
######################### Compute all possible positions #########################################    

        
        if len(azimuths) != 0:            
            # when at least one mark is detected, improve position estimation
            anonymised_marks, anonymised_angles, anonymised_distances = anonymise_measures(landmarks, azimuths, range_of_vision)        
            inner_boxes, outer_boxes, frontier_boxes = compute_boxes(field, anonymised_marks, anonymised_distances, anonymised_angles)
            static_estimations = inner_boxes + frontier_boxes
        else:
            static_estimations = [field]

        f_evol = Function("x[2]", "(x[0] + %f*%s*cos(%s), x[1] + %f*%s*sin(%s))"%(2*(dt, str(speed), str(heading))))
        dynamic_estimations = [f_evol.eval_vector(X_box).inflate(move_noise) for X_box in X_boxes]
        
        # possible positions = field & static estimation & dynamic estimation
        X_boxes = []
        
        print "Static boxes: ", len(static_estimations)
        print "Dynamic boxes: ", len(dynamic_estimations)
        print
        
        if len(static_estimations) > 1: # meaning at least one mark is detected
            dynamic_union = dynamic_estimations[0]  # then we can afford to loose some precision on dynamics given
            for dynamic_box in dynamic_estimations[1:]: # that static estimations are better, in order to reduce computing time
                dynamic_union |= dynamic_box
        
            for static_box in static_estimations:
                intersection_box = field & dynamic_union & static_box
                if not intersection_box.is_empty():
                    X_boxes.append(intersection_box)
                    
        else: # In case no marks are detected, all estimations are computed based on system dynamics
            for dynamic_box in dynamic_estimations:
                intersection_box = field & dynamic_box
                if not intersection_box.is_empty():
                    X_boxes.append(intersection_box)
                
        t1 = time()
##################################################################################################
############################    Optional features     ############################################    

        ## Drawing   
        vibes.clearFigure()        
        scale = (field_y_high-field_y_low)/100. 
        for X_box in X_boxes:
            vibes.drawBox(X_box[0][0], X_box[0][1], X_box[1][0], X_box[1][1], color='[blue]')
        vibes.drawCircle(X[0,0], X[1,0], 1*scale, 'blue[black]') 
        for b in azimuths:
            vibes.drawPie((X[0,0],X[1,0]), (0.,range_of_vision), (b[0],b[1]), color='black',use_radian=True)             
        for m in landmarks:
            vibes.drawCircle(m[0], m[1], 1*scale, 'yellow[black]')  
        for m in spotted:
            vibes.drawCircle(m[0], m[1], 1*scale, 'yellow[blue]')  
           
##################################################################################################
############################     Gather in areas      ############################################
        t2 = time()
        
        map = zeros((int((field_y_high-field_y_low)/pos_wanted_accuracy), int((field_x_high-field_x_low)/pos_wanted_accuracy)), uint8)
        map, possible_positions = compute_gathered_positions(map, X_boxes, field, pos_wanted_accuracy)
        
        t3 = time()

##################################################################################################
############################    Optional features     ############################################    

        ## Performances assessment
        print "Temps de calcul: "
        print t1-t0
        print "Temps de rassemblement: "
        print t3-t2
        print "Positions estimees: "
        print asarray(possible_positions)
        
        ## Display
        cv2.imshow("Test", cv2.flip(map, 0))
        
        key = cv2.waitKey(int(1000*slowing_ratio*dt))
        if  key & 0xFF == 32: 
            pause(0.5)
            key = cv2.waitKey(1)
            while key & 0xFF != 32 and key & 0xFF != 27:    
                pause(0.2)
                key = cv2.waitKey(1)                
        if key & 0xFF == 27:    
            cv2.destroyAllWindows()
            break

##################################################################################################
################################     Evolution      ##############################################

        # command
        u = array([[0.], [-0.025]])  #(speed_command, rotation_command)

        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        


vibes.endDrawing()

