#!/usr/bin/env python2
############################################################
# Localisation simulation1: 
#     mobile: tank style
#     landmarks: 2
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#     noise: no
#     limited range: no
# Remarks:
############################################################     


##########################  IMPORTS   ############################
import rospy

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('brave_2020_localisation')
sys.path.append(pkg+'/src/Localisation')
from roblib import *



#####################  SIMULATION SETUP   ########################
# Time step
dt = 0.7

# Position of the landmarks
landmarks = [array([[15], [15]]), array([[5], [15]])]

# Initial state of the boat
Xinit = array([[6], [0], [pi/2+0.5], [1]])



##########################  FUNCTIONS   ############################

# Evolution equation
def evolX(X, u):
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX

def getBearing(X, landmark):
    x, y = X[0, 0], X[1, 0]
    xl, yl = landmark[0, 0], landmark[1, 0]
    
    return sawtooth(atan2(yl-y, xl-x) - X[2,0])


def approxPos2(heading, bearing1, bearing2, landmarks):
    # Case with two landmarks and a compass (Mobile Robotics, L.Jaulin, p.130-131)
    A = array([[sin(heading+bearing1), -cos(heading+bearing1)],
               [sin(heading+bearing2), -cos(heading+bearing2)]])
    
    B = array([[landmarks[0][0,0]*sin(heading+bearing1) - landmarks[0][1,0]*cos(heading+bearing1)],
               [landmarks[1][0,0]*sin(heading+bearing2) - landmarks[1][1,0]*cos(heading+bearing2)]])
    
    
    if det(A) != 0:
        return matmul(inv(A), B)
    else:
        return None
    
    
####################################################################




# Figure initialisation
ax = init_figure(0, 20, 0, 20)
    

# Initialising variables
X = Xinit
headingPrev, bearingPrev = 0, 0

# Main loop
if __name__ == "__main__":
    
    for t in arange(0, 15, dt):
        # displaying real elements
        draw_sailboat(X, 0.5, 0, 0, 0)
        plot(landmarks[0][0], landmarks[0][1], marker = '.', color = 'black')
        plot(landmarks[1][0], landmarks[1][1], marker = '.', color = 'black')
        
        # measures
        bearing1 = getBearing(X ,landmarks[0])
        bearing2 = getBearing(X ,landmarks[1])
        plot([X[0], landmarks[0][0]], [X[1], landmarks[0][1]], color = 'green')
        plot([X[0], landmarks[1][0]], [X[1], landmarks[1][1]], color = 'green')
        text(X[0]+10*cos(X[2]+bearing1)/2, X[1]+10*sin(X[2]+bearing1)/2, "Bearing = "+str(bearing1))
        text(X[0]+10*cos(X[2]+bearing2)/2, X[1]+10*sin(X[2]+bearing2)/2, "Bearing = "+str(bearing2))
        
        
        # localisation
        Xhat = approxPos2(X[2,0], bearing1, bearing2, landmarks)
        if Xhat is not None:
            draw_disk(Xhat,2,ax,'red')
        
        # command
        u = array([[0], [-0.15]])
        
        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        # end display
        pause(dt)
        clear(ax)



















    
    
