#!/usr/bin/env python2
############################################################
# Localisation simulation1: 
#     mobile: tank style
#     landmarks: 1
#     measures: 
#         angle to landmark (bearing)
#         heading of the boat
#         speed of the boat
#     noise: no
#     limited range: no
# Remarks:
#    works while the derivative of bearing is high enough
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

# Position of the landmark
landmark = array([[15], [15]])

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


def approxPos1(speed, heading, dHeading, bearing, dBearing, landmark):
    # Case with one landmark, a compass and speed known (Mobile Robotics, L.Jaulin, p.131-132)
    A = array([[sin(heading+bearing), cos(heading+bearing)],
               [-cos(heading+bearing), sin(heading+bearing)]])
    
    B = array([[-landmark[1, 0], landmark[0, 0]],
               [landmark[0, 0] + speed*sin(heading)/(dHeading+dBearing), landmark[1, 0] - speed*cos(heading)/(dHeading+dBearing)]])
#    print(B)
    C = array([[cos(heading+bearing)], [sin(heading+bearing)]])
    
    return matmul(matmul(A,B), C)
    
    
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
        plot(landmark[0], landmark[1], marker = '.', color = 'black')
        
        # measures
        bearing = getBearing(X ,landmark)
        plot([X[0], landmark[0]], [X[1], landmark[1]], color = 'green')
        text(X[0]+10*cos(X[2]+bearing)/2, X[1]+10*sin(X[2]+bearing)/2, "Bearing = "+str(bearing))
        
        
        # localisation
        dBearing = (bearing - bearingPrev)/dt
        dHeading = (X[2,0] - headingPrev)/dt  #to be replaced by IMU
        bearingPrev, headingPrev = bearing, X[2,0]
        Xhat = approxPos1(X[3,0], X[2,0], dHeading, bearing, dBearing, landmark)
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



















    
    
