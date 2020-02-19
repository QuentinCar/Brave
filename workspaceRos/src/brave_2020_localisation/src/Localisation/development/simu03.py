#!/usr/bin/env python2
############################################################
# Localisation simulation1: 
#     mobile: tank style
#     landmarks: >=2 (but only 2 at a time)
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed of the boat
#     noise: no
#     limited range: yes
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
from simu1 import evolX, getBearing, approxPos1
from simu2 import approxPos2


#####################  SIMULATION SETUP   ########################
# Time step
dt = 0.7

# Position of the landmarks
landmarks = [array([[15], [15]]), 
             array([[5], [15]]), 
             array([[13], [17]]),
             array([[2], [30]]),
             array([[20], [4]])]

# Initial state of the boat
Xinit = array([[10], [0], [pi/2+0.5], [1]])



##########################  FUNCTIONS   ############################
def inRange(X, landmarks):
    spotted = []
    for i in range(len(landmarks)):
        dist = norm(X[:2]-landmarks[i])
        if dist < 15 and abs(getBearing(X, landmarks[i])) < 45*pi/180:
            spotted.append(landmarks[i])
    return spotted


####################################################################


# Figure initialisation
ax = init_figure(0, 30, 0, 30)
    

# Initialising variables
X = Xinit
headingPrev, bearingPrev = 0, 0

# Main loop
if __name__ == "__main__":
    
    for t in arange(0, 35, dt):
        # displaying real elements
        draw_sailboat(X, 0.5, 0, 0, 0)
        for landmark in landmarks:
            plot(landmark[0], landmark[1], marker = '.', color = 'black')
        
        # look for landmarks
        spotted = inRange(X, landmarks)
        
        # measures
        if len(spotted) == 0:
            pass
        # One landmark spotted: use it + speed + compass to assess position
        if len(spotted) == 1:
            bearing = getBearing(X ,spotted[0])            
            plot([X[0], spotted[0][0]], [X[1], spotted[0][1]], color = 'green')           
            text((X[0]+spotted[0][0])/2, (X[1]+spotted[0][1])/2, "Bearing = "+str(bearing))
         
        # two or more landmarks spotted: use the two best of them + compass to assess position
        if len(spotted) >= 2:
            bearings = [(landmark,getBearing(X ,landmark)) for landmark in spotted]
#            bearings = sorted(bearings, key = lambda x: abs(abs(x[1])-pi/2))
            bearing1, bearing2 = bearings[0], bearings[1]
            plot([X[0], bearing1[0][0]], [X[1], bearing1[0][1]], color = 'green')
            plot([X[0], bearing2[0][0]], [X[1], bearing2[0][1]], color = 'green')
            text((X[0]+bearing1[0][0])/2, (X[1]+bearing1[0][1])/2, "Bearing1 = "+str(bearing1[1]))
            text((X[0]+bearing2[0][0])/2, (X[1]+bearing2[0][1])/2, "Bearing2 = "+str(bearing2[1]))
        
        
        # localisation
        if len(spotted) == 1:
            dBearing = (bearing - bearingPrev)/dt
            dHeading = (X[2,0] - headingPrev)/dt  #to be replaced by IMU
            bearingPrev, headingPrev = bearing, X[2,0]
            Xhat = approxPos1(X[3,0], X[2,0], dHeading, bearing, dBearing, spotted[0])
            if Xhat is not None:
                draw_disk(Xhat,2,ax,'red')
                
        if len(spotted) >= 2:
            Xhat = approxPos2(X[2,0], bearing1[1], bearing2[1], [bearing1[0], bearing2[0]])
            if Xhat is not None:
                draw_disk(Xhat,2,ax,'red')
        
        
        # command
        u = array([[0], [-0.05]])
        
        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        # end display
        pause(dt)
        clear(ax)



















    
    
