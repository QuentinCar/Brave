#!/usr/bin/env python2
############################################################
# Localisation simulation1: 
#     mobile: tank style
#     landmarks: >=2 (but only 2 at a time)
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed of the boat
#     noise: no, but possible
#     limited range: yes
# Remarks: noise matrices not accurate
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
from simu3 import inRange


global test 
test= 0
#####################  SIMULATION SETUP   ########################
# Time step
dt = 0.7

# Position of the landmarks
landmarks = [array([[15], [15]]), 
             array([[5], [15]]), 
             array([[13], [17]]),
             array([[2], [30]]),
             array([[20], [20]])]

#landmarks = [array([[5], [25]])]

# Initial state of the boat
Xinit = array([[20], [0], [pi/2], [1]])
#Xinit = array([[20], [0], [170*pi/180], [1]])

# Noise matrices
Galpha = diag([0.1, 0.1, 5*pi/180, 0.1])**2  #noise on evolution
gbeta1 = 2
gbeta2 = 0.5
####################################################################




##########################  FUNCTIONS   ############################
#available at https://www.ensta-bretagne.fr/jaulin/roblib.py 

def kalman_predict(xup,Gup,u,GammaAlpha,A):
    Gamma1 = matmul( matmul(A, Gup), A.T) + GammaAlpha
    x1 = matmul(A, xup) + u    
    return(x1,Gamma1)    

def kalman_correc(x0,Gamma0,y,GammaBeta,C):
    S = matmul(matmul(C, Gamma0), C.T) + GammaBeta        
    K = matmul(matmul(Gamma0, C.T), inv(S))           
    ytilde = y - matmul(C, x0)        
    Gup = sqrtm(matmul( matmul((eye(len(x0)) - matmul(K, C)), Gamma0) , (matmul((eye(len(x0)) - matmul(K, C)), Gamma0)).T)) 
    xup = x0 + matmul(K, ytilde)
    return(xup,Gup) 


# Evolution matrix
def findA(Xhat):
    A1 = array([[0, 0, 0, cos(Xhat[2,0])],
                [0, 0, 0, sin(Xhat[2,0])],
                [0, 0, 0,     0      ],
                [0, 0, 0,     0      ]])
                
    A = eye(4) + A1*dt
    return A    

# Command matrix
def findU(u):
    return dt*array([[0], [0], [u[1,0]], [u[0,0]]])

# Measurement matrix
def findC(detection_state, Xhat, bearings, heading_next = None, bearing_prev = None):
    if detection_state == 0:
        return None
    
    elif detection_state == 1:
        C = array([[1, 0, 0, 0],
                   [0, 1, 0, 0]])
        C = vstack((C, array([[0,0,1,0]])))  # Corresponds to heading measurement
        C = vstack((C, array([[0,0,0,1]])))  # Hypothesis: speed is quite constant (we cannot measure it)
        return C
    
    else:
        heading = Xhat[2,0]        
        C = array([[sin(heading+spot[1]), - cos(heading+spot[1]), 0, 0] for spot in bearings])
        C = vstack((C, array([[0,0,1,0]])))  # Corresponds to heading measurement
        return C
    

def findY(Xhat, bearings, heading_next = None, bearing_prev = None):
    global test
    # bearings = [(landmark, bearing), (...), (...)]
    # landmark = array([[xl], [yl]])
    # bearing = float (angle in radians)
    
    if len(bearings) == 0 or (len(bearings) == 1 and (heading_next is None or bearing_prev is None)):
        return None, 0
    
    elif len(bearings) == 1 and heading_next is not None and bearing_prev is not None:
        heading = Xhat[2,0]
        speed = Xhat[3,0]
        bearing = bearings[0][1]
        landmark = bearings[0][0]
        dBearing = (bearing-bearing_prev)/dt   # to be replaced by IMU
        dHeading = (heading_next-heading)/dt
        
        if abs(dBearing) <= 1.2*abs(dHeading):
            return None, 0
        
        tmp1 = array([[sin(heading+bearing), cos(heading+bearing)],
                   [-cos(heading+bearing), sin(heading+bearing)]])
        tmp2 = array([[-landmark[1, 0], landmark[0, 0]],
                   [landmark[0, 0] + speed*sin(heading)/(dHeading+dBearing),
                   landmark[1, 0] - speed*cos(heading)/(dHeading+dBearing)]])
        tmp3 = array([[cos(heading+bearing)], [sin(heading+bearing)]])
        
        Y = matmul(matmul(tmp1, tmp2), tmp3) 
        Y = vstack((Y, X[2,0]))   # Assuming we can measure the heading
        Y = vstack((Y, Xhat[3,0])) # We will assume that speed is relatively constant as we cannot measure it.
        
        print("Landmark spotted")
        print("Derivees ", dHeading, dBearing)
        print("Position ", Y)
        if test == 40:
            sys.exit()
        test += 1
        
        return Y, 1

    else:
        heading = Xhat[2,0]        
        Y = array([[spot[0][0,0]*sin(heading+spot[1]) - spot[0][1,0]*cos(heading+spot[1])] for spot in bearings])
        Y = vstack((Y, X[2,0]))
        return Y, 2


def findGbeta(detection_state, bearings, heading_next = None, bearing_prev = None):
    if detection_state == 0:
        return None
    # 1 landmark spotted
    elif detection_state == 1:
        Gbeta = diag([gbeta1**2, gbeta1**2, 0.001**2, 0.2**2]) # errors on [x, y, heading_measured, speed_hypothesis]
        return Gbeta
    # 2 or more landmarks spotted
    else:
        coefs = [gbeta2**2]*len(bearings)
        coefs.append(0.001**2)
        Gbeta = diag(coefs)
        return Gbeta
    
    
####################################################################






# Figure initialisation
ax = init_figure(0, 30, 0, 30)
    

# Initialising variables
X = Xinit
Xhat = Xinit
Gx = zeros((4,4))
#Xhat = zeros((4,1))
#Gx = diag([1,1,1,1])*10**6

Xnext,Gnext = Xhat, Gx
heading_next, bearing_prev = None, None

# Main loop
if __name__ == "__main__":
    
    for t in arange(0, 40, dt):
        # displaying real elements
        draw_sailboat(X, 0.5, 0, 0, 0)
        for landmark in landmarks:
            plot(landmark[0], landmark[1], marker = '.', color = 'black')
        
        # look for landmarks
        spotted = inRange(X, landmarks)
        
        # measures
        noiseB = 0*float(mvnrnd1(5*pi/180))  #Bruit en angle percu
        bearings = [(landmark,getBearing(X ,landmark)+noiseB) for landmark in spotted]
        for bearing in bearings:
            plot([X[0], bearing[0][0]], [X[1], bearing[0][1]], color = 'green')
            text((X[0]+bearing[0][0])/2, (X[1]+bearing[0][1])/2, "Bearing = "+str(bearing[1]))



        ### localisation
        print("### CORRECTION  ###")
#        print("Real X = ", X[:2])
        
        # correction
        Y, detection_state = findY(Xhat, bearings, heading_next, bearing_prev)
        Gbeta = findGbeta(detection_state, bearings, heading_next, bearing_prev)
        C = findC(detection_state, Xhat, bearings, heading_next, bearing_prev)
        if len(bearings) == 0 or (len(bearings) == 1 and (Y is None or Gbeta is None or C is None)):
            Xhat, Gx = Xnext, Gnext
        else:
            Xhat, Gx = kalman_correc(Xnext, Gnext, Y, Gbeta, C)
        
        draw_ellipse(Xhat[0:2],Gx[0:2,0:2],0.95,ax,[0,0,0.6])
        print("Detection state = ", detection_state)
        print('#### Comparison  ###')
        print(X[0,0], Xhat[0,0])
        print(X[1,0], Xhat[1,0])
        print(X[2,0], Xhat[2,0])
        print(X[3,0], Xhat[3,0])
        print('####################')

#        print('###############')
#        print(Gx)
#        print('###############\n')
        
        # command
        u = array([[0], [0.05]])
#        u = array([[0], [-0.06]])
        
        pause(0.5)
        print("### PREDICTION  ###")
        
        # prediction
        U = findU(u)
        A = findA(Xhat)
        Xnext, Gnext = kalman_predict(Xhat,Gx,U,Galpha,A)
        draw_ellipse(Xnext[0:2],Gnext[0:2,0:2],0.95,ax,[0.6,0,0])
        
        # Store data to compute angular velocities
        if len(bearings) == 1:
            heading_next = Xnext[2,0]
            bearing_prev = bearings[0][1]
        else:
            heading_next = None
            bearing_prev = None
        
        
        # evolution
        dX = evolX(X, u)        
        X = X + dt*dX
        X[2] = sawtooth(X[2])
        
        # end display
#        pause(dt)
        pause(0.5)
        clear(ax)



















    
    
