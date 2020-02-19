#!/usr/bin/env python2
############################################################
# Localisation simulation5: 
#     mobile: tank style
#     landmarks: >=2 (but only 2 at a time)
#     measures: 
#         angle to landmarks (bearings)
#         heading of the boat
#         speed of the boat
#     noise: yes   # to be added on evolution
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




#####################  SIMULATION SETUP   ########################
# Time step
dt = 0.7              # Integration step
slowing_ratio = 2     # Slow down or speed up simulation without affecting integration step dt.

# Position of the landmarks
landmarks = [array([[15], [15]]), 
             array([[5], [15]]), 
             array([[13], [17]]),
             array([[2], [30]]),
             array([[20], [20]])]

#landmarks = [array([[5], [25]])]

# Initial state of the boat
Xinit = array([[20], [0], [pi/2], [1]])

# Noise matrices
Galpha = diag([0.1, 0.1, 5*pi/180, 0.1])**2  #noise on evolution
gbeta1 = 2
gbeta2 = 0.5
####################################################################



####################  SIMULATION FUNCTIONS   #######################
# Evolution equation
def evolX(X, u):
    """Returns the derivative of X."""
    X, u = X.flatten(), u.flatten()
    x, y, theta, v = X[0], X[1], X[2], X[3]
    
    dX = array([[v*cos(theta)], [v*sin(theta)], [u[1]], [u[0]]])
    
    return dX


def getBearing(X, landmark):
    """Returns the bearing for the designated landmark."""
    x, y = X[0, 0], X[1, 0]
    xl, yl = landmark[0, 0], landmark[1, 0]
    
    return sawtooth(atan2(yl-y, xl-x) - X[2,0])


def inRange(X, landmarks):
    """Returns the list of the landmarks that the boat is really able to see."""
    perceptionAngle = 45  #degrees
    spotted = []
    for i in range(len(landmarks)):
        dist = norm(X[:2]-landmarks[i])
        if dist < 15 and abs(getBearing(X, landmarks[i])) < perceptionAngle*pi/180:
            spotted.append(landmarks[i])
    return spotted

### Info
# bearings = [(landmark, bearing), (...), (...)]
# landmark = array([[xl], [yl]])
# bearing = float (angle in radians)
###
####################################################################



######################  KALMAN FUNCTIONS   #########################
#available at https://www.ensta-bretagne.fr/jaulin/roblib.py 

def kalman_predict(xup,Gup,u,GammaAlpha,A):
    Gamma1 = matmul( matmul(A, Gup), A.T) + GammaAlpha
    x1 = matmul(A, xup) + u    
    return(x1,Gamma1)    

def kalman_correc(x0,Gamma0,y,GammaBeta,C):
    S = matmul(matmul(C, Gamma0), C.T) + GammaBeta        
    K = matmul(matmul(Gamma0, C.T), inv(S))           
    ytilde = y - matmul(C, x0)        
    Gup = sqrtm(matmul( matmul((eye(len(x0)) - matmul(K, C)), Gamma0) ,
               (matmul((eye(len(x0)) - matmul(K, C)), Gamma0)).T)) 
    xup = x0 + matmul(K, ytilde)
    return(xup,Gup) 

####################################################################



##################  LINEARISATION FUNCTIONS   ######################

### Approximated evolution matrix
def findA(Xhat):
    A1 = array([[0, 0, 0, cos(Xhat[2,0])],
                [0, 0, 0, sin(Xhat[2,0])],
                [0, 0, 0,     0      ],
                [0, 0, 0,     0      ]])
                
    A = eye(4) + A1*dt
    return A


### Command matrix
def findU(u):
    return dt*array([[0], [0], [u[1,0]], [u[0,0]]])


### Measures vector
def findY_1(Xhat, bearings, heading_next, bearing_prev):
    """Returns the vector of measures in the case in which 1 landmark is detected.
     Reminder:
     bearings = [(landmark, bearing), (...), (...)]
     landmark = array([[xl], [yl]])
     bearing = float (angle in radians)"""
    
    heading = Xhat[2,0]
    speed = Xhat[3,0]
    bearing = bearings[0][1]
    landmark = bearings[0][0]
    dBearing = (bearing-bearing_prev)/dt   # to be replaced by IMU
    dHeading = (heading_next-heading)/dt
    
    if abs(dBearing) <= 1.2*abs(dHeading): # If the boat moves close to the line between it and the landmark, 
        return None                        # the estimation will be bad or impossible. (Y could be adapted to represent that line)
    
    tmp1 = array([[sin(heading+bearing), cos(heading+bearing)],
               [-cos(heading+bearing), sin(heading+bearing)]])
    tmp2 = array([[-landmark[1, 0], landmark[0, 0]],
               [landmark[0, 0] + speed*sin(heading)/(dHeading+dBearing),
               landmark[1, 0] - speed*cos(heading)/(dHeading+dBearing)]])
    tmp3 = array([[cos(heading+bearing)], [sin(heading+bearing)]])
    
    Y = matmul(matmul(tmp1, tmp2), tmp3) 
    Y = vstack((Y, X[2,0]))   # Assuming we can measure the heading
    Y = vstack((Y, Xhat[3,0])) # We will assume that speed is relatively constant as we cannot measure it. 
    return Y

def findY_2(Xhat, bearings):
    """Returns the vector of measures in the case in which 2 landmarks or more are detected.
     Reminder:
     bearings = [(landmark, bearing), (...), (...)]
     landmark = array([[xl], [yl]])
     bearing = float (angle in radians)"""

    heading = Xhat[2,0]        
    Y = array([[spot[0][0,0]*sin(heading+spot[1]) - spot[0][1,0]*cos(heading+spot[1])] for spot in bearings])
    Y = vstack((Y, X[2,0]))
    return Y



### Measurement matrices
def findC_1(Xhat, bearings):
    """Returns C in the case in which 1 landmark is detected."""
    C = array([[1, 0, 0, 0],
               [0, 1, 0, 0]])
    C = vstack((C, array([[0,0,1,0]])))  # Corresponds to heading measurement
    C = vstack((C, array([[0,0,0,1]])))  # Hypothesis: speed is quite constant (we cannot measure it)
    return C
    

def findC_2(Xhat, bearings):
    """Returns C in the case in which 2 landmarks or more are detected."""
    heading = Xhat[2,0]        
    C = array([[sin(heading+spot[1]), - cos(heading+spot[1]), 0, 0] for spot in bearings])
    C = vstack((C, array([[0,0,1,0]])))  # Corresponds to heading measurement
    return C



### Noise on measures
def findGbeta_1():
    Gbeta = diag([gbeta1**2, gbeta1**2, 0.001**2, 0.2**2]) # errors on [x, y, heading_measured, speed_hypothesis]
    return Gbeta

def findGbeta_2(bearings):
    coefs = [gbeta2**2]*len(bearings)
    coefs.append(0.001**2)
    Gbeta = diag(coefs)
    return Gbeta

####################################################################





####################################################################
###########                MAIN                  ###################
####################################################################



# Figure initialisation
ax = init_figure(0, 30, 0, 30)
    

# Initialising variables
X = Xinit

Xhat = Xinit            # Assuming we know exactly the initial location of the boat
Gx = zeros((4,4))

#Xhat = zeros((4,1))    # Assuming we do not know anything at the beginning
#Gx = diag([1,1,1,1])*10**6

Xnext,Gnext = Xhat, Gx
heading_next, bearing_prev = None, None



# Main loop
if __name__ == "__main__":
    
    for t in arange(0, 30, dt):
        # displaying real elements
        draw_sailboat(X, 0.5, 0, 0, 0)
        for landmark in landmarks:
            plot(landmark[0], landmark[1], marker = '.', color = 'black')
        
        # look for landmarks
        spotted = inRange(X, landmarks)


        # measures
        noiseB = 3*pi/180
        bearings = [(landmark,getBearing(X ,landmark)+float(mvnrnd1(noiseB**2))) for landmark in spotted]
        for bearing in bearings:
            plot([X[0], bearing[0][0]], [X[1], bearing[0][1]], color = 'green')
            text((X[0]+bearing[0][0])/2, (X[1]+bearing[0][1])/2, "Bearing = "+str(bearing[1]))



        ### localisation
        print("### CORRECTION  ###")

        # Identifying how many landmarks can be used for localisation
        if len(bearings) >= 2:
            detection_state = 2
        
        elif len(bearings) == 1 and bearing_prev is not None:
            detection_state = 1
        
        else:
            detection_state = 0
            
        print("Detection_state ", detection_state)
        
        
        # Correction of the previously estimated position
        if detection_state == 0:           # No landmark detected: the position cannot be better estimated.
            Xhat, Gx = Xnext, Gnext
        
        elif detection_state == 1:         # One landmark can be used to improve estimation
            Y = findY_1(Xhat, bearings, heading_next, bearing_prev)
            if Y is None:                  # It is not possible to use the landmark (see findY_1)
                Xhat, Gx = Xnext, Gnext
            else:
                C = findC_1(Xhat, bearings)
                Gbeta = findGbeta_1()
                Xhat, Gx = kalman_correc(Xnext, Gnext, Y, Gbeta, C) # Improve estimation with available measure

        elif detection_state == 2:         # Multiple landmarks can be used to improve estimation
            Y = findY_2(Xhat, bearings)
            C = findC_2(Xhat, bearings)
            Gbeta = findGbeta_2(bearings)
            Xhat, Gx = kalman_correc(Xnext, Gnext, Y, Gbeta, C)     # Improve estimation with available measure


        draw_ellipse(Xhat[0:2],Gx[0:2,0:2],0.95,ax,[0,0,0.6])  # Draw uncertainty matrices


        # command
        u = array([[0], [0.05]])

        pause(slowing_ratio* (2*dt/3))
        
        # prediction
        print("### PREDICTION  ###")        
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
        pause(slowing_ratio* (dt/3))
        clear(ax)







































