# from roblib import *
# import utm

import numpy as np
import utm

from numpy import cos, sin, arctan,  arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm


rho = 6371000
gamma_inf = pi/4
delta_rmax = 1
r,zeta,beta = 10, pi/4, pi/4

def sawtooth (x):
    """Deal with 2*PI modulo
    
    Input:
    ------
    x: rad 
    """
    return (x+pi)%(2*pi)-pi   


def deg2rad(x):
	return x*2*pi/360

def wind_direction2psi(wind_direction):
	"""
	input : wind direction in degres, 0 in pointing north
			clockwise rotation
	return : angle of the wind in rad, in trigonometric circle
	"""
	wind_direction = deg2rad(wind_direction)
	psi = sawtooth(pi/2 - wind_direction)
	return psi

def control(lxa, lya, lxb, lyb, lxm, lym):
    a = gps2geographic(lxa, lya)
    b = gps2geographic(lxb, lyb)
    m = gps2geographic(lxm, lym)
    theta = 0 #cap robot
    e = det(hstack((b-a,m-a)))/norm(b-a)

    print(e)
    # q=1
    # if abs(e) > r:
    #     q = sign(e)

    # phi = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
    # theta_bar = phi-arctan(e/r)

    # if (cos(self.psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(self.psi-phi) + cos(zeta) < 0)):
    #     theta_bar = pi + self.psi - self.q*zeta

    # delta_r = (delta_rmax/pi)*self.sawtooth(theta-theta_bar)
    # delta_smax = (pi/2)*( (cos(self.psi-theta_bar)+1)/2 )**(log(pi/2*beta)/log(2))

    return  0


def control2(lxa, lya, lxb, lyb, lxm, lym):
    a = gps2geographic(lxa, lya)
    b = gps2geographic(lxb, lyb)
    m = gps2geographic(lxm, lym)
    n = np.cross(a,b)/(norm(a)*norm(b))
    e = np.dot(m,n.T)

    print(e)
    theta = 0
    q = 1
    if abs(e) > r/2 :
    	q = sign(e)

    M = array([ [-sin(lxm), cos(lxm), 0],
                [-cos(lxm)*sin(lym), -sin(lxm)*sin(lym), cos(lym)] ])

    P = np.dot(M,(b-a).T)
    phi = arctan2(P[1,0], P[0,0])
    theta_bar = phi - (2*gamma_inf/pi)*arctan(e/r)

    if (cos(psi-theta_bar) + cos(zeta) < 0) or ( (abs(e)<r) and (cos(psi-phi) + cos(zeta) < 0)):
        theta_bar = pi + psi - q*zeta

    if cos(theta-theta_bar) >= 0 :
        delta_r = delta_rmax*sin(theta-theta_bar)
    else :
        delta_r = delta_rmax*sign(sin(theta-theta_bar))

    delta_smax = (pi/2)*( (cos(psi-theta_bar)+1)/2 )**(log(pi/2*beta)/log(2))

    return delta_r, delta_smax

def gps2geographic(lx, ly): #lx : longitude, ly : latitude
    return array([[rho*cos(ly)*cos(lx), rho*cos(ly)*sin(lx), rho*sin(ly)]])
    # return array([   [rho*cos(ly)*cos(lx)],
    # 			     [rho*cos(ly)*sin(lx)],
    # 			     [rho*sin(ly)]         ])

# def gps2geographic(lx, ly): #lx : longitude, ly : latitude

#     (EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER) = utm.from_latlon(ly, lx)
#     # return array([[rho*cos(ly)*cos(lx), rho*cos(ly)*sin(lx), rho*sin(ly)]])
#     return array([[EASTING], [NORTHING]])

if __name__ == "__main__" :
	# psi = wind_direction2psi(60)
	# delta_r, delta_smax = control2(1,2,3,4,5,6)
	# print(delta_r, delta_smax)
	psi = wind_direction2psi(271)
	# print(psi)


	l1,l2 = 48.198562, -3.015039

	l3,l4 = 48.198608, -3.016351 #gauche

	l5,l6 = 48.199377, -3.014854 #haut

	control2(l2,l1,l6,l5,l3,l2)
