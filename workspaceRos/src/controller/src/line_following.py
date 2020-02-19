#!/usr/bin/env python

import rospy
import numpy as np
import utm

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm

from controller.msg import Command
from controller.msg import Waypoints
from controller.msg import Gps
from controller.msg import Compass
from controller.msg import Meteo
from controller.msg import Wind


r,zeta = 10, pi/4
delta_rmax = pi/3
EARTH_RADIUS = 6371000.
# Origine repere Guerledan
#lat0, lon0 = (48.198427, -3.014750) 

#Origine repere Ty Colo
lat0, lon0 = (48.431775, -4.615529)


class Controller():


	def __init__(self,rosrate=10):
		self.pub_pwm = rospy.Publisher('/Command', Command, queue_size=10)

		rospy.Subscriber('/ublox/GPRMC', Gps, self._callback_gps)
		rospy.Subscriber('/ublox/HCHDG', Compass, self._callback_compass)
		rospy.Subscriber('/ublox/WIMDA', Meteo, self._callback_meteo)
		rospy.Subscriber('/ublox/WIMWV', Wind, self._callback_wind)
	   
		# rospy.Subscriber('/Waypoints', Waypoints, self._callback_waypoints)

		self.rate = rospy.Rate(rosrate)

		self.pwm_min_rudder = 900
		self.pwm_min_main_sail = 900
		self.pwm_min_fore_sail = 920

		self.pwm_mid_rudder = 1250 
		self.pwm_mid_main_sail = 1180
		self.pwm_mid_fore_sail = 1460    

		self.pwm_max_rudder = 1600
		self.pwm_max_main_sail = 1460
		self.pwm_max_fore_sail = 1710 

		self.q = 0

		self.lxa, self.lya = -3.013613, 48.198712
		self.lxb, self.lyb = -3.016612, 48.199009 
		self.lxm, self.lym = 0., 0.
		self.heading = 0.
		self.psi = 0.
		self.lat_str = ''
		self.lon_str = ''

		self.FirstLine = True
		self.pt1, self.pt2 = array([]), array([])


	def _callback_gps(self, msg):
		"""
		float64 timeStamp
		string validation
		float64 latitude
		string latitude_indic
		float64 longitude
		string longitude_indic
		float64 speed
		float64 heading
		float64 date
		float64 magnetic_declination
		string declination_indic
		string positionning_mode
		"""

		if msg.latitude_indic == "N":
			lat  = 1
		else: 
			lat = -1
		if msg.longitude_indic == "W":
			lon = -1
		else:
			lon = 1

		self.lat_str = str(msg.latitude)
		lat_str = self.lat_str.split(".")
		self.lon_str = str(msg.longitude)
		lon_str = self.lon_str.split(".")

		#conv degres.minute _> degres.decimal
		self.lym = lat*(float(lat_str[0][0:len(lat_str[0])-2]) + float(lat_str[0][len(lat_str[0])-2:])/60. + float(lat_str[1])/10**len(lat_str[1])/60.)
		self.lxm = lon*(float(lon_str[0][0:len(lon_str[0])-2]) + float(lon_str[0][len(lon_str[0])-2:])/60. + float(lon_str[1])/10**len(lon_str[1])/60.)


	def _callback_compass(self, msg):
		"""
		float64 heading
		string heading_indic
		float64 magnetic_declination
		"""

		self.heading = self.north2east( msg.heading )  


	def _callback_meteo(self, msg):
		"""
		float64 barometric_pressure_mercury
		float64 barometric_pressure_bars
		float64 temperature
		float64 true_wind_direction #angle of the wind en degre par rapport au nord dans le sens horaire
		float64 magnetic_wind_direction0.
		float64 wind_speed
		"""

		self.psi = self.north2east( msg.true_wind_direction )  


	def _callback_wind(self, msg):
		"""
		float64 wind_direction
		string reference
		float64 wind_speed
		string wind_speed_units
		string status
		"""
		#self.psi = self.north2east( msg.wind_direction ) 


	# def _callback_waypoints(self, msg):
		"""
		Non fonctionnel
		"""
	#     self.lxa, self.lya = msg.lxa, msg.lya
	#     self.lxb, self.lyb = msg.lxb, msg.lyb
		  

	def control(self, a, b, m):
		"""
		Input: cartesian coordinates of points a and b to follow line a-b
			cartesian coordinates of the boat, m.

		Return: rudder angle in rad delta_r
				sail angle in rad delta_s

		"""

		################ Controle Rudder ##############
		#------------- Controle en cap ----------------
		theta = self.heading #cap robot
		phi = arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
		cap = self.sawtooth(theta-phi)

		delta_r = -(delta_rmax/pi)*cap #de -pi/3 a pi/3


		#------------ Ajout ecart a la ligne --------
		ke=0.5
		e = det(hstack((b-a,m-a)))/norm(b-a)
		thetaBar = phi - ke*arctan(e/r)
		print("Ecart ligne : ", e)

				

		#----------- Ajout "remonte au vent" ---------
		#zeta = angle a partir du quel on considere qu'on remonte au vent (pi/4 un peu fort --- pi/6 mieux ?)
		if abs(e) > r:# on est en dehors du chenal
			self.q = np.sign(e) #de quel cote on est de la ligne

		#si mon angle vent - angle boat < zeta
		if cos(self.psi-thetaBar) + cos(zeta) < 0:
			print("Je remonte le vent")
			thetaBar = pi + self.psi - self.q*zeta

		
		
		delta_r = (delta_rmax/pi) * self.sawtooth(thetaBar-theta)#de -pi/3 a pi/3
		
		############## Controle des voiles ############
		delta_s = pi/2 * ((cos(self.psi-theta) + 1)/2)

		

		return delta_r, delta_s


	def sawtooth (self, x):
		"""Deal with 2*PI modulo
		
		Input:
		------
		x: rad 
		"""
		return (x+pi)%(2*pi)-pi   


	def deg2rad(self, x):
		"""
		Conversion deg to rad
		"""
		return x*2*pi/360

	def north2east(self, x):
		"""
		Input:
		------
		Wind direction in degres, 0 is pointing north
		clockwise rotation

		Return:
		-------
		Angle of the wind in rad, in trigonometric circle
		"""

		x = self.deg2rad(x)
		return self.sawtooth(pi/2 - x)

	def WGS84_to_cart(self, lat, lon):
		"""
		Input: gps coord decimal lat, lon
		Return: cartesian coord x, y with (lat0, lon0) the origin
		"""
		x = (pi/180.)*EARTH_RADIUS*(lon-lon0)*cos((pi/180.)*lat)
		y = (pi/180.)*EARTH_RADIUS*(lat-lat0)
		return x, y

	def cart_to_WGS84(self, x, y):
		"""
		Input: cartesian coord x, y with (lat0, lon0) the origin
		Return: gps coord decimal lat, lon
		"""
		EPSILON=0.00000000001
		lat = y*180./pi/EARTH_RADIUS+lat0
		if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
			lon = 0
		else:
			lon = (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+lon0
		return lat, lon



	def rad2pwm(self, x, sail_name):
		"""
		Bijection de [0, pi/2] sur [pwm_min, pwm_max] pour les voiles.
		[-pi/3,pi/3] pour rudder
		"""
		if sail_name == "main":
			return (2/pi)*(self.pwm_max_main_sail - self.pwm_min_main_sail)*x + self.pwm_min_main_sail
		elif sail_name == "fore":
			return (2/pi)*(self.pwm_max_fore_sail - self.pwm_min_fore_sail)*x + self.pwm_min_fore_sail    
		elif sail_name == "rudder":
			x = x+pi/3
			return (3/(2*pi))*(self.pwm_max_rudder - self.pwm_min_rudder)*x + self.pwm_min_rudder



	def main(self):
		lxa, lya = -3.015067, 48.198905
		lxb, lyb = -3.015603, 48.198301
		lxc, lyc = -3.016049, 48.198762

		xa, ya = self.WGS84_to_cart(lya, lxa)
		xb, yb = self.WGS84_to_cart(lyb, lxb)
		xc, yc = self.WGS84_to_cart(lyc, lxc)

		xm, ym = self.WGS84_to_cart(self.lym, self.lxm)


		a = array([[xa],[ya]])
		b = array([[xb],[yb]])
		c = array([[xc],[yc]])

		m = array([[xm],[ym]])


		if abs(m[0]-b[0])<5 and abs(m[1]-b[1])<5:
			print("ligne bc")
			self.FirstLine = False
			self.pt1 = b
			self.pt2 = c

		elif abs(m[0]-c[0])<5 and abs(m[1]-c[1])<5:
			print("ligne ca")
			self.FirstLine = False
			self.pt1 = c
			self.pt2 = a

		if self.FirstLine:
			print("ligne ab")
			self.pt1 = a
			self.pt2 = b


		u1, u2 = self.control(self.pt1,self.pt2,m) 

		delta_rudder, delta_main_sail, delta_fore_sail = u1, u2, u2

		pwm_rudder = self.rad2pwm(-delta_rudder, "rudder")
		pwm_main_sail = self.rad2pwm(delta_main_sail, "main")
		pwm_fore_sail = self.rad2pwm(delta_fore_sail, "fore")

		pwm = Command()
		pwm.pwm_rudder = pwm_rudder
		pwm.pwm_main_sail = pwm_main_sail
		pwm.pwm_fore_sail = pwm_fore_sail

		self.pub_pwm.publish(pwm)




if __name__ == "__main__":
	rospy.init_node('controller', anonymous=True)
	controller = Controller()
	while not rospy.is_shutdown():
		controller.main()
		controller.rate.sleep()
