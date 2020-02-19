#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from geometry_msgs.msg import Vector3

from numpy import pi, sign

from pynput import keyboard
from pynput.keyboard import Key

from time import time, sleep



def on_press(key):
    global rudder, sail, rudder_sensibility, sail_sensibility, timeLastMove, end
    ##################################################################################################
    # Control of connected sailboats.
    # Commands for rudder and sail. 
    ##################################################################################################

    if key in [Key.up, Key.down, Key.left, Key.right]:
        rudder += rudder_sensibility * (1 if key == Key.left else -1 if key == Key.right else 0)
        sail += sail_sensibility * (1 if key == Key.up else -1 if key == Key.down else 0)
        timeLastMove = time()




###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Variables
###################################################################

    global rudder_sensibility, sail_sensibility, rudder, sail, timeLastMove


###################################################################
#    keyBoardListener
###################################################################


    keyboardListener = keyboard.Listener(
        on_press=on_press)
    keyboardListener.start()
    end = False

###################################################################
#    initialisation of variables
###################################################################


    initRudder = 0.0
    initSail = 0.0
    rudder, sail = initRudder, initSail

    rudder_sensibility = 0.005

    sail_sensibility = 0.001

    timeLastMove = time()


###################################################################
#    Ros initialisation
###################################################################


    rospy.init_node('operator', anonymous=True)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubCommand = rospy.Publisher('commands', Vector3, queue_size = 2)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if time()-timeLastMove > 0.1:
            rudder, sail = initRudder, initSail

        commands = Vector3(x = rudder, y = sail)

        pubCommand.publish(commands)

        rate.sleep()


    end = True







if __name__ == "__main__":
    run()