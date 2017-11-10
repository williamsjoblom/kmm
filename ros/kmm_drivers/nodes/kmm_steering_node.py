#!/usr/bin/env python

import os
import sys
import serial
import spidev
import rospy
import time
import struct
import RPi.GPIO as GPIO

from math import pi, sin, cos
from time import time, sleep
from geometry_msgs.msg import Twist

# CONSTANTS
MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.2
RESET_PIN = 31

def build_packet(cmd, speed_vector):
    buf = []

    # Magic 8-bit header
    buf.extend(struct.pack("<b", 42))

    # Data
    buf.extend(struct.pack("<h", cmd))
    for i in speed_vector:
        buf.extend(struct.pack("<h", i))

    # Magic 8-bit footer
    buf.extend(struct.pack("<b", 24))

    return bytearray(buf)

class SteeringDriver:
    def __init__(self):
        """
        Constructor.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(1, 1) #connection to spi device 0 CHIP SELECT 0
        self.spi.max_speed_hz = 100


    def subscribe(self):
        """
        Subscribe.
        """
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback)

        
    def reset(self):
        """
        Reset AVR and stepper drivers.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(RESET_PIN, GPIO.OUT)

        GPIO.output(RESET_PIN, 0)
        time.sleep(0.01)
        GPIO.output(RESET_PIN, 1)

        
    def callback(self, msg):
        """
        Callback.
        """
        # m/s
        x = (msg.linear.x / 100.0)
        y = (msg.linear.y / -100.0)
        # rad/s
        z = (msg.angular.z / -100.0)

        # meters
        R = 0.03
        L = 0.095

        # rad/s
        w0 = (1./R) * (-sin(pi/3.)*x + cos(pi/3.)*y + L*z)
        w1 = (1./R) * (0.*x + -1.*y + L*z)
        w2 = (1./R) * (sin(pi/3.)*x + cos(pi/3.)*y + L*z)

        # revolutions / 1000s
        speed = [w0, w1, w2]
        speed = [int(w * 1000. / (2. * pi)) for w in speed]
        cmd = 0

        rospy.loginfo("speed -> %d, %d, %d", speed[0], speed[1], speed[2])

        byte_buf = build_packet(cmd, speed)

        self.spi.xfer(byte_buf)

        
if __name__ == '__main__':
    rospy.init_node('kmm_steering')
    try:
        driver = SteeringDriver()
        
        driver.reset()
        driver.subscribe()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
