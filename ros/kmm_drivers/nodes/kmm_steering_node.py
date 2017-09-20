#!/usr/bin/env python

import os
import sys
import serial
import rospy
import time
import struct
from math import pi, sin, cos
from geometry_msgs.msg import Twist

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
        device = rospy.get_param("~device")

        if not os.path.exists(device):
            print("'" + device + "' port not found!")
            sys.exit(1)

        self.s = serial.Serial(port=device, baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS)

        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback)

    def callback(self, msg):

        # m/s
        x = msg.linear.x
        y = msg.linear.y
        # rad/s
        z = msg.angular.z
        # m
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

        self.s.write(byte_buf)

        """
        # Wait for response
        time.sleep(0.1)

        out = []
        while self.s.inWaiting() > 0:
            out += self.s.read(1)
            time.sleep(0.001)

        out_str = "".join([chr(c) for c in out])
        rospy.loginfo("response -> %s", out_str)
        """

if __name__ == '__main__':
    rospy.init_node('kmm_steering')
    try:
        steering_driver = SteeringDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
