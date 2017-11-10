#!/usr/bin/env python

import os
import sys
import serial
import spidev
import rospy
import time
import struct
from math import pi, sin, cos
from time import time, sleep
from geometry_msgs.msg import Twist

# CONSTANTS
MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.2

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

def lowpass(x, y0, dt, T):
    return y0 + (x - y0) * (dt/(dt+T))

class SteeringDriver:
    def __init__(self):
        device = rospy.get_param("~device")

        if not os.path.exists(device):
            rospy.logerror("'%s' port not found!", device)
            sys.exit(1)

        # self.s = serial.Serial(port=device, baudrate=9600,
        #            parity=serial.PARITY_NONE,
        #            stopbits=serial.STOPBITS_ONE,
        #            bytesize=serial.EIGHTBITS)

        self.spi = spidev.SpiDev()
        self.spi.open(0,0) #connection to spi device 0 CHIP SELECT 0
        self.spi.max_speed_hz = 100

        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback)

        self.l_x = 0.0
        self.l_y = 0.0
        self.l_z = 0.0
        self.T = 0.2
        self.ts = rospy.Time.now()

    def callback(self, msg):

        # m/s
        x = (msg.linear.x / 100.0)
        y = (msg.linear.y / -100.0)
        # rad/s
        z = (msg.angular.z / -100.0)

        # Lowpass filtering
        dt = (rospy.Time.now() - ts).to_sec()
        ts = rospy.Time.now()

        l_x = lowpass(x, l_x, dt, T)
        l_y = lowpass(y, l_y, dt, T)
        l_z = lowpass(z, l_z, dt, T)

        # Adjustment to max linear an angular velocities
        x = l_x * MAX_LINEAR_VEL
        y = l_y * MAX_LINEAR_VEL
        z = l_z * MAX_ANGULAR_VEL
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

        # self.s.write(byte_buf)
        self.spi.xfer(byte_buf)

        """
        # Wait for response
        sleep(0.1)

        out = []
        while self.s.inWaiting() > 0:
            out += self.s.read(1)
            sleep(0.001)

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
