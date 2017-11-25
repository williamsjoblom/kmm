#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

BTN_PIN = 11
btn_state = False;

def callback(data):
    btn_state = data.data


if __name__ == '__main__':
    rospy.init_node('kmm_input', anonymous=True)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BTN_PIN, GPIO.IN)

    try:
        pub = rospy.Publisher('btn_state', Bool, queue_size=1)
        sub = rospy.Subscriber('btn_state', Bool, callback)
        rate = rospy.Rate(60) # 60Hz
        pressed = False

        while not rospy.is_shutdown():
            if GPIO.input(BTN_PIN) and not pressed:
                btn_state = not btn_state
                pressed = True;
            elif not GPIO.input(BTN_PIN):
                pressed = False;
            pub.publish(btn_state)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup(BTN_PIN)
