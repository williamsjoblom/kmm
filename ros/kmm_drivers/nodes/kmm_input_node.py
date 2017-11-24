#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

BTN_PIN = 11

if __name__ == '__main__':
    rospy.init_node('kmm_input', anonymous=True)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BTN_PIN, GPIO.IN)

    try:
        pub = rospy.Publisher('btn_state', Bool, queue_size=1)
        rate = rospy.Rate(60) # 60Hz

        last_state = GPIO.input(BTN_PIN)
        while not rospy.is_shutdown():
            state = GPIO.input(BTN_PIN)

            if state != last_state:
                pub.publish(Bool(state))

            last_state = state
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup(BTN_PIN)
