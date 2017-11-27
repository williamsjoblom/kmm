#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

BTN_PIN = 11
global auto_mode = False;

def auto_mode_callback(data):
    btn_state = data.data


if __name__ == '__main__':
    rospy.init_node('kmm_input', anonymous=True)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BTN_PIN, GPIO.IN)

    set_auto_mode = rospy.ServiceProxy('set_auto_mode', SetBool)
    rospy.Subscriber("auto_mode", Bool, auto_mode_callback);

    try:
        rate = rospy.Rate(60) # 60Hz
        pressed = False

        while not rospy.is_shutdown():
            if GPIO.input(BTN_PIN) and not pressed:
                rospy.wait_for_service('set_auto_mode')

                try:
                    set_auto_mode(not btn_state)
                except rospy.ServiceException, e:
                    print "Service call set_auto_mode failed in kmm_input: %s"%e

            elif not GPIO.input(BTN_PIN):
                pressed = False;

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup(BTN_PIN)
