#!/usr/bin/python

import os
import sys
import serial
import spidev
import rospy
import struct
import ctypes
import RPi.GPIO as GPIO

from time import time, sleep
from bitstring import Bits as B
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

# Values from adafruit c++ acc code to modify raw data.
ACC_MG_LSB  = 0.001
SENSORS_GRAVITY_STANDARD = 9.80665

# Values from adafruit c++ gyro code to modfy raw data.
GYRO_RANGE_250DPS = 0.00875
SENSORS_DPS_TO_RADS = 0.017453293

def reset():
  """
  Reset the sensor.
  """

  # Configure reset pins
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup([29, 31], GPIO.OUT)

  # Reset
  GPIO.output([29, 31], 0)
  sleep(0.1)
  GPIO.output([29, 31], 1)


if __name__ == "__main__":
  print("Starting session...")

  try:
    rospy.init_node("kmm_sensor")
    
    spi = spidev.SpiDev()
    spi.open(0, 0) #connection to spi device 0 CHIP SELECT 0
    spi.max_speed_hz = 100000
    spi.bits_per_word = 8
 
    imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)

    reset()

    idx = 0
    rawdata = []
    sensor_data = [0,0,0]

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
      idx = 0
      rawdata = spi.readbytes(6)
      for i in range(0,6,2):
        sensor_data[idx] = B(uint=((rawdata[i+1] << 8) | rawdata[i]), length=16).int
        idx = idx + 1


      imu_msg = Imu()
      imu_msg.header.stamp = rospy.Time.now()
      imu_msg.header.frame_id = 'imu'
    
      # Convert values to understandable format and package into Imu message.
      x_acc = sensor_data[0] * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
      y_acc = sensor_data[1] * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
      z_gyro = sensor_data[2] * GYRO_RANGE_250DPS * SENSORS_DPS_TO_RADS

      rospy.loginfo("Acc x: {0}\nAcc y: {1}\nGyro z: {2}".format(x_acc, y_acc, z_gyro))

      imu_msg.linear_acceleration.x = x_acc
      imu_msg.linear_acceleration.y = y_acc
      imu_msg.angular_velocity.z = z_gyro
      
      imu_pub.publish(imu_msg)
      
      rate.sleep()

  except rospy.ROSInterruptException:
    pass


