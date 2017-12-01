#!/usr/bin/python

import os
import sys
import serial
import spidev
import rospy
import struct
import RPi.GPIO as GPIO
import numpy as np

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

global x_mean
global y_mean
global z_mean

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

  def calibrate():
    """
    Main calibration function. Gathers 1000 points of data and calculates mean
    values, variances as well as covariances for each of the accelerometer axis
    x and y. The same is done for the z axis of the gyroscope. Returns a dict 
    with these values. 

    The format for dict is:
    calibration_values = {'coordinate': [mean, variance, covariance1..n], ...}
    """

    n = 0
    raw_values = {'x': [], 'y': [], 'z': []}
    byte_data = []

    calibration_values = {'x': [], 'y': [], 'z': []}

    while(n < 1000):

      byte_data = spi.readbytes(6)
      raw_values['x'].append(float(B(uint=((byte_data[1] << 8) | byte_data[0]), length=16).int))
      raw_values['y'].append(float(B(uint=((byte_data[3] << 8) | byte_data[2]), length=16).int))
      raw_values['z'].append(float(B(uint=((byte_data[5] << 8) | byte_data[4]), length=16).int))

      n = n + 1

    # Create a 2-d array with covariances and variances for x and y
    covariance_array = np.cov(raw_values['x'], raw_values['y'])

    x_mean = np.mean(raw_values['x'])
    y_mean = np.mean(raw_values['y'])
    z_mean = np.mean(raw_values['z'])

    calibration_values['x'].append(x_mean)
    calibration_values['x'].append(covariance_array[0][0])
    calibration_values['x'].append(covariance_array[0][1])

    calibration_values['y'].append(y_mean)
    calibration_values['y'].append(covariance_array[1][1])
    calibration_values['y'].append(covariance_array[1][0])
    
    # Only one axis used for gyro. No need for covariance, just call
    # for the variance with np.var
    calibration_values['z'].append(z_mean)
    calibration_values['z'].append(np.var(raw_values['z']))

    return calibration_values

  def get_mean(coordinate, calibration_values):
    return calibration_values[coordinate][0]

  def get_variance(coordinate, calibration_values):
    return calibration_values[coordinate][1]

  def get_covariance(coordinate, calibration_values):
    return calibration_values[coordinate][2]   



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

    # Run a calibration before starting to send data to the robot.

    cal_values = calibrate()

    imu_msg = Imu()

    imu_msg.angular_velocity_covariance = [
      0, 0, 0,
      0, 0, 0,
      0, 0, get_variance(z, cal_values)
    ]
    
    imu_msg.linear_acceleration_covariance = [
      get_variance(x, cal_values), get_covariance(x, cal_values), 0,
      get_covariance(y, cal_values), get_variance(y, cal_values), 0,
      0, 0, 0
    ]

    idx = 0
    rawdata = []
    sensor_data = [0,0,0]

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
      idx = 0
      rawdata = spi.readbytes(6)
      for i in range(0,6,2):
        sensor_data[idx] = float(B(uint=((rawdata[i+1] << 8) | rawdata[i]), length=16).int)
        idx = idx + 1

      imu_msg.header.stamp = rospy.Time.now()
      imu_msg.header.frame_id = 'imu'

      # Convert values to understandable format and package into Imu message.
      x_acc = (sensor_data[0] - x_mean) * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
      y_acc = (sensor_data[1] - y_mean) * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
      z_gyro = (sensor_data[2] - z_mean) * GYRO_RANGE_250DPS * SENSORS_DPS_TO_RADS

      rospy.loginfo("Acc x: {0}\nAcc y: {1}\nGyro z: {2}".format(x_acc, y_acc, z_gyro))

      imu_msg.linear_acceleration.x = x_acc
      imu_msg.linear_acceleration.y = y_acc
      imu_msg.angular_velocity.z = z_gyro

      imu_pub.publish(imu_msg)

      rate.sleep()

  except rospy.ROSInterruptException:
    pass


