import sys, os, readline, argparse, struct, serial, time, ctypes
from bitstring import Bits as B
import rospy 
import spidev
import RPi.GPIO as GPIO

if __name__ == "__main__":
    print("Starting session...")

    spi = spidev.SpiDev()
    spi.open(0, 0) # Sensor
    spi.max_speed_hz = 100000

    # Configure reset pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([29, 31], GPIO.OUT)

    # Reset
    GPIO.output([29, 31], 0)
    time.sleep(0.1)
    GPIO.output([29, 31], 1)

    # Values from adafruit c++ acc code to modify raw data.
    ACC_MG_LSB  = 0.001;
    SENSORS_GRAVITY_STANDARD = 9.80665;

    # Values from adafruit c++ gyro code to modfy raw data.
    GYRO_RANGE_250DPS = 0.00875;
    SENSORS_DPS_TO_RADS = 0.017453293;

    rawdata = []
    sdata = [0,0,0]

    while True:
        idx = 0
        rawdata = spi.readbytes(6)
        for i in range(0,6,2):
            sdata[idx] = B(uint=((rawdata[i+1] << 8) | rawdata[i]), length=16).int
            idx = idx + 1
        # print(rawdata)

        # Convert values to understandable format.
        sdata[0] = sdata[0] * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
        sdata[1] = sdata[1] * ACC_MG_LSB * SENSORS_GRAVITY_STANDARD
        sdata[2] = sdata[2] * GYRO_RANGE_250DPS * SENSORS_DPS_TO_RADS
        print("Acc x: {0}\nAcc y: {1}\nGyro z: {2}".format(sdata[0], sdata[1], sdata[2]))
        time.sleep(0.5)
