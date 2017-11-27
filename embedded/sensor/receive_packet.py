import sys, os, readline, argparse, struct, serial, time, ctypes
import binascii
import spidev
import struct
import binascii
import RPi.GPIO as GPIO


if __name__ == "__main__":
    print("Starting session...")

    spi = spidev.SpiDev()
    spi.open(0, 0) # Steering module
    spi.max_speed_hz = 10000

    # Configure reset pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([29, 31], GPIO.OUT)

    # Reset
    GPIO.output([29, 31], 0)
    time.sleep(0.1)
    GPIO.output([29, 31], 1)

    sdata = [0,0,0]
    rawdata = []

    while True:
        rawdata = spi.readbytes(6)
        # Concatenate high and low bits for acc_x, acc_y and gyro_z
        sdata = [((rawdata[i+1] << 8) | rawdata[i]) for i in range(0,6,2)]
        
        print("Acc x: {0} \n Acc y: {1} \n Gyro z: {2}".format(sdata[0], sdata[1], sdata[2]))
        time.sleep(0.5)


