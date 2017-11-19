import sys, os, readline, argparse, struct, serial, time, ctypes
import binascii
import spidev
import RPi.GPIO as GPIO

if __name__ == "__main__":
    print("Starting session...")

    spi = spidev.SpiDev()
    spi.open(0, 0) # Steering module
    spi.max_speed_hz = 7692

    # Configure reset pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([29, 31], GPIO.OUT)

    # Reset
    GPIO.output([29, 31], 0)
    time.sleep(0.1)
    GPIO.output([29, 31], 1)

    while True:
        spi.readbytes(6)
        print(bytes)
        input()