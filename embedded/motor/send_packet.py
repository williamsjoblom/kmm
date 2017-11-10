import sys, os, readline, argparse, struct, serial, time, ctypes
import binascii
import spidev
import RPi.GPIO as GPIO


def is_int16(i):
    """
    Does i fit in a 16 bit signed integer?
    """
    return abs(i) < int(2**16/2)


def is_int16_vector(v):
    """
    Is v a three dimensional vector?
    """
    for i in v:
        if not is_int16(i):
            return False

    return True


def read_int16():
    """
    Read a 16 bit signed integer from stdin.
    """
    while True:
        try:
            i = int(raw_input('> '))

            if is_int16(i):
                return i
        except ValueError:
            pass

        print("Must be a positive integer between", -2**16/2 + 1, "  and", 2**16/2 - 1)


def read_speed_vector():
    """
    Read three dimensional 16 bit signed integer vector from stdin.
    """
    while True:
        try:
            v = list(map(int, raw_input('> ').strip().split(" ")))

            if len(v) == 3 and is_int16_vector(v):
                return (v[0], v[1], v[2])
        except ValueError:
            pass

        print("Vector must be three positive integers between", -2**16/2 + 1, " and", 2**16/2 - 1)



def build_packet():
    """
    Build packet bytearray from user input.
    """
    cmd = 0
    speed_vector = read_speed_vector()

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


if __name__ == "__main__":
    print("Starting session...")

    spi = spidev.SpiDev()
    spi.open(1, 1) # Steering module
    spi.max_speed_hz = 100

    # Configure reset pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([29, 31], GPIO.OUT) 

    # Reset
    GPIO.output([29, 31], 0)
    GPIO.output([29, 31], 1)
    
    while True:
        byte_buf = build_packet()
        spi.xfer(byte_buf)
