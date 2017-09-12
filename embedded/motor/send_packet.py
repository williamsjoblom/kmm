#!/usr/bin/env python3

import sys, os, readline, argparse, struct, serial, time

def read_uint16():
    while True:
        try:
            i = int(input('> '))
            if i >= 0 and i < int(2**16/2):
                return i
        except ValueError:
            pass
        print("Must be a positive integer between 0 and", 2**16/2 - 1)


def build_packet():
    cmd = read_uint16()
                                        
    buf = []

    # Magic 8-bit header
    buf.extend(struct.pack("<b", 42))

    # Data
    buf.extend(struct.pack("<h", cmd))
    for i in range(3):
        buf.extend(struct.pack("<h", i))

    # Magic 8-bit footer
    buf.extend(struct.pack("<b", 24))

    return bytearray(buf)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("device")
    args = p.parse_args()

    device = args.device

    print("Starting session on", device + "...")
    
    if not os.path.exists(device):
        print("'" + device + "' port not found!")
        sys.exit(1)
        
    s = serial.Serial(port=device, baudrate=9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS)
        
    while True:
        byte_buf = build_packet()

        s.write(byte_buf)

        # Wait for response
        time.sleep(0.1)
        
        out = []
        while s.inWaiting() > 0:
            out += s.read(1)

        out_str = "".join([chr(c) for c in out])
        print(out_str)
