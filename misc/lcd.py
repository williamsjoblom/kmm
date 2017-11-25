#!/usr/bin/env python

import socket
import fcntl
import struct
import time

from subprocess import check_output

from RPLCD.gpio import CharLCD
from RPi import GPIO

"""
Last string written to the display.
"""
last_string = ' '


def wlan_ip():
    """
    Get IP from wlan interface.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', 'wlan0'[:15])
    )[20:24])


def wlan_ssid():
    """
    Get SSID of current network.
    """
    output = check_output(["iwlist", "wlan0", "scan"])
    for line in output.split():
        if line.startswith("ESSID"):
            return line.split('"')[1]

    return None

def center(s):
    i = 16 % len(s)
    return ' '*(i/2) + s


def refresh(lcd):
    """
    Refresh LCD.
    """
    ip = wlan_ip()
    ssid = wlan_ssid()
    if ssid == None:
        ssid = "not connected"

    lcd.clear()
    lcd.cursor_pos = (0, 0)
    
    s = center(ip) + '\n\r' + center(ssid)
    lcd.write_string(s)
        
if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    
    try:
        lcd = CharLCD(pin_rs=8, pin_e=10, pin_rw=None,
                      pins_data=[12, 16, 18, 22], cols=16,
                      rows=2, numbering_mode=GPIO.BOARD,
                      auto_linebreaks=True)
        lcd.clear()

                
        while True:
            refresh(lcd)
            time.sleep(0.5)
            print('Reset: ' + str(GPIO.input(11)))
	    print('SSID: ' + wlan_ssid())
	    print('IP: ' + str(wlan_ip()))                
            
    except Exception as e:
        print("Exception occurred: " + str(e))
    finally:
        lcd.close(True)
