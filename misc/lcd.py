#!/usr/bin/env python

import socket
import fcntl
import struct
import time

from subprocess import check_output

from RPLCD.gpio import CharLCD
from RPi import GPIO

last_ip = None
last_ssid = None

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
    output = check_output(["iwgetid", "wlan0", "-r"])
    return output.strip() if output else None
    

def center(s):
    """
    Return centered string.
    """
    i = int((16 - len(s)) / 2)
    return ' '*i + s


def refresh(lcd):
    """
    Refresh LCD.
    """
    global last_ssid
    global last_ip
    
    ip = wlan_ip()
    ssid = wlan_ssid()
    if not ssid:
        ssid = "not connected"
        
    if ip != last_ip or ssid != last_ssid:
        lcd.clear()
        
        lcd.cursor_pos = (0, 0)
        lcd.write_string(center(ip))
        
        lcd.cursor_pos = (1, 0)
        lcd.write_string(center(ssid))

        last_ip = ip
        last_ssid = ssid

    
if __name__ == '__main__':
    try:
        
        lcd = CharLCD(pin_rs=8, pin_e=10, pin_rw=None,
                      pins_data=[12, 16, 18, 22], cols=16,
                      rows=2, numbering_mode=GPIO.BOARD,
                      auto_linebreaks=True)
                        
        while True:
            refresh(lcd)
            time.sleep(2)
            print('SSID: ' + wlan_ssid())
            print('IP: ' + str(wlan_ip()))                
            
    except Exception as e:
        print("Exception occurred: " + str(e))
    finally:
        lcd.clear()
        lcd.close(True)
