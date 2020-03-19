#!/usr/bin/env python

import time
import serial

ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=9600)
# ser.port = "/dev/ttyUSB0"
# ser.baudrate = 9600
# ser.open()
if ser.isOpen():
    ser.write('QS\r')
	i = 1
    while i<4:
        response = ser.readline()
        print(response)
		i = i+1
ser.close()
