#!/usr/bin/env python

import time
import serial
import rospy
ser = serial.Serial(port = '/dev/ttyUSB1',baudrate=115200)
# ser.open()
ser.write('CB\r') # Communication for ASCII
time.sleep(0.01)
response = ser.readline()
if response == 'CB\r\n':
    response = ser.readline()
    print(response)
    response = ser.readline()
else:
    print('set your baudrate at 115200')
# ser.write('RS\r')

ser.write('CD A\r') # Communication for ASCII
time.sleep(0.01)
response = ser.readline()
if response == '>CD A\r\n':
    response = ser.readline()
    print('Communication with ASCII')
else:
    print('Communication with ASCII error, try again')

ser.write('CV 20\r') # Mode selection of Resolved force/torque
time.sleep(0.01)
response = ser.readline()
if response == '>CV 20\r\n':
    response = ser.readline()
    print('Only Resolved Z axis torque(unit : count)')
else:
    print('mode selection of resolved torque error, try again')



ser.write('CD R\r') # Communication for Resolved force/torque
time.sleep(0.01)
response = ser.readline()
if response == '>CD R\r\n':
    response = ser.readline()
    print('Communication with Resolved torque(unit : count)')
else:
    print('Communication with Resolved torque error, try again')

ser.write('SA 8\r') # Moving average fillter 0 2 4 8 16 32 64 128
time.sleep(0.01)
response = ser.readline()
if response == '>SA 8\r\n':
    response = ser.readline()

    print('Moving average fillter : 8')
else:
    print('Moving average fillter error, try again')


ser.write('ST 1\r') # Temperature compensation
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

ser.write('SF 1000\r') 
time.sleep(0.01)
response = ser.readline()
response = ser.readline()


ser.write('SZ\r') # Sensor Zero bias
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

for i in range(3):# remove biaS
    ser.write('SB\r') # fill the buffer
    time.sleep(0.01)
    response = ser.readline()
    response = ser.readline()
    
ser.write('SU\r') # remove bias
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

print('Complete for removing bias')

ser.write('SP 1\r') # Peak level monitoring
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

#

print('Complete for Initializing.............')


