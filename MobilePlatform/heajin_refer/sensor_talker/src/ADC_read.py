#!/usr/bin/env python
import rospy
import time
import serial
from msgpkg.msg import magnetic_encoder

M_PI = 3.14159265358979323846
hz = 500.0
n_init = 10

# magnetic encoder

ser = serial.Serial(port = '/dev/ttyUSB1',baudrate=115200)

def talker():
    pub1 = rospy.Publisher('angle1', magnetic_encoder, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    t0 = time.time()
    t_pre = time.time()
    sum_1 = 0
    i=0
    while i < n_init:
	t_cur = time.time()
	if (t_cur-t_pre)>1/float(hz):
	    response = ser.readline()
	    va1 = float(response)
	    angle_data_1 = float(va1)/4095.0*2*M_PI
	    sum_1 = sum_1 + angle_data_1
	    t_pre = t_cur
	    i = i+1

    base_1 = float(sum_1)/float(n_init)
    t0 = time.time()
    t_pre = time.time()
    while not rospy.is_shutdown():
	t_cur = time.time()
	if (t_cur-t_pre)>1/float(hz):
	    #### read
	    response = ser.readline()
	    va1 = float(response)

	    #### data
	    angle_data_1 = float(va1)/4095.0*2*M_PI - base_1

	    #### pub
	    pub1.publish(angle_data_1)
	    #print(base_1)
	    
	    t_pre = t_cur;

	    #response = ser.readline()
            #where1 = response.find(',')
	    #response2 = response[where1+1:]
            #where2 = response2.find(',')
            #va1 = float(response[:where1])
            #va2 = float(response2[:where2])
            #va3 = float(response2[where2+1:])
            #pub1.publish(va1)
            #pub2.publish(va2)
            #pub3.publish(va3)
            #rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


