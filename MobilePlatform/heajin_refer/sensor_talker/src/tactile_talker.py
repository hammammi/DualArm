#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import Float32
from msgpkg.msg import fabric

msg = fabric()

ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)


def talker():
    pub1 = rospy.Publisher('tactile', fabric, queue_size=1)
    rospy.init_node('tactile', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    print("hi")
    while not rospy.is_shutdown():
        ser.write('a')
	time.sleep(0.000001)
        #print("hi")
        response = ser.readline()
        where1 = response.find(',')
	response2 = response[where1+1:]
	where2 = response2.find(',')
        msg.Data1 = int(response[:where1])
	msg.Data2 = int(response2)
	#print("hi")
        pub1.publish(msg)
	rate.sleep()

        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
