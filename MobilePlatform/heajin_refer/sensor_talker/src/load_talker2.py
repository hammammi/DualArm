#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import Float32
from msgpkg.msg import load_cell
from msgpkg.msg import fsr_sensor


ser = serial.Serial(port = '/dev/ttyUSB2',baudrate=115200)

def talker():

    pub1 = rospy.Publisher('loadcell', fsr_sensor, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        response = ser.readline()
        #rospy.loginfo(response)
        va1 = float(response)
        pub1.publish(va1)

        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
