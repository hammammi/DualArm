//sudo slcand -o -c -f -s4 /dev/ttyUSB3 slcan0
//sudo ifconfig slcan0 up
//candump slcan0

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "msgpkg/fabric.h"

#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <serial/serial.h>

using namespace std;
serial::Serial ser;
float hz = 1.0/2000.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_serial");
    ros::NodeHandle n;
    ros::Publisher arduino_pub = n.advertise<msgpkg::fabric>("ser_cpp", 10);
    ros::Rate loop_rate(int(1.0/hz));
    //conect to serial communication
    try
    {
        ser.setPort("/dev/ttyUSB6");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
                                                                                                                                                                                                                                                                                       
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    msgpkg::fabric msg;
    string tmp;
    char tmp2[8];
    while (ros::ok())
    {
	 //if(ser.available()){
	ser.write("a");
        sleep(0.000005);
        for (int i=0;i<8;i++){
            tmp = ser.read();
            tmp2[i] = strtol(tmp.c_str(),NULL,16);
        }
        msg.Data1 = (tmp2[0]& 0b0011)<<12 | tmp2[1]<<8 | tmp2[2]<<4 | tmp2[3];
	msg.Data2 = (tmp2[4]& 0b0011)<<12 | tmp2[5]<<8 | tmp2[6]<<4 | tmp2[7];
	arduino_pub.publish(msg);
	loop_rate.sleep();
		//}
     
    }
	ser.close();

    return 0;
} 
