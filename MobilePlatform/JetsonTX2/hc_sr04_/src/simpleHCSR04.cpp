/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/// simpleHCSR04.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <unistd.h>
//#include <jetsonGPIO.h>
//#include <hcsr04.h>
#include <hcsr04.cpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <vector>

using namespace std;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}



int main(int argc, char *argv[]){

  // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "hc_sr04_test");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 10 hz
    // Use trigger, echo
    HCSR04 *hcsr04_1 = new HCSR04(gpio398, gpio298);
    HCSR04 *hcsr04_2 = new HCSR04(gpio254, gpio255);
    HCSR04 *hcsr04_3 = new HCSR04(gpio389, gpio395);
    HCSR04 *hcsr04_4 = new HCSR04(gpio393, gpio394);
    // Make the HC-SR04 available in user space
    hcsr04_1->exportGPIO() ;
    hcsr04_2->exportGPIO() ;
    hcsr04_3->exportGPIO() ;
    hcsr04_4->exportGPIO() ;

    usleep(1000000);

    // Then set the direction of the pins
    hcsr04_1->setDirection() ;
 
    hcsr04_2->setDirection() ;
 
    hcsr04_3->setDirection() ;
 
    hcsr04_4->setDirection() ;
    
    ros::Publisher sonar_pub1 = node.advertise<sensor_msgs::Range>("sonar1", 10);
    ros::Publisher sonar_pub2 = node.advertise<sensor_msgs::Range>("sonar2", 10);
    ros::Publisher sonar_pub3 = node.advertise<sensor_msgs::Range>("sonar3", 10);
    ros::Publisher sonar_pub4 = node.advertise<sensor_msgs::Range>("sonar4", 10);


    sensor_msgs::Range range;
    range.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range.min_range = 0.0;
    range.max_range = 30.0;


    while(ros::ok()) {    

       
       unsigned int duration1 = hcsr04_1->pingMedian(5) ;
      
       
       //ROS_INFO("%d",duration);
        if (duration1 == NO_ECHO)
           { ROS_WARN("Error on sonar1");}
        else{
          range.header.stamp = ros::Time::now(); 
          range.range = duration1/58.0;
          unsigned int Range = range.range;
//          ROS_INFO("1");
//          ROS_INFO("%d",Range);
	  sonar_pub1.publish(range);}

       unsigned int duration2 = hcsr04_2->pingMedian(5) ;
    
        if (duration2 == NO_ECHO)
           { ROS_WARN("Error on sonar2");}
        else{
          range.header.stamp = ros::Time::now();
          range.range = duration2/58.0;
          unsigned int Range = range.range;
//  	    ROS_INFO("2");
//          ROS_INFO("%d",Range);
	  sonar_pub2.publish(range);}

       unsigned int duration3 = hcsr04_3->pingMedian(5) ;
    
        if (duration3 == NO_ECHO)
           { ROS_WARN("Error on sonar3");}
        else{
          range.header.stamp = ros::Time::now();
          range.range = duration3/58.0;
          unsigned int Range = range.range;
//  	    ROS_INFO("3");
//          ROS_INFO("%d",Range);
	  sonar_pub3.publish(range);}

       unsigned int duration4 = hcsr04_4->pingMedian(5) ;

        if (duration4 == NO_ECHO)
           { ROS_WARN("Error on sonar4");}
        else{
          range.header.stamp = ros::Time::now();
          range.range = duration4/58.0;
          unsigned int Range = range.range;
//	    ROS_INFO("4");
//          ROS_INFO("%d",Range);
	  sonar_pub4.publish(range);}


      ros::spinOnce();
      rate.sleep();    
    }
  
    cout << "HC-SR04 example finished." << endl;
    hcsr04_1->unexportGPIO() ;
    hcsr04_2->unexportGPIO() ;
    hcsr04_3->unexportGPIO() ;
    hcsr04_4->unexportGPIO() ;
    return 0;
}


