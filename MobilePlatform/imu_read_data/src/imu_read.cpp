#include "ros/ros.h"
#include <string>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <imu_read_data/msgImu.h>

serial::Serial ser;


/* Imu data */
double euler[3];
double gyro[3];
double acc[3];
double magnet[3];

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_read");
    ros::NodeHandle nh;

    /* Publisher and Subscriber */
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read",100);
    ros::Publisher imu_data_pub = nh.advertise<imu_read_data::msgImu>("/imu_data",100);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port Initialized");
    }else{
        return -1;
    }
    /* Publish rate : 100 Hz */
    ros::Rate loop_rate(100);

    while(ros::ok()){

        ros::spinOnce();

        /* Declare variables */
        imu_read_data::msgImu imu_data;

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read:"<<result.data);
            read_pub.publish(result);



        /* the position to be chopped and the data to be sliced */
        std::size_t pos[11];
        std::string ch_data[12];
        ros::spinOnce();

        /* The procedure of slicing */
        for(int i=0;i<11;i++){
            pos[i] = result.data.find(",");
            ch_data[i] = result.data.substr(1,pos[i]);
            result.data.replace(result.data.find(ch_data[i]),ch_data[i].length(),"");
        }
            ch_data[11]=result.data.substr(1,7);

        /* Convert string to float */
        for(int j=0;j<3;j++){
          euler[j]  = atof(ch_data[j].c_str());
          gyro[j]   = atof(ch_data[j+3].c_str());
          acc[j]    = atof(ch_data[j+6].c_str());
          magnet[j] = atof(ch_data[j+9].c_str());
        }


        imu_data.euler_x = euler[0];
        imu_data.euler_y = euler[1];
        imu_data.euler_z = euler[2];

        imu_data.gyro_x  = gyro[0];
        imu_data.gyro_y  = gyro[1];
        imu_data.gyro_z  = gyro[2];

        imu_data.acc_x   = acc[0];
        imu_data.acc_y   = acc[1];
        imu_data.acc_z   = acc[2];

        imu_data.magn_x  = magnet[0];
        imu_data.magn_y  = magnet[1];
        imu_data.magn_z  = magnet[2];

        imu_data_pub.publish(imu_data);
        }

        loop_rate.sleep();
    }
    return 0;
}
