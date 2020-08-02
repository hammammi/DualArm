#include <ros/ros.h>
#include <iostream>
#include <vehicle_control/motorsMsg.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using vehicle_control::motorsMsg;
using Eigen::VectorXd;

using std::cout;
using std::endl;

class motor_vel_profile{

    public:
        void publisher_setting();
        void velocity_profile_pub();

    private:

        ros::NodeHandle nh;
        ros::Publisher velocity_publisher;

        double current_time;
        double last_time = ros::Time::now().toSec();
        double T = 0;
        double dt = 0;

        const double alpha = 3000;
        const double omega = 3000;
        int control_input = 0;
};

void motor_vel_profile::publisher_setting(){
    velocity_publisher = nh.advertise<motorsMsg>("input_msg",1);
}

void motor_vel_profile::velocity_profile_pub(){
    
    motorsMsg control_msg;

    current_time = ros::Time::now().toSec();
    dt = current_time - last_time;

    if(T>=2.000 && T<4.000){
        control_input = (int) 3000*(T-2.000);
    }
    else if(T>4.000 && T<10.000){
        control_input = 6000;
    }
    else if(T>=10.000 && T<12.000){
        control_input = (int) 6000 - 3000*(T-10.000);
    }
    else if(T>=12.000 && T<14.000){
        control_input = 0;
    }
    else if(T>=14.000 && T<16.000){
        control_input = (int) -3000*(T-14.000);
    }
    else if(T>=16.000 && T<26.000){
        control_input = -6000;
    }
    else if(T>=26.000 && T<28.000){
        control_input = -6000 + 3000*(T-26.000);
    }
    else if(T>=19.000 && T<22.000){
        control_input = 0;
    }
    else if(T>=22.000){
        control_input = 0;
        T = 0;
    }

    T += dt;
    last_time = current_time;
    control_msg.omega1 = control_input;
    control_msg.omega2 = -control_input;
    control_msg.omega3 = -control_input;
    control_msg.omega4 = control_input;
    
    velocity_publisher.publish(control_msg);

}