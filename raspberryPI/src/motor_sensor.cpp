#include "ros/ros.h"
#include "epos_tutorial/realVel.h"

void commandCallback(const epos_tutorial::realVel::ConstPtr& msg)
{
    ROS_INFO("Motor Velocity (rpm): %ld, %ld, %ld, %ld", msg->realVel[0], msg->realVel[1], msg->realVel[2], msg->realVel[3]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_sensor");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("measure", 1000, commandCallback);

    ros::spin();

    return 0;
}