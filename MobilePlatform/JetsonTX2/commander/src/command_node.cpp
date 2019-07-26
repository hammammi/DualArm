#include "ros/ros.h"
#include "/home/nvidia/catkin_ws/devel/include/mobile_control/motorMsg.h"

int32_t getVel[]={0,0,0,0};

void commandCallback(const mobile_control::motorMsg::ConstPtr& msg)
{
	getVel[0]=msg->omega1;
	getVel[1]=msg->omega2;
    getVel[2]=msg->omega3;
    getVel[3]=msg->omega4;

}



int main(int argc,char **argv)
{
    ros::init(argc, argv, "command_node");
    ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("input",1000,commandCallback);

    ros::Publisher motor_pub = nh.advertise<mobile_control::motorMsg>("input_msg",1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
		mobile_control::motorMsg msg;

        msg.omega1 = getVel[0];
		msg.omega2 = getVel[1];
		msg.omega3 = getVel[2];
		msg.omega4 = getVel[3];

		motor_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
