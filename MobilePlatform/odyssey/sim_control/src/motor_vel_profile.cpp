#include <sim_control/motor_vel_profile.hpp>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"profile_test");

    motor_vel_profile profile_generator;

    profile_generator.publisher_setting();

    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        profile_generator.velocity_profile_pub();
        ros::spinOnce();
        loop_rate.sleep();
    }
}