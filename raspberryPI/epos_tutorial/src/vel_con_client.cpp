#include "ros/ros.h"
#include "epos_tutorial/VelCommand.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_con_client");
    if (argc != 5)
    {
        ROS_INFO("usage: vel_con_client Vel1 Vel2 Vel3 Vel4");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<epos_tutorial::VelCommand>("vel_con");
    epos_tutorial::VelCommand srv;
    srv.request.Vel1 = atoll(argv[1]);
    srv.request.Vel2 = atoll(argv[2]);
    srv.request.Vel3 = atoll(argv[3]);
    srv.request.Vel4 = atoll(argv[4]);

    if (client.call(srv))
    {
        ROS_INFO("Set velocity as %ld, %ld, %ld, %ld", (long int)srv.response.setVel1
        ,(long int)srv.response.setVel2, (long int)srv.response.setVel3, (long int)srv.response.setVel4);
    }
    else
    {
        ROS_ERROR("Failed to call service vel_con");
        return 1;
    }

    return 0;
}