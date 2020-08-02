#include <path_planner/lidar_localization.hpp>

int main(int argc, char** argv){
    ros::init(argc,argv,"localization");

    localization velodyne_localization;

    velodyne_localization.publish_setting();
    velodyne_localization.subscribe_setting();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        velodyne_localization.publish_odom();

        ros::spinOnce();
        loop_rate.sleep();
    }
}