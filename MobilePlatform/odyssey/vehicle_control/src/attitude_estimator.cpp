#include <vehicle_control/attitude_estimator.hpp>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"attitude_estimator");
    attitude_estimator estimator;
    estimator.PublisherSetting();
    estimator.EncoderSubscriberSetting();
    estimator.InitiateVariables();
    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        estimator.TransformPublisher();
        ros::spinOnce();
        loop_rate.sleep();
    }

}