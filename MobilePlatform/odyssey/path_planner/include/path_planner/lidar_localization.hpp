#include "ros/ros.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <math.h>

#define _USE_MATH_DEFINES

using namespace nav_msgs;
using namespace sensor_msgs;

class localization{

    public:

    double center_feature_distance[2] = {0, 0};
    double center_feature_angle[2] = {0, 0};
    int center_feature_index[2] = {0, 0};

    double right_feature_distance[2][10];
    double right_feature_angle[2][10];
    int right_feature_index[2][10];    

    double left_feature_distance[2][10];
    double left_feature_angle[2][10];
    int left_feature_index[2][10];

    bool is_init = false;

    double x_lidar_odom = 0;
    double y_lidar_odom = 0;

    void subscribe_setting(){
        
        subscriber_scan = nh_.subscribe("/scan",1,&localization::callback_scan,this);
    }

    void callback_scan(const LaserScan::ConstPtr& scan_msg){
        
        int size_of_scan_data = scan_msg -> ranges.size();
        int i = 0;
        double feature_scan_data[size_of_scan_data*2/8];
        double arr[size_of_scan_data*2/8];

        double max_distance = 0;

            for(i=0;i<size_of_scan_data*2/8;i++){
                feature_scan_data[i] = scan_msg -> ranges[i+size_of_scan_data*1/8];

                arr[i] = feature_scan_data[i];

                if(std::isinf(arr[i]) == 1){
                    arr[i] = 0;
                }

                if(arr[i]>max_distance){
                    max_distance = arr[i];
                    center_feature_index[1] = i;
                }
            }
            
        int n = sizeof(arr)/sizeof(double);
        std::sort(arr,arr+n,std::greater<double>());

        center_feature_distance[1] = arr[0];
        center_feature_angle[1] = (double) M_PI/2.0*center_feature_index[1]/n;

        ROS_INFO("feature angle %lf",center_feature_angle[0]* 180/M_PI);

        for(int j=0;j<10;j++){
            right_feature_index[1][j] = center_feature_index[1] - j;
            right_feature_distance[1][j] = feature_scan_data[right_feature_index[1][j]];
            right_feature_angle[1][j] = (double) M_PI/2.0*right_feature_index[1][j]/n;

            left_feature_index[1][j] = center_feature_index[1] + j;
            left_feature_distance[1][j] = feature_scan_data[left_feature_index[1][j]];   
            left_feature_angle[1][j] = (double) M_PI/2.0*left_feature_index[1][j]/n;
        }
        // Registere Initial position
        if(is_init == 0){


                center_feature_index[0] = center_feature_index[1];
                center_feature_distance[0] = center_feature_distance[1];
                center_feature_angle[0] = center_feature_angle[1];

            for(int j=0;j<10;j++)
            {

                right_feature_index[0][j] = right_feature_index[1][j];
                right_feature_distance[0][j] = right_feature_distance[1][j];
                right_feature_angle[0][j] = right_feature_angle[1][j];

                left_feature_index[0][j] = left_feature_index[1][j];
                left_feature_distance[0][j] = left_feature_distance[1][j];
                left_feature_angle[0][j] = left_feature_angle[1][j];
            }
            is_init = true;
            
            ROS_INFO("Regiestered Initial Position");
        }
    }

    void publish_setting(){
        publisher_lidar_odom = nh_.advertise<Odometry>("/lidar_odom",1);
    }

    void publish_odom(){

        Odometry lidar_odom;
            if(is_init == 1){
                for(int j = 0;j<3;j++){
                x_lidar_odom +=  -left_feature_distance[1][j] * sin(left_feature_angle[1][j]) + left_feature_distance[0][j] * sin(left_feature_angle[0][j]);
                
                //x_lidar_odom += right_feature_distance[1] * cos(right_feature_angle[1]) - right_feature_distance[0] * cos(right_feature_angle[0]);
                //x_lidar_odom += left_feature_distance[1] * cos(left_feature_angle[1]) - left_feature_distance[0] * cos(left_feature_angle[0]);
                //x_lidar_odom = x_lidar_odom / 3.0;

                y_lidar_odom +=  right_feature_distance[1][j] * cos(right_feature_angle[1][j]) - right_feature_distance[0][j] * cos(right_feature_angle[0][j]);
                //y_lidar_odom += right_feature_distance[1] * sin(right_feature_angle[1]) - right_feature_distance[0] * sin(right_feature_angle[0]);
                //y_lidar_odom += left_feature_distance[1] * sin(left_feature_angle[1]) - left_feature_distance[0] * sin(left_feature_angle[0]);       
                //y_lidar_odom = y_lidar_odom / 3.0;
                }

            }else
            {
                x_lidar_odom = 0;
                y_lidar_odom = 0;
            }

            x_lidar_odom = x_lidar_odom/3.0;
            y_lidar_odom = y_lidar_odom/3.0;
            

            lidar_odom.pose.pose.position.x = x_lidar_odom;
            lidar_odom.pose.pose.position.y = y_lidar_odom;

        publisher_lidar_odom.publish(lidar_odom);
    }


    private:

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_scan;
    ros::Publisher publisher_lidar_odom;
    
};