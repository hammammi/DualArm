#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mobile_control/motorMsg.h>
#include <sim_control/desiredMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#define _USE_MATH_DEFINES

using namespace teb_local_planner;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sim_control;

struct motor_vel{
    double w[4] = {0,0,0,0};
};

struct goal_pos{
    double x_goal = 0;
    double y_goal = 0;
    double phi_goal = 0;
};

struct desired_pos_vel_acc{
    // In the global frame
    double x_des = 0;
    double y_des = 0;
    double phi_des = 0;

    double vx_des = 0;
    double vy_des =0;
    double vphi_des = 0;
    
    double ax_des = 0;
    double ay_des = 0;
    double aphi_des = 0;
};

struct estimated_pos_vel{
    // In the global frame
    double x_est = 0;
    double y_est = 0;
    double phi_est = 0;

    double qw_est = 1;
    double qx_est = 0;
    double qy_est = 0;
    double qz_est = 0;
    // In the robot frame
    double vx_est = 0;
    double vy_est = 0;
    double vphi_est = 0;
};

struct error_pos_vel{
    // In the robot frame
    double x_error = 0;
    double y_error = 0;
    double phi_error = 0;

    double vx_error = 0;
    double vy_error = 0;
    double vphi_error = 0;
};

struct command_vel_acc{
    // In the robot frame
    double ax_cmd = 0;
    double ay_cmd = 0;
    double aphi_cmd = 0;
    
    double vx_cmd = 0;
    double vy_cmd = 0;
    double vphi_cmd = 0;

    double vx_accumul = 0;
    double vy_accumul = 0;
    double vphi_accumul = 0;
};

class tracking_controller{


    public:

    // Publisher Declaration
    void cmd_vel_pub_setting(){
        publisher_desired_traj = nh_.advertise<nav_msgs::Odometry>("/des_traj",1);
        publisher_cmd_vel = nh_.advertise<mobile_control::motorMsg>("/input_msg",1);
        publisher_slam_pose = nh_.advertise<nav_msgs::Odometry>("/slam_pose",1);
        publisher_error = nh_.advertise<nav_msgs::Odometry>("/error_msg",1);
    }


    // Subscriber Declaration
    void subscriber_declaration(){
        // Subscribe
        subscriber_state = nh_.subscribe("/odom",1,&tracking_controller::callback_state,this);
        subscriber_trajectory = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback",1,&tracking_controller::callback_traj,this);
        subscriber_goal = nh_.subscribe("/move_base_simple/goal",1,&tracking_controller::callback_goal,this);
    }

    // Publish commend motor velocity
    void cmd_vel_pub(){



        tf::StampedTransform transform;

        try{

            curr_time = ros::Time::now().toSec();
            dt = curr_time - last_time;

            listener_.lookupTransform("map","base_footprint",ros::Time(0),transform);
            
            est_pos_vel.x_est = transform.getOrigin().x();
            est_pos_vel.y_est = transform.getOrigin().y();
        
            tf::Quaternion quat = transform.getRotation();
            est_pos_vel.phi_est = tf::getYaw(quat);

            if(est_pos_vel.phi_est > M_PI)
            {
                est_pos_vel.phi_est -= 2*M_PI;
            }

            ROS_INFO("x_est y_est phi_est: %lf, %lf, %lf",est_pos_vel.x_est,est_pos_vel.y_est,est_pos_vel.phi_est);
            //ROS_INFO("vx robot vy_robot (SLAM) : %lf, %lf",vx_robot, vy_robot);

        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // Position and orientation error
        err_pos_vel.x_error = (des_pva.x_des - est_pos_vel.x_est)*cos(est_pos_vel.phi_est) - (des_pva.y_des - est_pos_vel.y_est)*sin(est_pos_vel.phi_est);
        err_pos_vel.y_error = (des_pva.x_des - est_pos_vel.x_est)*sin(est_pos_vel.phi_est) + (des_pva.y_des - est_pos_vel.y_est)*cos(est_pos_vel.phi_est);
        err_pos_vel.phi_error = des_pva.phi_des - est_pos_vel.phi_est;

        // Velocity error
        err_pos_vel.vx_error = (des_pva.vx_des - est_pos_vel.vx_est)*cos(est_pos_vel.phi_est) - (des_pva.vy_des - est_pos_vel.vy_est)*sin(est_pos_vel.phi_est);
        err_pos_vel.vy_error = (des_pva.vx_des - est_pos_vel.vx_est)*sin(est_pos_vel.phi_est) + (des_pva.vy_des - est_pos_vel.vy_est)*cos(est_pos_vel.phi_est);
        err_pos_vel.vphi_error = des_pva.vphi_des - est_pos_vel.vphi_est;

        if(err_pos_vel.phi_error > M_PI)
        {
            err_pos_vel.phi_error -= 2*M_PI;
        }
        else if(err_pos_vel.phi_error <= - M_PI)
        {
            err_pos_vel.phi_error += 2*M_PI;
        }

        double ax_des_robot = 0;
        double ay_des_robot = 0;

        double vx_des_robot = 0;
        double vy_des_robot = 0;

        ax_des_robot = des_pva.ax_des * cos(est_pos_vel.phi_est) - des_pva.ay_des * sin(est_pos_vel.phi_est);
        ay_des_robot = des_pva.ax_des * sin(est_pos_vel.phi_est) + des_pva.ay_des * cos(est_pos_vel.phi_est);
        
        vx_des_robot = des_pva.vx_des * cos(est_pos_vel.phi_est) - des_pva.vy_des * sin(est_pos_vel.phi_est);
        vy_des_robot = des_pva.vx_des * sin(est_pos_vel.phi_est) + des_pva.vy_des * cos(est_pos_vel.phi_est);

        cmd_vel_acc.vx_cmd = Kp[0] * err_pos_vel.x_error + Kd[0] * err_pos_vel.vx_error;
        cmd_vel_acc.vy_cmd = Kp[1] * err_pos_vel.y_error + Kd[1] * err_pos_vel.vy_error;
        cmd_vel_acc.vphi_cmd = Kp[2] * err_pos_vel.phi_error + Kd[2] * err_pos_vel.vphi_error;        

        ROS_INFO("vx vy vphi : %lf %lf %lf",cmd_vel_acc.vx_cmd,cmd_vel_acc.vy_cmd,cmd_vel_acc.vphi_cmd);
        
        cmd_motor_vel = inverse_kinematics(cmd_vel_acc);

        // Unit conversion
        cmd_motor_vel.w[0] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[0];
        cmd_motor_vel.w[1] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[1];
        cmd_motor_vel.w[2] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[2];
        cmd_motor_vel.w[3] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[3];
                         
        cmd_motor_vel = clamping(cmd_motor_vel);

        if(sqrt(pow((goal_p.x_goal-est_pos_vel.x_est),2) + pow((goal_p.y_goal-est_pos_vel.y_est),2))<0.020 && fabs(goal_p.phi_goal - est_pos_vel.phi_est)<0.02 || init_pos == true){

            cmd_motor_vel.w[0] = 0;
            cmd_motor_vel.w[1] = 0;
            cmd_motor_vel.w[2] = 0;
            cmd_motor_vel.w[3] = 0;

            goal_p.x_goal = est_pos_vel.x_est;
            goal_p.y_goal = est_pos_vel.y_est;
            goal_p.phi_goal = est_pos_vel.phi_est;

            ROS_INFO("Stop");
            ROS_INFO("distance error : %lf phi_err : %lf",sqrt(pow((goal_p.x_goal-est_pos_vel.x_est),2) + pow((goal_p.y_goal-est_pos_vel.y_est),2)),fabs(goal_p.phi_goal - est_pos_vel.phi_est)*180/M_PI);
        }


        mobile_control::motorMsg motor_vel;
        nav_msgs::Odometry error_msg;

        motor_vel.omega1 = cmd_motor_vel.w[0];
        motor_vel.omega2 = -cmd_motor_vel.w[1];
        motor_vel.omega3 = -cmd_motor_vel.w[2];
        motor_vel.omega4 = cmd_motor_vel.w[3];
              
        error_msg.header.stamp = ros::Time::now();    
        error_msg.pose.pose.position.x = err_pos_vel.x_error;
        error_msg.pose.pose.position.y = err_pos_vel.y_error;
        
        error_msg.twist.twist.linear.x = err_pos_vel.vx_error;
        error_msg.twist.twist.linear.y = err_pos_vel.vy_error;
        error_msg.twist.twist.angular.z = err_pos_vel.vphi_error;

        //Publish control input msg
        publisher_cmd_vel.publish(motor_vel);
        publisher_error.publish(error_msg);

        last_time = ros::Time::now().toSec();
    }

    motor_vel inverse_kinematics(command_vel_acc vel){
        motor_vel m;

        m.w[0] = 1.0/r * vel.vx_cmd - 1.0/r * vel.vy_cmd - l/r * vel.vphi_cmd;
        m.w[1] = 1.0/r * vel.vx_cmd + 1.0/r * vel.vy_cmd + l/r * vel.vphi_cmd;
        m.w[2] = 1.0/r * vel.vx_cmd - 1.0/r * vel.vy_cmd + l/r * vel.vphi_cmd;
        m.w[3] = 1.0/r * vel.vx_cmd + 1.0/r * vel.vy_cmd - l/r * vel.vphi_cmd;

        return m;
    }

    motor_vel clamping(motor_vel clamping_motor_vel){

        double arr[4] = {fabs(clamping_motor_vel.w[0]),fabs(clamping_motor_vel.w[1]),fabs(clamping_motor_vel.w[2]),fabs(clamping_motor_vel.w[3])};
        double vx_;
        double vy_;
        double vphi_;

        motor_vel clamped_motor_vel;

        double motor_vel_max = 0;
        //motor_vel_max = *std::max_element(arr,arr+4);
        std::sort(arr,arr+4,std::greater<double>());
        motor_vel_max = arr[0];
        
        if(motor_vel_max>motor_vel_lim){
            clamped_motor_vel.w[0] = (double) motor_vel_lim / motor_vel_max * clamping_motor_vel.w[0];
            clamped_motor_vel.w[1] = (double) motor_vel_lim / motor_vel_max * clamping_motor_vel.w[1];
            clamped_motor_vel.w[2] = (double) motor_vel_lim / motor_vel_max * clamping_motor_vel.w[2];
            clamped_motor_vel.w[3] = (double) motor_vel_lim / motor_vel_max * clamping_motor_vel.w[3];
        }
        
        else{
            clamped_motor_vel.w[0] = (double) clamping_motor_vel.w[0];
            clamped_motor_vel.w[1] = (double) clamping_motor_vel.w[1];
            clamped_motor_vel.w[2] = (double) clamping_motor_vel.w[2];
            clamped_motor_vel.w[3] = (double) clamping_motor_vel.w[3];
        }

        vx_ = r/4.0 * (clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vy_ = r/4.0 * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] - clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vphi_ = r/4.0/l * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] - clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        
        return clamped_motor_vel;
        
    }

    void publish_slam_pose(){
        nav_msgs::Odometry slam_pose;

        slam_pose.header.stamp = ros::Time::now();
        // Get SLAM position
        slam_pose.pose.pose.position.x = est_pos_vel.x_est;
        slam_pose.pose.pose.position.y = est_pos_vel.y_est;
    
        // Get SLAM orientation
        geometry_msgs::Quaternion slam_quat = tf::createQuaternionMsgFromYaw(est_pos_vel.phi_est);
        slam_pose.pose.pose.orientation = slam_quat;

        publisher_slam_pose.publish(slam_pose);
    }

    void desired_traj(){ 
        nav_msgs::Odometry des_traj;
        des_traj.header.stamp = ros::Time::now();

        des_traj.pose.pose.position.x = des_pva.x_des;
        des_traj.pose.pose.position.y = des_pva.y_des;

        des_traj.twist.twist.linear.x = des_pva.vx_des;
        des_traj.twist.twist.linear.y = des_pva.vy_des;
        des_traj.twist.twist.angular.z = des_pva.vphi_des;
        
        publisher_desired_traj.publish(des_traj);
    }

    // Callback function 1 : Trajectory from planner
    void callback_traj(const TrajectoryPointMsg::ConstPtr& traj_msg){


        // Trajectory generation --> TrajectoryPointMsg
        // Teb planner --> FeedbackMsg
        // Robot index

        /**
        robot_index = traj_msg -> selected_trajectory_idx;
        
        int traj_index = 1;
        bool check_goal_message = false;

        
        if(!traj_msg->trajectories.empty())
        {
            if(check_goal_message == false)
            {
                check_goal_message = true;
            }
            init_pos = false;
        }

        // Desired velocity
        vx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.x;
        vy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.y;
        vphi_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.angular.z;

        // Desired position
        x_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.x;
        y_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.y;
        
        // Desired orientation
        qw_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.w;
        qx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.x;
        qy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.y;
        qz_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.z;
        
        **/        

        //Trajectory Generation 

        des_pva.ax_des = traj_msg -> acceleration.linear.x;
        des_pva.ay_des = traj_msg -> acceleration.linear.y;
        des_pva.aphi_des = traj_msg -> acceleration.angular.z;
        
        des_pva.vx_des = traj_msg -> velocity.linear.x;
        des_pva.vy_des = traj_msg -> velocity.linear.y;
        des_pva.vphi_des = traj_msg ->velocity.angular.z;

        des_pva.x_des = traj_msg -> pose.position.x;
        des_pva.y_des = traj_msg -> pose.position.y;

        double qw = traj_msg ->pose.orientation.w;        
        double qx = traj_msg ->pose.orientation.x;
        double qy = traj_msg ->pose.orientation.y;
        double qz = traj_msg ->pose.orientation.z;

        qw = 1.0;
        qx = 0;
        qy = 0;
        qz = 0;

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        

        des_pva.phi_des = atan2(siny_cosp,cosy_cosp);
    }

    // Callback function 2 : State
    void callback_state(const Odometry::ConstPtr& state_msg){
        est_pos_vel.vx_est = state_msg->twist.twist.linear.x;
        est_pos_vel.vy_est = state_msg->twist.twist.linear.y;
        est_pos_vel.vphi_est = state_msg -> twist.twist.angular.z;

    }

    // Callback function 3 : Goal
    void callback_goal(const PoseStamped::ConstPtr& goal_msg){

        ROS_INFO("Goal Recieved");

        init_pos = false;

        goal_p.x_goal = goal_msg->pose.position.x;
        goal_p.y_goal = goal_msg->pose.position.y;

        double qw = goal_msg ->pose.orientation.w;        
        double qx = goal_msg ->pose.orientation.x;
        double qy = goal_msg ->pose.orientation.y;
        double qz = goal_msg ->pose.orientation.z;

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

        goal_p.phi_goal = atan2(siny_cosp,cosy_cosp);
    }

    private:

    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    ros::Subscriber subscriber_trajectory;
    ros::Subscriber subscriber_state;
    ros::Subscriber subscriber_goal;

    ros::Publisher publisher_cmd_vel;
    ros::Publisher publisher_desired_traj;
    ros::Publisher publisher_slam_pose;
    ros::Publisher publisher_error;

    bool init_pos = true;

    goal_pos goal_p;
    desired_pos_vel_acc des_pva;
    estimated_pos_vel est_pos_vel;
    error_pos_vel err_pos_vel; 
    command_vel_acc cmd_vel_acc;

    motor_vel cmd_motor_vel;

    // Clamping info : Limited motor vel = 6000 RPM
    const double motor_vel_lim = 6000; 

    // Controller gain
    const double Kp[3] = {100,100,100};
    const double Kd[3] = {80,80,80};
    // Specification of robot
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double gear_ratio = 74.5;
    const double radps_to_rpm = 60.0/2.0/M_PI;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l = l_a + l_b;

    // Time info
    double curr_time;
    double last_time = ros::Time::now().toSec();
    double dt = 1.0/200;
};
