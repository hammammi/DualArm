
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <teb_local_planner/FeedbackMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mobile_control/motorMsg.h>
#include <sim_control/desiredMsg.h>
#include <tf/transform_listener.h>


#define _USE_MATH_DEFINES

using namespace teb_local_planner;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sim_control;

struct motor_vel{
    double w[4];
};


class tracking_controller{


    public:
    bool control_enable = false;
    bool is_init = false;
    // Robot index
    uint16_t robot_index = 0;

    // size of trajectory
    int size_of_traj;


    // Desired trajectory : acc, vel, position, orientation
    double time_start = 0;
    double time_from_start[3] = {1, 1, 1};
    double ax_des[3] = {0, 0, 0}, ay_des[3] = {0, 0, 0}, aphi_des[3] = {0, 0, 0};
    //double vx_des[3] = {0, 0, 0}, vy_des[3] = {0, 0, 0}, vphi_des[3] = {0, 0, 0};
    double vx_des = 0, vy_des = 0, vphi_des = 0;
    double x_des[3] = {0, 0, 0}, y_des[3] = {0, 0, 0};
    // phi_des[0] : previous desired yaw angle
    // phi_des : current desired yaw angle
    double phi_des[3] = {0, 0, 0};

    double x_goal = 0, y_goal = 0;
    double phi_goal = 0;

    // Robot state : vel, position, orientation
    double vx_robot = 0, vy_robot = 0, vphi_robot = 0;
    double x_robot = 0, y_robot = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    // phi_robot[0] : previous robot yaw angle
    // phi_robot : current robot yaw angle
    double phi_robot = 0;

    // Time info
    double current_time = 0;
    double last_time = 0;
    double dt = 0;

    // Controller Gain
    const double K_v[3] = {2.0, 2.0, 1.0};
    const double K_p[3] = {2.0, 2.0, 1.0};
    // Error : linear vel, angular vel, position, orientation
    double vx_err = 0, vy_err = 0, vphi_err = 0;
    double x_err = 0, y_err = 0, phi_err = 0;

    // Commend velocity in the global frame
    double ax_cmd = 0, ay_cmd = 0, aphi_cmd = 0;
    double vx_cmd = 0, vy_cmd = 0, vphi_cmd = 0; 

    // Commend velocity in the robot frame
    double vx_cmd_robot = 0, vy_cmd_robot = 0, vphi_cmd_robot = 0;

    // Clamping info : Limited motor vel = 6000 RPM
    const int motor_vel_lim = 6000; 

    // Commend motor velocity
    motor_vel cmd_motor_vel;

    // Specification of robot
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double gear_ratio = 74.5;
    const double radps_to_rpm = 60.0/2.0/M_PI;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    const double l_a = 0.2170;
    const double l_b = 0.16875;
    const double l = l_a + l_b;

    // Publisher Declaration
    void cmd_vel_pub_setting(){
        publisher_desired_traj = nh_.advertise<desiredMsg>("/des_traj",1);
        publisher_cmd_vel = nh_.advertise<mobile_control::motorMsg>("/input_msg",1);
    }


    // Subscriber Declaration
    void subscriber_declaration(){
        // Subscribe
        subscriber_state = nh_.subscribe("/odom",1,&tracking_controller::callback_state,this);
        subscriber_trajectory = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback",1,&tracking_controller::callback_traj,this);
        subscriber_goal = nh_.subscribe("/move_base_simple/goal",1,&tracking_controller::callback_goal,this);
        subscriber_vel = nh_.subscribe("/cmd_vel",1,&tracking_controller::callback_vel,this);
    }

    // Publish commend motor velocity
    void cmd_vel_pub(){
      tf::StampedTransform transform;
      if(!is_init){
        try{
          time_start = ros::Time::now().toSec();
          listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
          x_des[0] = transform.getOrigin().x();
          y_des[0] = transform.getOrigin().y();
          x_des[1] = x_des[0];
          y_des[1] = y_des[0];
          x_des[2] = x_des[0];
          y_des[2] = y_des[0];
          tf::Quaternion quat = transform.getRotation();
          phi_des[0] = tf::getYaw(quat);
          if (phi_des[0] > M_PI) {
              phi_des[0] -= 2*M_PI;
          }
          phi_des[1] = phi_des[0];
          phi_des[2] = phi_des[0];
          x_goal = x_des[0];
          y_goal = y_des[0];
          phi_goal = phi_des[0];
          is_init=true;
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

      }
      if(control_enable){

        try{
          listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
          x_robot = transform.getOrigin().x();
          y_robot = transform.getOrigin().y();
          tf::Quaternion quat = transform.getRotation();
          phi_robot = tf::getYaw(quat);
          if (phi_robot > M_PI) {
              phi_robot -= 2*M_PI;
          }
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        // Vel error
        //vx_err   = (vx_des - vx_robot);
        //vy_err   = (vy_des - vy_robot);
        //vphi_err = vphi_des - vphi_robot;

        // Position and orientation error
        double current_time_from_start = ros::Time::now().toSec() - time_start;
        unsigned int idx = 1;
        while ((current_time_from_start > time_from_start[idx])&&(idx < 2))
        {
          ++idx;
        }
        double lambda = (current_time_from_start-time_from_start[idx-1]) / (time_from_start[idx]-time_from_start[idx-1]);
        if (lambda > 1.0)
        {
          lambda = 1.0;
        }
        double current_x_des = (1-lambda)*x_des[idx-1] + lambda*x_des[idx];
        double current_y_des = (1-lambda)*y_des[idx-1] + lambda*y_des[idx];
        double cos_phi_des = (1-lambda)*cos(phi_des[idx-1]) + lambda*cos(phi_des[idx]);
        double sin_phi_des = (1-lambda)*sin(phi_des[idx-1]) + lambda*sin(phi_des[idx]);
        double current_phi_des = atan2(sin_phi_des, cos_phi_des);
        x_err    = (current_x_des - x_robot)*cos(phi_robot) - (current_y_des - y_robot)*sin(phi_robot);
        y_err    = (current_x_des - x_robot)*sin(phi_robot) + (current_y_des - y_robot)*cos(phi_robot);
        phi_err  = current_phi_des - phi_robot;
        if (phi_err > M_PI) {
            phi_err -=2*M_PI;
        }
        else if (phi_err <= -M_PI)
        {
            phi_err += 2*M_PI;
        }
        //double current_vx_des = (1-lambda)*vx_des[idx-1] + lambda*vx_des[idx];
        //double current_vy_des = (1-lambda)*vy_des[idx-1] + lambda*vy_des[idx];
        //double current_vphi_des = (1-lambda)*vphi_des[idx-1] + lambda*vphi_des[idx];
        vx_cmd = vx_des + K_p[0] * x_err;
        vy_cmd = vy_des + K_p[1] * y_err;
        vphi_cmd = vphi_des + K_p[2] * phi_err;
        //ROS_INFO("x_err: %lf y_err: %lf phi_err: %lf",x_err,y_err,phi_err);

        vx_cmd_robot = vx_cmd;
        vy_cmd_robot = vy_cmd;
        vphi_cmd_robot = vphi_cmd;

        //if(fabs(x_goal-x_robot)<0.2 && fabs(y_goal-y_robot)<0.2 && fabs(phi_goal-phi_robot)<1.0){
            
            //vx_cmd_robot = K_p[0]*x_err;
            //vy_cmd_robot = K_p[1]*y_err;
            //vphi_cmd_robot = K_p[2]*phi_err;
            ////ROS_INFO("Near Goal Position : x_err : %lf y_err : %lf phi_err : %lf",x_err,y_err,phi_err);
            ////ROS_INFO("vphi_cmd_robot : %lf",vphi_cmd_robot);

        //}

        if(fabs(x_goal-x_robot)<0.05 && fabs(y_goal-y_robot)<0.05 && fabs(phi_goal-phi_robot)<0.1){
           control_enable = false;           
           vx_cmd_robot = 0;
           vy_cmd_robot = 0;
           vphi_cmd_robot = 0;
            //ROS_INFO("Near Goal Position : x_err : %lf y_err : %lf phi_err : %lf",x_err,y_err,phi_err);
            //ROS_INFO("vphi_cmd_robot : %lf",vphi_cmd_robot);
        }

        // Inverse Kinematics
        cmd_motor_vel = inverse_kinematics();

        mobile_control::motorMsg motor_vel;


        // Unit conversion
        cmd_motor_vel.w[0] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[0];
        cmd_motor_vel.w[1] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[1];
        cmd_motor_vel.w[2] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[2];
        cmd_motor_vel.w[3] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[3];
                     
        cmd_motor_vel = clamping(cmd_motor_vel);

        motor_vel.omega1 = (int) cmd_motor_vel.w[0];
        motor_vel.omega2 = (int) cmd_motor_vel.w[1];
        motor_vel.omega3 = (int) cmd_motor_vel.w[2];
        motor_vel.omega4 = (int) cmd_motor_vel.w[3];



        publisher_cmd_vel.publish(motor_vel);


      } 

    }

    void desired_traj(){
        desiredMsg desired_traj_msg;
        
            desired_traj_msg.x_des = x_des[0];
            desired_traj_msg.y_des = y_des[0];

            //desired_traj_msg.vx_des = vx_des[0];
            //desired_traj_msg.vy_des = vy_des[0];
            desired_traj_msg.vx_des = vx_des;
            desired_traj_msg.vy_des = vy_des;

            desired_traj_msg.phi_des = phi_des[0];

        publisher_desired_traj.publish(desired_traj_msg);

    }

    motor_vel inverse_kinematics(){
        motor_vel m;

        m.w[0] = (int) 1.0/r * vx_cmd_robot - 1.0/r * vy_cmd_robot - l/r * vphi_cmd_robot;
        m.w[1] = (int) 1.0/r * vx_cmd_robot + 1.0/r * vy_cmd_robot + l/r * vphi_cmd_robot;
        m.w[2] = (int) 1.0/r * vx_cmd_robot - 1.0/r * vy_cmd_robot + l/r * vphi_cmd_robot;
        m.w[3] = (int) 1.0/r * vx_cmd_robot + 1.0/r * vy_cmd_robot - l/r * vphi_cmd_robot;

        return m;
    }

    motor_vel clamping(motor_vel clamped_motor_vel){

        double arr[4] = {clamped_motor_vel.w[0],clamped_motor_vel.w[1],clamped_motor_vel.w[2],clamped_motor_vel.w[3]};

        double vx_;
        double vy_;
        double vphi_;

        double motor_vel_max = 0;
        std::sort(arr,arr+4,std::greater<double>());
        motor_vel_max = arr[0];
        
        if(fabs(motor_vel_max)>motor_vel_lim){
            clamped_motor_vel.w[0] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[0];
            clamped_motor_vel.w[1] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[1];
            clamped_motor_vel.w[2] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[2];
            clamped_motor_vel.w[3] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[3];
        }
        
        vx_ = r/4.0 * (clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vy_ = r/4.0 * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] - clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vphi_ = r/4.0/l * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] - clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        
        //ROS_INFO("vx_cmd_robot : %lf vy_cmd_robot : %lf vphi_cmd_robot : %lf",vx_,vy_,vphi_);
        //ROS_INFO("Robot vel - vx : %lf, vy : %lf vphi : %lf",vx_,vy_,vphi_);
        //ROS_INFO("vphi_ : %lf vphi_des : %lf phi_err : %lf",vphi_,vphi_des,phi_err);
        return clamped_motor_vel;
        
    }



    // Callback function 1 : Trajectory from planner
    void callback_traj(const FeedbackMsg::ConstPtr& traj_msg){
        
        // Robot index
        robot_index = traj_msg -> selected_trajectory_idx;
        
        size_of_traj = traj_msg ->trajectories[robot_index].trajectory.size();

        //Time
        time_start = ros::Time::now().toSec();
        for (unsigned int traj_index=0; traj_index < 3; ++traj_index)
        {
          time_from_start[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].time_from_start.toSec();

          // Desired acceleration
          //ax_des[traj_index] =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.x;
          //ay_des[traj_index] =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.y;
          //aphi_des[traj_index] =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.angular.z;
       

          // Desired velocity
          //vx_des[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.x;
          //vy_des[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.y;
          //vphi_des[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.angular.z;

          // Desired position
          x_des[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.x;
          y_des[traj_index] = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.y;
          
          // Desired orientation
          double qw_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.w;
          double qx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.x;
          double qy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.y;
          double qz_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.z;
          

          // Conversion from Quaternion to euler angle
          double siny_cosp = 2.0 * (qw_des*qz_des + qx_des*qy_des);
          double cosy_cosp = 1 - 2.0 * (qy_des*qy_des + qz_des*qz_des);
          phi_des[traj_index] = atan2(siny_cosp,cosy_cosp);
        }

    }

    // Callback function 2 : State
    void callback_state(const Odometry::ConstPtr& state_msg){
        
        // Robot Velocity 
        vx_robot = state_msg -> twist.twist.linear.x;
        vy_robot = state_msg -> twist.twist.linear.y;
        vphi_robot = state_msg -> twist.twist.angular.z;

        // Robot position
        x_robot = state_msg -> pose.pose.position.x;
        y_robot = state_msg -> pose.pose.position.y;
        
        // Robot orientation
        qw = state_msg -> pose.pose.orientation.w;
        qx = state_msg -> pose.pose.orientation.x;        
        qy = state_msg -> pose.pose.orientation.y;
        qz = state_msg -> pose.pose.orientation.z;

        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw*qz + qx*qy);
        double cosy_cosp = 1 - 2.0 * (qy*qy + qz*qz);
        phi_robot = atan2(siny_cosp,cosy_cosp);
    }

    void callback_vel(const geometry_msgs::Twist::ConstPtr& vel_msg){
        vx_des = vel_msg->linear.x;
        vy_des = vel_msg->linear.y;
        vphi_des = vel_msg->angular.z;
    }

    void callback_goal(const PoseStamped::ConstPtr& goal_msg){
        control_enable = true;
        x_goal = goal_msg->pose.position.x;
        y_goal = goal_msg->pose.position.y;

        double qw = goal_msg ->pose.orientation.w;        
        double qx = goal_msg ->pose.orientation.x;
        double qy = goal_msg ->pose.orientation.y;
        double qz = goal_msg ->pose.orientation.z;
        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw*qz + qx*qy);
        double cosy_cosp = 1 - 2.0 * (qy*qy + qz*qz);
        phi_goal = atan2(siny_cosp,cosy_cosp);
        ROS_INFO("x_goal: %lf y_goal: %lf phi_goal: %lf",x_goal,y_goal,phi_goal);
    }


    private:

    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    ros::Subscriber subscriber_trajectory;
    ros::Subscriber subscriber_state;
    ros::Subscriber subscriber_goal;
    ros::Subscriber subscriber_vel;

    ros::Publisher publisher_cmd_vel;
    ros::Publisher publisher_desired_traj;
            
    

};