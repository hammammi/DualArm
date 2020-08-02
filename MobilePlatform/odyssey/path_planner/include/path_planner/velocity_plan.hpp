// Rotation planning ignored
// Only take translational motion into account
#include "ros/ros.h"
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>

#include <math.h>

#define _USE_MATH_DEFINES

using namespace teb_local_planner;
using namespace geometry_msgs;
using namespace nav_msgs;

class velocity_plan{

    public:

    int robot_index = 0;
    int traj_index = 1;

    // angular acceleration limit and velocity limit
    const double motor_acc_lim = 1.0;
    //const double motor_acc_lim = 0.92; 
    double motor_vel_lim = 4.21;
    //double motor_vel_lim = 7.7;
    // Robot vel lim and acc lim
    double vel_lim = 0, acc_lim = 0;
    
    // goal position
    double x_goal = 0, y_goal = 0;

    // initial position
    double x_init = 0, y_init = 0;
    bool is_init_position_updated = false;
    bool goal_reached_ = false;
    // desired position and velocity
    double linear_vel = 0, linear_acc = 0;
    double x_d = 0, y_d = 0;
    double vx_d = 0, vy_d = 0;
    double ax_d = 0, ay_d = 0;

    double x_odom = 0, y_odom = 0;

    // Velocity direction alpha
    double alpha = 0;

    // wheel radious
    const double r = 0.1520/2.0;

    // Time info
    double t_curr = 0;
    double t_last = 0;
    double dt = 0;
    double t = 0; // Time for trajectory generation

    // planning time info
    double t_1 = 0;
    double t_2 = 0;
    double t_final = 0;
    double distance = 0;


    void subscriber_setting(){
        subscriber_goal = nh_.subscribe("/move_base_simple/goal",1,&velocity_plan::callback_goal,this);
        subscriber_odom = nh_.subscribe("/camera/odom/sample",1,&velocity_plan::callback_odom,this);
    }

    void callback_goal(const PoseStamped::ConstPtr& goal_msg){
        x_goal = goal_msg -> pose.position.x;
        y_goal = goal_msg -> pose.position.y;
        ROS_INFO("Goal position - x : %lf y : %lf",x_goal,y_goal);
    }

    void callback_odom(const Odometry::ConstPtr& odom_msg){
        x_odom = odom_msg -> pose.pose.position.x;
        y_odom = odom_msg -> pose.pose.position.y;
    }

    void publisher_setting(){
        publisher_traj = nh_.advertise<TrajectoryPointMsg>("/move_base/TebLocalPlannerROS/teb_feedback",10);
    }

    bool goal_reached(){
        goal_reached_ = false;
        if(x_d == x_goal && y_d == y_goal){
            goal_reached_ = true;
        }
        return goal_reached_;
    }

    void time_calculation(){
    
        ROS_INFO("x_goal : %lf y_goal : %lf",x_goal,y_goal);
        alpha = atan2((y_goal - y_init),(x_goal - x_init));
        vel_lim = r * motor_vel_lim / fabs(fabs(sin(alpha)) + fabs(cos(alpha)));
        acc_lim = r * motor_acc_lim / fabs(fabs(sin(alpha)) + fabs(cos(alpha)));
        ROS_INFO("alpha : %lf",alpha);
    

        t_1 = vel_lim/acc_lim;
        distance = sqrt((x_goal - x_init)*(x_goal - x_init) + (y_goal - y_init)*(y_goal - y_init));
        t_2 = distance / vel_lim;
        t_final = t_2 + t_1;

        if(t_1>=t_2){
            vel_lim = vel_lim / 2.0;
            t_1 = t_1 * 0.5;
            t_2 = t_2 * 2.0;
            t_final = t_1 + t_2;
        }
        //ROS_INFO("T final : %lf t_1 : %lf t_2 : %lf",t_final,t_1,t_2);
        
    }

    void traj_generation(){
        t_curr = ros::Time::now().toSec();
        if(t_last == 0){
            dt = 0;
        }else
        {
            dt = t_curr - t_last;
        }

        
        t_last = t_curr;
        ROS_INFO("Time : %lf",t);
        t += dt;
        

        if(t < t_1){

            ax_d = acc_lim * cos(alpha);
            ay_d = acc_lim * sin(alpha);

            vx_d += ax_d * dt;
            vy_d += ay_d * dt;

            x_d += vx_d * dt + 0.5 * ax_d * dt * dt;
            y_d += vy_d * dt + 0.5 * ay_d * dt * dt;
        }

        if(t>t_1 && t<t_2){
            
            ax_d = 0;
            ay_d = 0;

            linear_vel = vel_lim;
            vx_d = vel_lim * cos(alpha);
            vy_d = vel_lim * sin(alpha);

            x_d += vx_d*dt;
            y_d += vy_d*dt;

        }

        if(t>t_2 && t<t_final){

            ax_d = -acc_lim * cos(alpha);
            ay_d = -acc_lim * sin(alpha);

            vx_d += ax_d * dt;
            vy_d += ay_d * dt;

            x_d += vx_d * dt - 0.5 * ax_d * dt * dt;
            y_d += vy_d * dt - 0.5 * ay_d * dt * dt;
        }

        if(t>t_final){
            ROS_INFO("Stop");
            ax_d = 0;
            ay_d = 0;

            vx_d = 0;
            vy_d = 0;

            x_d = x_goal;
            y_d = y_goal;

            x_init = x_goal;
            y_init = y_goal;
            ROS_INFO("x_init : %lf , y_init : %lf",x_init,y_init);
        }

        ROS_INFO("x_d : %lf y_d : %lf",x_d,y_d);
        ROS_INFO("vx_d : %lf vy_d : %lf",vx_d,vy_d);

        TrajectoryPointMsg traj_msg;

        traj_msg.acceleration.linear.x = ax_d;
        traj_msg.acceleration.linear.y = ay_d;
        traj_msg.acceleration.angular.z = 0;

        traj_msg.velocity.linear.x = vx_d;
        traj_msg.velocity.linear.y = vy_d;
        traj_msg.velocity.angular.z = 0;

        traj_msg.pose.position.x = x_d;
        traj_msg.pose.position.y = y_d;

        traj_msg.pose.orientation.w = 1.0;
        traj_msg.pose.orientation.x = 0.0;
        traj_msg.pose.orientation.y = 0.0;
        traj_msg.pose.orientation.z = 0.0;
        
        publisher_traj.publish(traj_msg);

    }


    private:

    ros::NodeHandle nh_;
    
    ros::Subscriber subscriber_goal;
    ros::Subscriber subscriber_odom;
    ros::Publisher publisher_traj; 

};
