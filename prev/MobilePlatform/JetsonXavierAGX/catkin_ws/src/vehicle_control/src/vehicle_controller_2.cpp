#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include "vehicle_control/motorMsg.h"
#include "epos_tutorial/realVel.h"
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>







#define PI 3.14159265358979323846

/* Declare Global Variables */

// From commend_msg //
double pos_des[3];

/* Ecoder data (unit : rpm) */
double w[4];

// From odometry msg //
double pos_act[2];
double vel_act[3];
double quat_act[4];

// From ground truth //
double pos_act2[2];
double quat_act2[4];
double vel_gnd[3];


double q[4];
double dx;
double dy;
double dt;
double curr_time=0;
double last_time=0;
double phi=0;
double phi0=0;

int index_angle = 1;
int iii = 0;

/* Specification */
double wheel_diameter = 0.1520;
double wheel_radius = wheel_diameter / 2.0;
double gear_ratio = 76.0;
double wheel_separation_a = 0.2600;
double wheel_separation_b = 0.2680;
double l = wheel_separation_a + wheel_separation_b;

/* Conversion ratio */
double rpm_to_rps = 2.0 * PI / 60;




void cmdCallback(const vehicle_control::commendMsg::ConstPtr& cmd_msg)
{
    pos_des[0]   = cmd_msg->xd;
    pos_des[1]   = cmd_msg->yd;
    pos_des[2]   = cmd_msg->phid;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){



    q[0]     = imu -> orientation.w;
    q[1]     = imu -> orientation.x;
    q[2]     = imu -> orientation.y;
    q[3]     = imu -> orientation.z;

    if(index_angle==1 && phi!=0){
        phi0 = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
        index_angle=2;
    }
    phi = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])) - phi0;
}




void encoderCallback(const epos_tutorial::realVel::ConstPtr& motors){

    if (iii==0){
        last_time = ros::Time::now().toSec();
    }
    iii += 1;
    w[0] = motors -> realVel[0];
    w[1] = motors -> realVel[1];
    w[2] = motors -> realVel[2];
    w[3] = motors -> realVel[3];

    w[0] = w[0];
    w[1] = -w[1];
    w[2] = w[2];
    w[3] = -w[3];

    w[0] = (double) w[0];
    w[1] = (double) w[1];
    w[2] = (double) w[2];
    w[3] = (double) w[3];

    w[0] =  w[0] / gear_ratio * rpm_to_rps;
    w[1] =  w[1] / gear_ratio * rpm_to_rps;
    w[2] =  w[2] / gear_ratio * rpm_to_rps;
    w[3] =  w[3] / gear_ratio * rpm_to_rps;

    curr_time = ros::Time::now().toSec();
    dt = curr_time - last_time;
    last_time = curr_time;
    dx = wheel_radius/4 * (w[0]+w[1]+w[2]+w[3])*dt;
    dy = wheel_radius/4 * (-w[0]+w[1]+w[2]-w[3])*dt;
}




void gndCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos_act2[0] = msg->pose.pose.position.x;
    pos_act2[1] = msg->pose.pose.position.y;

    quat_act2[0] = msg->pose.pose.orientation.x;
    quat_act2[1] = msg->pose.pose.orientation.y;
    quat_act2[2] = msg->pose.pose.orientation.z;
    quat_act2[3] = msg->pose.pose.orientation.w;

	vel_gnd[0] = msg->twist.twist.linear.x;
	vel_gnd[1] = msg->twist.twist.linear.y;
	vel_gnd[2] = msg->twist.twist.angular.z;

}




/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher publ_input = nh.advertise<vehicle_control::motorMsg>("/input_msg",1000);
  ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
//  ros::Publisher est_pos_pub = nh.advertise<vehicle_control::posMsgs>("/est_pose",100);
//  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom1",100);
//  ros::Publisher tf_pub = nh.advertise<vehicle_control::tfMsg>("/tf_pub",100);
//  ros::Publisher noise_pub = nh.advertise<vehicle_control::noise>("/noise",100);
//  ros::Publisher w_pub = nh.advertise<vehicle_control::wMsg>("/w_input",100);


//  ros::Publisher gnd_pub = nh.advertise<vehicle_control::gnd>("/gnd",100);


  // Quesize : 100 //

  ros::Subscriber sub1 = nh.subscribe("/ns1/cmd_msg",1000,cmdCallback);

  ros::Subscriber sub3 = nh.subscribe("/odom",100,gndCallback);
  ros::Subscriber enc_sub = nh.subscribe("/measure",100,encoderCallback);
  ros::Subscriber imu_sub = nh.subscribe("/imu",100,imuCallback);
  double radps_to_rpm = 60.0/2.0/PI;
  double rpm_to_radps = 2.0 * PI / 60;
//  ros::Subscriber sub3 = nh.subscribe("/gnd_truth",1,gndCallback,ros::TransportHints().unreliable().reliable().maxDatagramSize(1000).tcpNoDelay());
//  ros::Subscriber sub4 = nh.subscribe("/tf",100,tfCallback);

  // Publish rate : 100Hz //
  ros::Rate loop_rate(100);


/** Initialization for controller **/
  // reference {prev, current} //
  double x_d[2]      = {0, 0};
  double y_d[2]      = {0, 0};
  double phi_d[2]    = {0, 0};

  // Feedback data //
  double x_act[2]    = {0, 0};
  double y_act[2]    = {0, 0};
  double phi_act[2]  = {0, 0};
  double multiturn_phi_act[2] = {0,0};
  double n_phi=0;

  double vx_act[2]   = {0, 0};
  double vy_act[2]   = {0, 0};
  double dphi_act[2] = {0, 0};

  double x_quat   = 0;
  double y_quat   = 0;
  double z_quat   = 0;
  double w_quat   = 0;

  double angle[2] = {0,0};
  double multiturn_angle[2] = {0,0};
  double n_angle=0;
  // Output velocity //
  double w_act_curr[4] = {0,0,0,0};
  double w_act_prev[4] = {0,0,0,0};


  // Input(Velocity) //
  double del_s[2]={0,0};
  double vel_linear;

  double u_x = 0;
  double u_y = 0;
  double u_p = 0;

  // Motor speed in rad/sec - initialization {prev,curr}//
  double wheel_speed_lf = 0;
  double wheel_speed_rf = 0;
  double wheel_speed_lb = 0;
  double wheel_speed_rb = 0;

  // Motor speed in RPM - initialization //

  int w1 = 0;
  int w2 = 0;
  int w3 = 0;
  int w4 = 0;

//  // White gausian noise //
//  double del1=0;
//  double del2=0;
//  double del3=0;
//  double del4=0;


  // Time //
  double prev_t = 0;
  double dt = 0.01;
  double curr_t=0.01;


/** Controller gains Setting **/

  // P control //
  double kp_s = 1.0;
  double kp_phi = 10.0;

  // D control //
  double kd_s = 0.0;
  double kd_phi = 0.0;

  // I control //
  double ki_s = 0.0;
  double ki_phi = 0.0;

  // accumulated error //
  double Is = 0.0;
//  double Iphi = 0.0;



// Linear velocity : 0.2m/s , dphidt constraint :  10 deg/sec ( 0.174 rad/sec ), and scale factor  //
  double v_lim       = 0.6;
  double dphidt_lim  = 0.1;


  double x=0;
  double y=0;
  double dphi=0;


  while(ros::ok())
  {

//    geometry_msgs::Twist cmd_vel;
    vehicle_control::motorMsg input_msg;
//    vehicle_control::posMsgs pos;
//    vehicle_control::tfMsg tf;
//    vehicle_control::gnd gnd_truth;
//    vehicle_control::noise noise;
    geometry_msgs::Twist cmd_vel;
//    vehicle_control::wMsg w_input;


    x = x + dx * cos(phi) - dy * sin(phi);
    y = y + dx * sin(phi) + dy * cos(phi);

    geometry_msgs::Quaternion odom_quat;

    double cy = 1;
    double sy = 0;
    double cr = 1;
    double sr = 0;
    double cp = cos(phi * 0.5);
    double sp = sin(phi * 0.5);

    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;


    phi_act[1] = phi;

    if (phi_act[0]>(PI/2.0) && phi_act[1]<-(PI/2.0)){
    multiturn_phi_act[1] = phi_act[1] + PI + (2.0*n_phi+1)*PI;
    n_phi = n_phi + 1;
    }

    if (phi_act[0]<(-PI/2.0) && phi_act[1]>(PI/2.0)){
    multiturn_phi_act[1] = phi_act[1] - PI + (2.0*n_phi+1)*PI;
    n_phi = n_phi - 1;
    }

    if (phi_act[0]*phi_act[1]>=0){
    multiturn_phi_act[1] = phi_act[1] + 2.0*n_phi*PI;
    }


    if (phi_act[0]*phi_act[1]<=0){
    multiturn_phi_act[1] = phi_act[1] + 2.0*n_phi*PI;
    }







    x_act[1] = x;
    y_act[1] = y;

    // Current values from reference  phi --> rad//
    x_d[1]      = pos_des[0];
    y_d[1]      = pos_des[1];
    phi_d[1]    = pos_des[2];

    // Current values from Odometry  //

/*
    x_act[1]    = pos_act[0];
    y_act[1]    = pos_act[1];
*/
    vx_act[1]   = vel_act[0];
    vy_act[1]   = vel_act[1];
    dphi_act[1] = vel_act[2];
/*
    x_quat   = quat_act[0];
    y_quat   = quat_act[1];
    z_quat   = quat_act[2];
    w_quat   = quat_act[3];
*/

    /** Calculation of angles **/
/*
    phi_act[1]  = atan2(2.0 * (w_quat * z_quat + x_quat * y_quat),
                        1.0-2.0 * (y_quat*y_quat + z_quat * z_quat));
*/

    angle[1] = atan2(y_d[1]-y_act[1],x_d[1]-x_act[1]);


    if (angle[0]>(PI/2.0) && angle[1]<-(PI/2.0)){
    multiturn_angle[1] = angle[1] + PI + (2.0*n_angle+1)*PI;
    n_angle = n_angle + 1;
    }

    if (angle[0]<(-PI/2.0) && angle[1]>(PI/2.0)){
    multiturn_angle[1] = angle[1] - PI + (2.0*n_angle+1)*PI;
    n_angle = n_angle - 1;
    }

    if (angle[0]*angle[1]>=0){
    multiturn_angle[1] = angle[1] + 2.0*n_angle*PI;
    }


    if (angle[0]*angle[1]<=0){
    multiturn_angle[1] = angle[1] + 2.0*n_angle*PI;
    }







   /* PID control */

  del_s[1] = sqrt( (x_d[1] - x_act[1]) * (x_d[1] - x_act[1]) + (y_d[1] - y_act[1]) * (y_d[1] - y_act[1]));

  vel_linear = kp_s * del_s[1]; //+ kd_s * ( del_s[1] - del_s[0] ) / dt + ki_s * Is;

  u_p =  kp_phi * (phi_d[1]-multiturn_phi_act[1]);
        //+kd_phi * ((phi_d[1] - phi_d[0])-(phi_act[1] - phi_act[0]))/dt
//        +ki_phi * Iphi;




    if(vel_linear>v_lim)
    {
        vel_linear = v_lim;

    }

    if(dphidt_lim < u_p)
    {
      u_p = dphidt_lim;
    }
    else if(-dphidt_lim > u_p)
    {
      u_p = -dphidt_lim;

    }

    u_x = vel_linear * cos(angle[1]-phi_act[1]);
    u_y = vel_linear * sin(angle[1]-phi_act[1]);


//    ROS_INFO("u_x : %lf u_y : %lf u_phi : %lf",u_x,u_y,u_p);


    // Inverse Kinematics for motor input (uint : RPM) //

    w1 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y - l * u_p);
    w2 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y + l * u_p);
    w3 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y - l * u_p);
    w4 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y + l * u_p);

    // Model - motor's velocity //
    wheel_speed_lf = (double) w1 * rpm_to_radps;
    wheel_speed_rf = (double) w2 * rpm_to_radps;
    wheel_speed_lb = (double) w3 * rpm_to_radps;
    wheel_speed_rb = (double) w4 * rpm_to_radps;

    if(u_x * u_x + u_y * u_y + u_p * u_p < 0.01*0.01){
        w1 = 0;
        w2 = 0;
        w3 = 0;
        w4 = 0;

        u_x = 0;
        u_y = 0;
        u_p = 0;
    }


    // Forward Kinematics //
    u_x = wheel_radius / 4.0 * (wheel_speed_lf + wheel_speed_rf + wheel_speed_lb
        + wheel_speed_rb);

    u_y = wheel_radius / 4.0 * (-wheel_speed_lf + wheel_speed_rf + wheel_speed_lb- wheel_speed_rb);

    u_p = wheel_radius / ( 4.0 * l) * (-wheel_speed_lf + wheel_speed_rf -wheel_speed_lb + wheel_speed_rb);

    cmd_vel.linear.x  = u_x;
    cmd_vel.linear.y  = u_y;
    cmd_vel.angular.z = u_p;

    input_msg.omega1 = w1 * gear_ratio;
    input_msg.omega2 = -w2 * gear_ratio;
    input_msg.omega3 = w3 * gear_ratio;
    input_msg.omega4 = -w4 * gear_ratio;



    ctrl_pub.publish(cmd_vel);
    publ_input.publish(input_msg);


    ros::spinOnce();
    loop_rate.sleep();

        // Last values from reference  phi --> rad//
    x_d[0]      = x_d[1];
    y_d[0]      = y_d[1];
    del_s[0]    = del_s[1];
    phi_d[0]    = phi_d[1];

    // Last values from Odometry  //

    x_act[0]    = x_act[1];
    y_act[0]    = y_act[1];
    phi_act[0]  = phi_act[1];

    vx_act[0]   = vx_act[1];
    vy_act[0]   = vy_act[1];
    dphi_act[0] = dphi_act[1];


  }
  return 0;
}
