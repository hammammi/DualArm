#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include "vehicle_control/motorMsg.h"
#include "vehicle_control/wMsg.h"
#include "vehicle_control/posMsgs.h"
//#include "vehicle_control/tfMsg.h"
//#include "vehicle_control/gnd.h"

#include <nav_msgs/Odometry.h>
#include "vehicle_control/positionMsg.h"
#include <geometry_msgs/Twist.h>
//#include <tf/tfMessage.h>



/* for white gaussian */
//#include <iostream>
//#include <iterator>
//#include <random>
//#include "vehicle_control/noise.h"
//#include "vehicle_control/wMsg.h"




#define PI 3.14159265358979323846

/* Declare Global Variables */

// From commend_msg //
double pos_des[3];

// From odometry msg //
double pos_act[2];
double vel_act[3];
double quat_act[4];

// From ground truth //
double pos_act2[2];
double quat_act2[4];
double vel_gnd[3];

// From tf msg //

double pose_est1[2];
double pose_est2[2];
double pose_est3[2];

double pose_est[2];

double quat_est1[4];
double quat_est2[4];
double quat_est3[4];

double phi_est;

/* White Gaussian noise generator to reflect slip effect */
//double AGN(double mean, double stddev)
//{
//    double result;
//    std::mt19937 generator(std::random_device{} ());
//    std::normal_distribution<double> dist(mean,stddev);
//    result = dist(generator);
//    return result;
//}


/* Callback function to subscribe */

void cmdCallback(const vehicle_control::commendMsg::ConstPtr& cmd_msg)
{
    pos_des[0]   = cmd_msg->xd;
    pos_des[1]   = cmd_msg->yd;
    pos_des[2]   = cmd_msg->phid;
}

void odomCallback(const vehicle_control::positionMsg::ConstPtr& msg)
{
    pos_act2[0] = msg->x;
    pos_act2[1] = msg->y;

    ROS_INFO("x : %lf y : %lf",pos_act2[0],pos_act2[1]);
    quat_act2[0] = msg->qx;
    quat_act2[1] = msg->qy;
    quat_act2[2] = msg->qz;
    quat_act2[3] = msg->qw;
}




/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher publ_input = nh.advertise<vehicle_control::motorMsg>("/input_msg",1);
  ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
//  ros::Publisher est_pos_pub = nh.advertise<vehicle_control::posMsgs>("/est_pose",100);
//  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom1",100);
//  ros::Publisher tf_pub = nh.advertise<vehicle_control::tfMsg>("/tf_pub",100);
//  ros::Publisher noise_pub = nh.advertise<vehicle_control::noise>("/noise",100);
//  ros::Publisher w_pub = nh.advertise<vehicle_control::wMsg>("/w_input",100);


//  ros::Publisher gnd_pub = nh.advertise<vehicle_control::gnd>("/gnd",100);


  // Quesize : 100 //

  ros::Subscriber sub1 = nh.subscribe("/ns1/cmd_msg",1,cmdCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom",1,odomCallback);
//  ros::Subscriber sub3 = nh.subscribe("/odom",100,gndCallback);

//  ros::Subscriber sub3 = nh.subscribe("/odom2",1,gndCallback,ros::TransportHints().unreliable().reliable().maxDatagramSize(1000).tcpNoDelay());
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
  double dphidt_lim  = 0.174;

/* Wheel specification - unit: meter */
  double wheel_diameter = 0.152;
  double wheel_radius = wheel_diameter / 2.0;
  double wheel_separation_a = 0.2570;
  double wheel_separation_b = 0.2810;
  double l = wheel_separation_a + wheel_separation_b;


// Gear ratio //

  double gear_ratio = 73.5;

// radps_to_rpm : rad/sec --> rpm //
// rpm_to_radps : rpm --> rad/sec //

  double radps_to_rpm = 60.0/2.0/PI;
  double rpm_to_radps = 2.0 * PI / 60;





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



//
//  double phi_est1  = atan2(2.0 * (quat_est1[3] * quat_est1[2] + quat_est1[0] * quat_est1[1]),
//                        1.0-2.0 * (quat_est1[1] * quat_est1[1] + quat_est1[2] * quat_est1[2]));
//
//  double phi_est2  = atan2(2.0 * (quat_est2[3] * quat_est2[2] + quat_est2[0] * quat_est2[1]),
//                        1.0-2.0 * (quat_est2[1] * quat_est2[1] + quat_est2[2] * quat_est2[2]));
//
//  double phi_est3  = atan2(2.0 * (quat_est3[3] * quat_est3[2] + quat_est3[0] * quat_est3[1]),
//                        1.0-2.0 * (quat_est3[1] * quat_est3[1] + quat_est3[2] * quat_est3[2]));



 
/************SLAM data feedback***************/

//  phi_act[1] = phi_est1 + phi_est2 + phi_est3;
/*********************************************/


/************Gnd data feedback ***************/
    phi_act[1] =  atan2(2.0 * (quat_act2[3] * quat_act2[2] + quat_act2[0] * quat_act2[1])
    ,1.0-2.0 * (quat_act2[1] * quat_act2[1] + quat_act2[2] * quat_act2[2]));


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





/**********************************************/
//
//  double alp2 = phi_est1;
//  double alp3 = phi_est1+phi_est2;
//
//  double  x_tf2 = pose_est2[0] * cos(alp2) - pose_est2[1] * sin(alp2);
//  double  y_tf2 = pose_est2[1] * sin(alp2) + pose_est2[1] * cos(alp2);
//
//  double  x_tf3 = pose_est3[0] * cos(alp3) - pose_est3[1] * sin(alp3);
//  double  y_tf3 = pose_est3[0] * sin(alp3) + pose_est3[1] * cos(alp3);


/********** SLAM data feedback ****************/
//  x_act[1] = pose_est1[0] + x_tf2 + x_tf3;
//  y_act[1] = pose_est1[1] + y_tf2 + y_tf3;


/**********************************************/

/************GND truth feedback **************/

    x_act[1] = pos_act2[0];
    y_act[1] = pos_act2[1];

/**********************************************/

/************ SLAM data **********************/

//    double x_est =  pose_est1[0] + x_tf2 + x_tf3;
//    double y_est =  pose_est1[1] + y_tf2 + y_tf3;
//    double phi_est = phi_est1 + phi_est2 + phi_est3;

/********************************************/

/*
    epos_tutorial::DesiredVel input_msg;
*/
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


    // Slip model to simulator//

  // wheel left forward slip //

//    float mean1 = fabs(0.2*wheel_speed_lf)/8.5;
//    float mean2 = fabs(0.2*wheel_speed_rf)/8.5;
//    float mean3 = fabs(0.2*wheel_speed_lb)/8.5;
//    float mean4 = fabs(0.2*wheel_speed_rb)/8.5;
//
//    del1 = AGN(mean1,0.01);
//    del2 = AGN(mean2,0.01);
//    del3 = AGN(mean3,0.01);
//    del4 = AGN(mean4,0.01);
//
//    del1 = del1>0 ? del1:0;
//    del2 = del2>0 ? del2:0;
//    del3 = del3>0 ? del3:0;
//    del4 = del4>0 ? del4:0;
//
//    if(del1!=0){
//    del1 = del1>mean1 ? mean1:del1;
//    }
//
//    if(del2!=0){
//    del2 = del2>mean2 ? mean2:del2;
//    }
//    if(del3!=0){
//    del3 = del3>mean3 ? mean3:del3;
//    }
//    if(del4!=0){
//    del4 = del4>mean4 ? mean4:del4;
//    }

//    ROS_INFO("%lf ",wheel_speed_lb);
//    wheel_speed_lf = wheel_speed_lf*(1 - del1);
//
//
//  // wheel right forward //
//    wheel_speed_rf = wheel_speed_rf*(1-del2);
//
//
//  // wheel left backward //
//   wheel_speed_lb = wheel_speed_lb*(1-del3) ;
//
//
//  // wheel right forward //
//  wheel_speed_rb = wheel_speed_rb*(1-del4);
//
//    if(w1==0){
//      wheel_speed_lf=0;
//    }
//    if(w2==0){
//      wheel_speed_rf=0;
//    }
//    if(w3==0){
//      wheel_speed_lb=0;
//    }
//    if(w4==0){
//      wheel_speed_rb=0;
//    }



//    w_input.w_input1 = wheel_speed_lf;
//    w_input.w_input2 = wheel_speed_rf;
//    w_input.w_input3 = wheel_speed_lb;
//    w_input.w_input4 = wheel_speed_rb;

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

//    pos.x_est = x_est;
//    pos.y_est = y_est;
//    pos.phi_est = phi_est;

//    tf.x_tf1   = pose_est1[0];
//    tf.y_tf1   = pose_est1[1];
//    tf.phi_tf1 = phi_est1;
//
//
//    tf.x_tf2 = pose_est2[0];
//    tf.y_tf2 = pose_est2[1];
//    tf.phi_tf2 = phi_est2;
//
//
//    tf.x_tf3 = pose_est3[0];
//    tf.y_tf3 = pose_est3[1];
//    tf.phi_tf3 = phi_est3;

/*
    input_msg.vel1 = w1;
    input_msg.vel2 = w2;
    input_msg.vel3 = w3;
    input_msg.vel4 = w4;

*/

//    gnd_truth.x_gnd = pos_act2[0];
//    gnd_truth.y_gnd = pos_act2[1];
//    gnd_truth.q_x = quat_act2[0];
//    gnd_truth.q_y = quat_act2[1];
//    gnd_truth.q_z = quat_act2[2];
//    gnd_truth.q_w = quat_act2[3];
//	gnd_truth.vel_x = vel_gnd[0];
//	gnd_truth.vel_y = vel_gnd[1];
//	gnd_truth.vel_phi=vel_gnd[2];


//    noise.noise1 = del1;
//    noise.noise2 = del2;
//    noise.noise3 = del3;
//    noise.noise4 = del4;

  
    ctrl_pub.publish(cmd_vel);
    publ_input.publish(input_msg);
//    est_pos_pub.publish(pos);
//    tf_pub.publish(tf);
//    gnd_pub.publish(gnd_truth);
//    noise_pub.publish(noise);
//    w_pub.publish(w_input);
//    listener_pub.publish(pos);


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

//    // Accumulated error update //
//    Is = Is + (del_s[1]-del_s[0])*dt;
//    Iphi = Iphi + (phi_d[1] - phi_act[1])*dt;
  }
  return 0;
}
