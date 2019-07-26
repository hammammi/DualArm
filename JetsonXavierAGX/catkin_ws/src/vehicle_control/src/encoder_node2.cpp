#include <string>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "epos_tutorial/realVel.h"

#define Pi 3.14159265358979323846

/* Ecoder data (unit : rpm) */
double w[4];

/* Imu data */
double a[3];  // a[xyz][prev curr]
double gyro_yaw;
double q[4];
double dx;
double dy;



void encoderCallback(const epos_tutorial::realVel::ConstPtr& motors){

    w[0] = motors -> realVel[0];
    w[1] = motors -> realVel[1];
    w[2] = motors -> realVel[2];
    w[3] = motors -> realVel[3];

    w[0] = w[0];
    w[1] = -w[1];
    w[2] = w[2];
    w[3] = -w[3];

}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){

    a[0]      = imu -> linear_acceleration.x;
    a[1]      = imu -> linear_acceleration.y;

    q[0]     = imu -> orientation.w;
    q[1]     = imu -> orientation.x;
    q[2]     = imu -> orientation.y;
    q[3]     = imu -> orientation.z;
    gyro_yaw = imu -> angular_velocity.z;
}

int main(int argc,char** argv){

  ros::init(argc,argv,"state_publisher");
  ros::NodeHandle n;

  /* Encoder Subscriber*/
  ros::Subscriber enc_sub = n.subscribe("/measure",100,encoderCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu",100,imuCallback);


  /* Declare Odom Publisher and broadcasterTf */
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom",100);
  tf::TransformBroadcaster broadcaster;

  ros::Rate loop_rate(100);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  nav_msgs::Odometry odom;

  /* Declare variables to restore pose data */
  double x=0;
  double y=0;
  double phi=0;
  double phi0=0;
  double dx=0;
  double dy=0;
  double dphi=0;


  /* Time data */
  double dt=0.01;
  double curr_time=0;
  double last_time=0;
  int index=1;

  int index_angle = 1;

  /* Specification */
  double wheel_diameter = 0.1520;
  double wheel_radius = wheel_diameter / 2.0;
  double gear_ratio = 76.0;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;

  /* Conversion ratio */
  double rpm_to_rps = 2.0 * Pi / 60;


  while(ros::ok()){

	dt = 0.01;
    // Time //

    curr_time = ros::Time::now().toSec();
    dt = curr_time - last_time;

      // Time //
      if(index==1||dt==0||dt>1){
          dt=0.01;
          index=2;
      }
      last_time = curr_time;


//	ROS_INFO("w1 : %lf",w[0]);
    // angular vel : rpm --> rps // 

    w[0] = (double) w[0];
    w[1] = (double) w[1];
    w[2] = (double) w[2];
    w[3] = (double) w[3];

    w[0] =  w[0] / gear_ratio * rpm_to_rps;
    w[1] =  w[1] / gear_ratio * rpm_to_rps;
    w[2] =  w[2] / gear_ratio * rpm_to_rps;
    w[3] =  w[3] / gear_ratio * rpm_to_rps;

   dx = wheel_radius/4 * (w[0]+w[1]+w[2]+w[3]) * dt;
   dy = wheel_radius/4 * (-w[0]+w[1]+w[2]-w[3]) * dt;

  if(index_angle==1 && phi!=0){
   phi0 = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
   index_angle=2;
   }


   phi = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])) - phi0;

//   ROS_INFO("phi : %lf  phi0 : %lf  index : %d",phi,phi0,index_angle);
    ROS_INFO("w1 : %lf w2 : %lf w3 : %lf w4 : %lf ",w[0],w[1],w[2],w[3]);
    ROS_INFO("dy : %lf phi : %lf", dy, phi);
   x = x + dx * cosf(phi) - dy * sinf(phi);
   y = y + dx * sinf(phi) + dy * cosf(phi);

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




    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.rotation.w = qw;
    odom_trans.transform.rotation.x = qx;
    odom_trans.transform.rotation.y = qy;
    odom_trans.transform.rotation.z = qz;


    //filling the odometry
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;


    odom.twist.twist.linear.x = dx/dt;
    odom.twist.twist.linear.y = dy/dt;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.z = gyro_yaw;

    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
