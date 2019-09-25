#include <string>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "vehicle_control/motorsMsg.h"

#define Pi 3.14159265358979323846

/* Ecoder data (unit : rpm) */
double w[4];

/* Imu data */
double a[3];  // a[xyz]
double yaw_rate;
double q[4];

/* Magnetometer (unit : Gause) */
double m[3];

/* velocity */
double vel[3];

void encoderCallback(const vehicle_control::motorsMsg::ConstPtr& motors){

    w[0] = motors -> omega1;
    w[1] = motors -> omega2;
    w[2] = motors -> omega3;
    w[3] = motors -> omega4;

    w[0] = -w[0];
    w[2] = -w[2];
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){

    a[0]      = imu -> linear_acceleration.x;
    a[1]      = imu -> linear_acceleration.y;

    q[0]     = imu -> orientation.x;
    q[1]     = imu -> orientation.y;
    q[2]     = imu -> orientation.z;
    q[3]     = imu -> orientation.w;
    yaw_rate = imu -> angular_velocity.z;
}

void mgnCallback(const geometry_msgs::Vector3Stamped::ConstPtr& mgn){
   m[0] = mgn -> vector.x;
   m[1] = mgn -> vector.y;
   m[2] = mgn -> vector.z;
}

void gndCallback(const nav_msgs::Odometry::ConstPtr& v){
    vel[0] = v -> twist.twist.linear.x;
    vel[1] = v -> twist.twist.linear.y;

}




int main(int argc,char** argv){

  ros::init(argc,argv,"state_publisher");
  ros::NodeHandle n;

  /* Encoder Subscriber*/
  ros::Subscriber enc_sub = n.subscribe("/input_msg",100,encoderCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu",100,imuCallback);
  ros::Subscriber mgn_sub = n.subscribe("/magnet",100,mgnCallback);
  ros::Subscriber gnd_sub = n.subscribe("/gnd_truth",100,gndCallback);


  // Declare Odom Publisher and broadcasterTf //
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

  /* Time data */
  double dt=0.01;
  double curr_time=0.01;
  double last_time=0;



  // Specification //
  double wheel_diameter = 0.1520;
  double wheel_radius = wheel_diameter / 2.0;
  double gear_ratio = 76.0;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;

  // Conversion ratio //
  double rpm_to_rps = 2.0 * Pi / 60;

        // Matrix for Kalman filter //
//	double A[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
//	double H[3][3] = {{1,0,0,},{0,1,0},{0,0,1}};
        double B[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

        double P[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
        double P_temp[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
        double P_last[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

        double K[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

        // State variables //
        double x_est[3] = {0,0,0};
        double x_est_last[3] = {0,0,0};
        double x_est_temp[3] = {0,0,0};

        // Measurement and control input //
        double z_meas[3] = {0,0,0};
        double u[3] = {0,0,0};

        double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

        // Covariance Matrices Q : system noise, R : sensor noise//
        double Q[3][3] = {{0.09,0,0},{0,0.09,0},{0,0,0.000036}};
        double R[3][3] = {{0.01,0,0},{0,0.01,0},{0,0,0.0001}};

        int index = 1;

        // Slip Reflection ratio //

        double alp = 1;


  while(ros::ok()){

   // Time //
    curr_time = ros::Time::now().toSec();
        dt = curr_time - last_time;
        if(index == 1 || dt == 0 || dt > 1){
        dt = 0.01;
        index=2;
        }
    last_time = curr_time;

//  ROS_INFO("dt : %lf curr time : %lf last time : %lf",dt,curr_time,last_time);
//  ROS_INFO("K0 : %lf  %lf %lf ",K[0][0],K[0][1],K[0][2]);
//  ROS_INFO("K1 : %lf  %lf %lf ",K[1][0],K[1][1],K[1][2]);
//  ROS_INFO("K2 : %lf  %lf %lf ",K[2][0],K[2][1],K[2][2]);
ROS_INFO("w : %lf  %lf %lf %lf",w[0],w[1],w[2],w[3]);
    // angular vel : rpm --> rps //

//ROS_INFO("%lf %lf",atan2(m[1],m[0]),yaw_rate);

    w[0] = (double) w[0];
    w[1] = (double) w[1];
    w[2] = (double) w[2];
    w[3] = (double) w[3];

    w[0] =  w[0] / gear_ratio * rpm_to_rps;
    w[1] =  w[1] / gear_ratio * rpm_to_rps;
    w[2] =  w[2] / gear_ratio * rpm_to_rps;
    w[3] =  w[3] / gear_ratio * rpm_to_rps;

//    B[0][0] = dt;
//    B[1][1] = dt;
//    B[2][2] = dt;

    u[0] = a[0];
    u[1] = a[1];
    u[2] = yaw_rate;


// x_est[0] : linear_vel.x , x_est[1] : linear_vel.y //
// x_est[2] : angular_vel.z //

        // "Predict" : 166 line ~ 204 line //

        // Motion Model //

        x_est_temp[0] = x_est_last[0] + u[0]*dt;
        x_est_temp[1] = x_est_last[1] + u[1]*dt;
        x_est_temp[2] = x_est_last[2] + u[2]*dt;




        // P matrix //

        for(int i = 0;i < 3;i++){
          for(int j = 0;j < 3;j++){
            P_temp[i][j] = P_last[i][j] + Q[i][j];
          }
        }

        // "Correction" : 207 line ~ 290 line //

        // Kalman gain //

        double temp3[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

        for(int i = 0;i < 3;i++){
          for(int j = 0;j < 3;j++){
            temp3[i][j] = P_temp[i][j] + R[i][j];
          }
        }

        double det=0;

        det = temp3[0][0] * (temp3[1][1]*temp3[2][2]-temp3[1][2]*temp3[2][1]);
        det =det-temp3[0][1] * (temp3[1][0]*temp3[2][2]-temp3[1][2]*temp3[2][0]);
        det =det+temp3[0][2] * (temp3[1][0]*temp3[2][1]-temp3[1][1]*temp3[2][0]);

        double temp4[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

        temp4[0][0] = 1/det * (temp3[1][1]*temp3[2][2]-temp3[1][2]*temp3[2][1]);
        temp4[0][1] = -1/det * (temp3[0][1]*temp3[2][2]-temp3[0][2]*temp3[2][1]);
        temp4[0][2] = 1/det * (temp3[0][1]*temp3[1][2]-temp3[0][2]*temp3[1][1]);

        temp4[1][0] = -1/det * (temp3[1][0]*temp3[2][2]-temp3[1][2]*temp3[2][0]);
        temp4[1][1] = 1/det * (temp3[0][0]*temp3[2][2]-temp3[0][2]*temp3[2][0]);
        temp4[1][2] = -1/det * (temp3[0][0]*temp3[1][2]-temp3[0][2]*temp3[1][0]);

        temp4[2][0] = 1/det * (temp3[1][0]*temp3[2][1]-temp3[1][1]*temp3[2][0]);
        temp4[2][1] = -1/det * (temp3[0][0]*temp3[2][1]-temp3[0][1]*temp3[2][0]);
        temp4[2][2] = 1/det * (temp3[0][0]*temp3[1][1]-temp3[1][0]*temp3[0][1]);

        for(int i = 0;i < 3;i++){
          for(int j = 0;j < 3;j++){
            K[i][j] = 0;
          }
        }

        for(int i = 0;i < 3;i++){
          for(int j = 0;j < 3;j++){
            for(int k = 0;k < 3;k++){
              K[i][j] += P_temp[i][k] * temp4[k][j];
            }
          }
        }

        // Estimation //

        // Measurement //

        // slip model :  v_phi,slip = yaw rate - v_phi,no_slip //

//	double v_phi_slip = yaw_rate - wheel_radius/4.0/l*(-w[0]+w[1]-w[2]+w[3]);

//	double v_x_slip = v_phi_slip * l + 2 *wheel_radius * alp / 4.0 * (w[0]+w[2]);
//	double v_y_slip = v_phi_slip * l + 2 *wheel_radius * alp / 4.0 * (w[2]-w[3]);

        z_meas[0] = wheel_radius / 4.0 * (w[0]+w[1]+w[2]+w[3]);
        z_meas[1] = wheel_radius / 4.0 * (-w[0]+w[1]+w[2]-w[3]);
        z_meas[2] = -atan2(m[1],m[0]);

        double temp5[3]={0,0,0};

        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
              temp5[i] += K[i][j]*(z_meas[j]-x_est_temp[j]);
          }
        }

        for(int i=0;i<3;i++){
          x_est[i] = x_est_temp[i] + temp5[i];
        }

        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            P[i][j]=0;
          }
        }

        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
              P[i][j] += (I[i][k]-K[i][k])*P_temp[k][j];
             }
           }
        }

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(x_est[2]);

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.rotation = odom_quat;

    //filling the odometry
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = x_est[0]*cos(x_est[2])-x_est[1]*sin(x_est[2]);
    odom.twist.twist.linear.y = x_est[0]*sin(x_est[2])+x_est[1]*cos(x_est[2]);
    odom.twist.twist.linear.z = 0;

//    odom.twist.twist.linear.x = vel[0];
//    odom.twist.twist.linear.y = vel[1];
//    odom.twist.twist.linear.z = 0;


    odom.twist.twist.angular.z = x_est[2];

    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);

    x = x + x_est[0] * dt * cos(x_est[2]) - x_est[1] * dt * sin(x_est[2]);
    y = y + x_est[0] * dt * sin(x_est[2]) + x_est[1] * dt * cos(x_est[2]);

    x_est_last[0] = x_est[0];
    x_est_last[1] = x_est[1];
    x_est_last[2] = x_est[2];


    for(int i=0;i<3;i++){
       for(int j=0;j<3;j++){
          P_last[i][j] = P[i][j];
       }
    }


    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
