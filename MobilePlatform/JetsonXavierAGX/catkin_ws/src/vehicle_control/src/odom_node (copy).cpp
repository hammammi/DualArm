#include <string>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "vehicle_control/motorsMsg.h"

#define Pi 3.14159265358979323846

/* Ecoder data (unit : rpm) */
double w[4];

/* Imu data */
double a[3];  // a[xyz][prev curr]
double gyro_yaw;
double q[4];

/* Magnetometer (unit : Gause) */
double m[3];




void encoderCallback(const vehicle_control::motorsMsg::ConstPtr& motors){

    w[0] = motors -> omega1;
    w[1] = motors -> omega2;
    w[2] = motors -> omega3;
    w[3] = motors -> omega4;

    w[1] = -w[1];
    w[3] = -w[3];
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){

    a[0]      = imu -> linear_acceleration.x;
    a[1]      = imu -> linear_acceleration.y;

    q[0]     = imu -> orientation.x;
    q[1]     = imu -> orientation.y;
    q[2]     = imu -> orientation.z;
    q[3]     = imu -> orientation.w;
    gyro_yaw = imu -> angular_velocity.z;
}

void mgnCallback(const sensor_msgs::MagneticField::ConstPtr& mgn){
   m[0] = mgn -> magnetic_field.x;
   m[1] = mgn -> magnetic_field.y;
   m[2] = mgn -> magnetic_field.z;
}

int main(int argc,char** argv){

  ros::init(argc,argv,"state_publisher");
  ros::NodeHandle n;

  /* Encoder Subscriber*/
  ros::Subscriber enc_sub = n.subscribe("/input_msg",100,encoderCallback);
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
  double dx=0;
  double dy=0;
  double dphi=0;


  /* Time data */
  double dt=0.01;
  double curr_time;
  double last_time;



  /* Specification */
  double wheel_diameter = 0.1520;
  double wheel_radius = wheel_diameter / 2.0;
  double gear_ratio = 76.0;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;

  /* Conversion ratio */
  double rpm_to_rps = 2.0 * Pi / 60;

  /* Kalman Filter State variable */

  // phi dphidt
  double phi_est_last[2]={0,0};
  double phi_est_temp[2]={0,0};
  double phi_est[2]={0,0};

  // vx vy
  double v_est_last[2]={0,0};
  double v_est_temp[2]={0,0};
  double v_est[2]={0,0};

  // For rotational motion //
  double k;


  double Ar[2][2] = {{1,0.01},{0,1}};
  double Hr[2][2] = {{1,0},{0,1}};
  double Pr_last[2][2]={{1,0},{0,1}};
  double Pr_temp[2][2]={{0,0},{0,0}};
  double Pr[2][2]={{0,0},{0,0}};

  double Qr[2][2]={{0.1,0},{0,0.1}};
  double Rr[2][2]={{100,0},{0,100}};

  double Kr[2][2]={{0,0},{0,0}};




  // For Translational motion //
  double Av[2][2] = {{1,0},{0,1}};
  double Hv[2][2] = {{1,0},{0,1}};
  double Pv_last[2][2]={{1,0},{0,1}};
  double Pv_temp[2][2]={{0,0},{0,0}};
  double Pv[2][2]={{0,0},{0,0}};

  double Qv[2][2]={{0.1,0},{0,0.1}};
  double Rv[2][2]={{100,0},{0,100}};

  double Kv[2][2]={{0,0},{0,0}};
  double alp =0.009;

  double v[2]={0,0};

  while(ros::ok()){

	dt = 0.01;
    // Time //

//    curr_time = ros::Time::now().toSec();
//    dt = curr_time - last_time;


//    last_time = curr_time;

    // angular vel : rpm --> rps // 

    w[0] = (double) w[0];
    w[1] = (double) w[1];
    w[2] = (double) w[2];
    w[3] = (double) w[3];

    w[0] =  w[0] / gear_ratio * rpm_to_rps;
    w[1] =  w[1] / gear_ratio * rpm_to_rps;
    w[2] =  w[2] / gear_ratio * rpm_to_rps;
    w[3] =  w[3] / gear_ratio * rpm_to_rps;



    double v_phi = -wheel_radius / (4.0 * l) * (-w[0] + w[1] - w[2] + w[3] );

    Ar[0][1] = dt;    
//    k = 4 * l * (phi_est[0]-phi_est[1]/dt-v_phi);

    
//   dx = v[0] * dt;
//   dy = v[1] * dt;
//   dphi = v_phi * dt;
//   phi = dphi + phi

    double I[2][2] = {{1,0},{0,1}};


    // Rotational KF //
    double phi_measure[2];

    phi_measure[0] = -atan2(m[1],m[0]);
    phi_measure[1] = (-atan2(m[1],m[0]) - phi_est_last[0])/dt;

    // motion model //
      phi_est_temp[0] = phi_est_last[0] + dt * gyro_yaw;
      phi_est_temp[1] = gyro_yaw;



      double temp1[2][2] = {{0,0},{0,0}};

      for(int i=0;i<2;i++){
         for(int j=0;j<2;j++){
            for(int k=0;k<2;k++){
               temp1[i][j] += Ar[i][k]*Pr_last[k][j];
            }
         }
      }



      double temp2[2][2] = {{0,0},{0,0}};

      for(int i=0;i<2;i++){
         for(int j=0;j<2;j++){
            for(int k=0;k<2;k++){
               temp2[i][j] += temp1[i][k]*Ar[j][k];
            }
         }
      }

      for(int i=0;i<2;i++){
         for(int j=0;j<2;j++){
               Pr_temp[i][j] = temp2[i][j]+Qr[i][j];
         }
      }


    // Kalman gain //

    double temp3[2][2]={{0,0},{0,0}};
    double temp4[2][2]={{0,0},{0,0}};

    for(int i=0;i<2;i++){
       for(int j=0;j<2;j++){
          temp3[i][j] = Pr_temp[i][j]+Rr[i][j];
       }
    }
    



    double det = temp3[0][0]*temp3[1][1]-temp3[0][1]*temp3[1][0];

    temp4[0][0] = 1/det * temp3[1][1];
    temp4[1][1] = 1/det * temp3[0][0];
    temp4[0][1] = -1/det * temp3[0][1];
    temp4[1][0] = -1/det * temp3[1][0];

 
    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
            Kr[i][j] = 0;
      }
    }  


    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
        for(int k=0;k<2;k++){
            Kr[i][j] += Pr_temp[i][k]*temp4[k][j];
        }
      }
    }  

// estimated angular velocity //

    double temp5[2]={0,0};


    for(int i=0;i<2;i++){
      for(int k=0;k<2;k++){
             temp5[i]+= Kr[i][k]*(phi_measure[k]-phi_est_temp[k]);
      }
    } 

    for(int i=0;i<2;i++){
      phi_est[i] = phi_est_temp[i] + temp5[i];
    }




    // Initialization of P matrix //
    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
          Pr[i][j] = 0;
      }
    }

    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
        for(int k=0;k<2;k++){
          Pr[i][j] += (I[i][k] -Kr[i][k]) * (Pr_temp[k][j]);
        }
      }
    }


// Translational KF //




// Motion model //
    for(int i=0;i<2;i++){
      v_est_temp[i] = v_est_last[i] + dt/2.0 * (a[i]+(v_est[i]-v_est_last[i])/dt);
    }


    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
            Pv_temp[i][j] = Pv_last[i][j] + Qv[i][j];
        }
    }
  
   double temp6[2][2]={{0,0},{0,0}};
   double temp7[2][2]={{0,0},{0,0}};

    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
            temp6[i][j] = Pv_temp[i][j] + Rv[i][j];
        }
    }
 

// Kalman gain //
    double det2 = temp6[0][0]*temp6[1][1]-temp6[0][1]*temp6[1][0];

    temp7[0][0] = 1/det2 * temp6[1][1];
    temp7[1][1] = 1/det2 * temp6[0][0];
    temp7[0][1] = -1/det2 * temp6[0][1];
    temp7[1][0] = -1/det2 * temp6[1][0];

 
    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
            Kv[i][j] = 0;
      }
    }  



    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
        for(int k=0;k<2;k++){
            Kv[i][j] += Pv_temp[i][k]*temp7[k][j];
        }
      }
    }  

// estimated linear velocity //


    double temp8[2]={0,0};
    temp8[0]=0;
    temp8[1]=0;

// Measurement //
   v[0] = -wheel_radius / 4.0 * ( w[0] + w[1] + w[2] + w[3] );
   v[1] = -wheel_radius / 4.0 * (-w[0] + w[1] + w[2] - w[3] );


    for(int i=0;i<2;i++){
      for(int k=0;k<2;k++){
             temp8[i]+= Kv[i][k]*(v[k]-v_est_temp[k]);
      }
    } 

    for(int i=0;i<2;i++){
      v_est[i] = v_est_temp[i] + temp8[i];
    }






	// Initialization of Pv matrix //
    for(int i = 0;i < 2;i++){
      for(int j = 0;j <2;j++){
         Pv[i][j]=0;
      }
    }


    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
        for(int k=0;k<2;k++){
          Pv[i][j] += (I[i][k] -Kv[i][k]) * (Pv_temp[k][j]);
        }
      }
    }


    ROS_INFO("vx : %lf vy : %lf vphi : %lf",v[0],v[1],v_phi);

     x += v_est[0] * dt * cos(phi_est[0]) - v_est[1] * dt * sin(phi_est[0]);
     y += v_est[0] * dt * sin(phi_est[0]) + v_est[1] * dt * cos(phi_est[0]);
    

//   x = x + dx * cosf(phi) - dy * sinf(phi);
//   y = y + dx * sinf(phi) + dy * cosf(phi);

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,phi);
/*
    odom_quat.w = cy * cr * cp + sy * sr * sp;
    odom_quat.x = cy * sr * cp - sy * cr * sp;
    odom_quat.y = cy * cr * sp + sy * sr * cp;
    odom_quat.z = sy * cr * cp - cy * sr * sp;
*/

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.rotation.x = q[0];
    odom_trans.transform.rotation.y = q[1];
    odom_trans.transform.rotation.z = q[2];
    odom_trans.transform.rotation.w = q[3];


    //filling the odometry
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
//    odom.pose.pose.orientation = odom_quat;
    odom.pose.pose.orientation.x = q[0];
    odom.pose.pose.orientation.y = q[1];
    odom.pose.pose.orientation.z = q[2];
    odom.pose.pose.orientation.w = q[3];


    odom.twist.twist.linear.x = dx/dt;
    odom.twist.twist.linear.y = dy/dt;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.z = dphi/dt;

    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);


    v_est_last[0] = v_est[0];
    v_est_last[1] = v_est[1];


    phi_est_last[0] = phi_est[0];      


    for(int i=0;i<2;i++){
       for(int j=0;j<2;j++){
          Pr_last[i][j] = Pr[i][j];
       }
    }

    for(int i=0;i<2;i++){
       for(int j=0;j<2;j++){
          Pv_last[i][j] = Pv[i][j];
       }
    }


    if(w[0] == 0 && w[1] == 0 && w[2] == 0 && w[3] == 0 && a[0] == 0 && a[1] == 0){
      v_est[0]=0;
      v_est[1]=0;
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
