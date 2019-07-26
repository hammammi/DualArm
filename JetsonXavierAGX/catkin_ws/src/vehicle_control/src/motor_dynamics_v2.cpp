#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/motorsMsg.h"
#include <geometry_msgs/Twist.h>
#include "vehicle_control/wMsg.h"

#define Pi 3.14159265358979323849


double motor_input[4];

void motorsCallback(const vehicle_control::motorsMsg::ConstPtr& motors){
  motor_input[0] = motors->omega1;
  motor_input[1] = motors->omega2;
  motor_input[2] = motors->omega3;
  motor_input[3] = motors->omega4;

  motor_input[0]=-motor_input[0];
  motor_input[2]=-motor_input[2];
}


double AGN(double mean, double stddev)
{
    double result;
    std::mt19937 generator(std::random_device{} ());
    std::normal_distribution<double> dist(mean,stddev);
    result = dist(generator);
    return result;
}


int main(int argc,char **argv){

  ros::init(argc,argv,"motor_dynamics");
  ros::NodeHandle nh;

  ros::Subscriber motor_sub = nh.subscribe("/input_msg",100,motorsCallback);

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  ros::Publisher w_pub = nh.advertise<vehicle_control::wMsg>("/w_msg",100);
  ros::Publisher wn_pub = nh.advertise<vehicle_control::wMsg>("/w_noise",100);
  ros::Rate loop_rate(100);

  // Mecanum platform specification //
  double wheel_radius = 0.076;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;

  // Motor specification //
  double gear_ratio = 76;
  double rpm_to_rps = 2*Pi/60;
  double rps_to_rpm = 60/2.0/Pi;
  double motor_act[4]={0,0,0,0};
  double alp[4]={0,0,0,0};
  const double alp_positive = 3000;
  const double alp_negative = -3000;
  const double alp_max = 3000;
  const double Kp[4] = {1,1,1,1};
  const double Kd[4] = {0.1,0.1,0.1,0.1};
  double dt=0.01;
  double curr_time=0;
  double last_time=0;
  int index=1;

  double vx;
  double vy;
  double vp;



  double del1=0;
  double del2=0;
  double del3=0;
  double del4=0;
  double p[4];
  double mean=0;
  double stddev=0.04;


  while(ros::ok()){
      geometry_msgs::Twist cmd_vel;
      vehicle_control::wMsg w_output;
      vehicle_control::wMsg wn;
      curr_time = ros::Time::now().toSec();
      dt = curr_time - last_time;

      // Time //
      if(index==1||dt==0||dt>1){
          dt=0.01;
          index=2;
      }
      last_time = curr_time;


      // input msg : unit conversion //
      motor_input[0] = (double) motor_input[0];
      motor_input[1] = (double) motor_input[1];
      motor_input[2] = (double) motor_input[2];
      motor_input[3] = (double) motor_input[3];

      // RPS to RPM //
      motor_act[0] = motor_act[0]*gear_ratio*rps_to_rpm;
      motor_act[1] = motor_act[1]*gear_ratio*rps_to_rpm;
      motor_act[2] = motor_act[2]*gear_ratio*rps_to_rpm;
      motor_act[3] = motor_act[3]*gear_ratio*rps_to_rpm;

      // angular acceleration : P control //
      alp[0] = Kp[0]*(motor_input[0]-motor_act[0]);
      alp[1] = Kp[1]*(motor_input[1]-motor_act[1]);
      alp[2] = Kp[2]*(motor_input[2]-motor_act[2]);
      alp[3] = Kp[3]*(motor_input[3]-motor_act[3]);

      // Error : Maximum value --> maximum acceleration //
      // line 114 ~ 150 //
    if(fabs(motor_input[0]-motor_act[0])/dt>alp_max){
        alp[0] = motor_input[0]>motor_act[0] ? alp_positive:alp_negative;
    }

    if(fabs(motor_input[1]-motor_act[1])/dt>alp_max){
        alp[1] = motor_input[1]>motor_act[1] ? alp_positive:alp_negative;
    }


    if(fabs(motor_input[2]-motor_act[2])/dt>alp_max){
        alp[2] = motor_input[2]>motor_act[2] ? alp_positive:alp_negative;
    }


    if(fabs(motor_input[3]-motor_act[3])/dt>alp_max){
        alp[3] = motor_input[3]>motor_act[3] ? alp_positive:alp_negative;
    }

    // Calculate Motor velocities //
    motor_act[0] = motor_act[0] + alp[0] * dt;
    motor_act[1] = motor_act[1] + alp[1] * dt;
    motor_act[2] = motor_act[2] + alp[2] * dt;
    motor_act[3] = motor_act[3] + alp[3] * dt;

    // RPM to RPS conversion to offer velocity to simulator //
      motor_act[0] = motor_act[0]/gear_ratio*rpm_to_rps;
      motor_act[1] = motor_act[1]/gear_ratio*rpm_to_rps;
      motor_act[2] = motor_act[2]/gear_ratio*rpm_to_rps;
      motor_act[3] = motor_act[3]/gear_ratio*rpm_to_rps;

      // w_output //
      w_output.w_output1 = motor_act[0];
      w_output.w_output2 = motor_act[1];
      w_output.w_output3 = motor_act[2];
      w_output.w_output4 = motor_act[3];


      // Gaussian noise //
      del1 = AGN(mean,stddev);
      del2 = AGN(mean,stddev);
      del3 = AGN(mean,stddev);
      del4 = AGN(mean,stddev);

      del1 = fabs(del1);
      del2 = fabs(del2);
      del3 = fabs(del3);
      del4 = fabs(del4);

      del1 = del1>=1?1:del1;
      del2 = del2>=1?1:del2;
      del3 = del3>=1?1:del3;
      del4 = del4>=1?1:del4;


      // No slip
      del1=0;
      del2=0;
      del3=0;
      del4=0;


      // w with noise
      wn.w_output1 = motor_act[0]*(1-del1);
      wn.w_output2 = motor_act[1]*(1-del2);
      wn.w_output3 = motor_act[2]*(1-del3);
      wn.w_output4 = motor_act[3]*(1-del4);



      // Forward Kinematics //


      vx = wheel_radius/4.0 * (motor_act[0]*(1-del1)+motor_act[1]*(1-del2)+motor_act[2]*(1-del3)+motor_act[3]*(1-del4));
      vy = wheel_radius/4.0 * (-motor_act[0]*(1-del1)+motor_act[1]*(1-del2)+motor_act[2]*(1-del3)-motor_act[3]*(1-del4));
      vp = wheel_radius/4.0/l * (-motor_act[0]*(1-del1)+motor_act[1]*(1-del2)-motor_act[2]*(1-del3)+motor_act[3]*(1-del4));

     // vx = wheel_radius/4.0 * (motor_act[0]+motor_act[1]+motor_act[2]+motor_act[3]);
     // vy = wheel_radius/4.0 * (-motor_act[0]+motor_act[1]+motor_act[2]*(1-del3)-motor_act[3]);
     // vp = wheel_radius/4.0/l * (-motor_act[0]+motor_act[1]-motor_act[2]+motor_act[3]);

      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vp;
      vel_pub.publish(cmd_vel);
      w_pub.publish(w_output);
      wn_pub.publish(wn);


      ros::spinOnce();
      loop_rate.sleep();


  }
  return 0;
}
