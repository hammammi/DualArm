#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/motorsMsg.h"
#include <geometry_msgs/Twist.h>

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
  const double Kp[4] = {1,1,1,1};
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


  while(ros::ok()){
      geometry_msgs::Twist cmd_vel;
      curr_time = ros::Time::now().toSec();
      dt = curr_time - last_time;



      if(index==1||dt==0||dt>1){
          dt=0.01;
          index=2;
      }
      last_time = curr_time;

      motor_input[0] = (double) motor_input[0];
      motor_input[1] = (double) motor_input[1];
      motor_input[2] = (double) motor_input[2];
      motor_input[3] = (double) motor_input[3];

      motor_act[0] = motor_act[0]*gear_ratio*rps_to_rpm;
      motor_act[1] = motor_act[1]*gear_ratio*rps_to_rpm;
      motor_act[2] = motor_act[2]*gear_ratio*rps_to_rpm;
      motor_act[3] = motor_act[3]*gear_ratio*rps_to_rpm;




      if(motor_input[0]!=motor_act[0]){
          alp[0] = motor_input[0]-motor_act[0] > 0 ? alp_positive:alp_negative;
          motor_act[0] = motor_act[0] + alp[0]*dt;
      }


      if(motor_input[1]!=motor_act[1]){
          alp[1] = motor_input[1]-motor_act[1] > 0 ? alp_positive:alp_negative;
          motor_act[1] = motor_act[1] + alp[1]*dt;
      }


      if(motor_input[2]!=motor_act[2]){
          alp[2] = motor_input[2]-motor_act[2] > 0 ? alp_positive:alp_negative;
          motor_act[2] = motor_act[2] + alp[2]*dt;
      }


      if(motor_input[3]!=motor_act[3]){
          alp[3] = motor_input[3]-motor_act[3] > 0 ? alp_positive:alp_negative;
          motor_act[3] = motor_act[3] + alp[3]*dt;
      }

	ROS_INFO("%lf %lf %lf %lf",motor_act[0],motor_act[1],motor_act[2],motor_act[3]);


      motor_act[0] = motor_act[0]/gear_ratio*rpm_to_rps;
      motor_act[1] = motor_act[1]/gear_ratio*rpm_to_rps;
      motor_act[2] = motor_act[2]/gear_ratio*rpm_to_rps;
      motor_act[3] = motor_act[3]/gear_ratio*rpm_to_rps;
      // Forward Kinematics //
      vx = wheel_radius/4.0 * (motor_act[0]+motor_act[1]+motor_act[2]+motor_act[3]);
      vy = wheel_radius/4.0 * (-motor_act[0]+motor_act[1]+motor_act[2]-motor_act[3]);
      vp = wheel_radius/4.0/l * (-motor_act[0]+motor_act[1]-motor_act[2]+motor_act[3]);

      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vp;
      vel_pub.publish(cmd_vel);
      


      ros::spinOnce();
      loop_rate.sleep();


  }
  return 0;
}
