#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd_publisher");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<vehicle_control::commendMsg>("/ns1/cmd_msg",100); //100 que size//

  ros::Rate loop_rate(15); // Setting 50 Hz //


  int N=800;


  double xd[N];
  double yd[N];
  double phid[N];


  int n=1;

  int i=1;
  int j=0;
  int index =1;


  while(ros::ok())
  {


    vehicle_control::commendMsg cmd_msg;

    // Trajectory : Circle


      xd[i] = 1 * cos( (double) 2.0*3.141592/N*i);
		///(1+sin( (double) 2.0*3.141592/N*i)*sin( (double) 2.0*3.141592/N*i));
      yd[i] = 1 * sin( (double) 2.0*3.141592/N*i); //*cos( (double) 2.0*3.141592/N*i)
		///(1+sin( (double) 2.0*3.141592/N*i)*sin( (double) 2.0*3.141592/N*i));
      phid[i] = (double) 2.0*3.141592/N*i+(n-1)*2*3.141592;

    if(index > 3){
    xd[i] = 0;
    yd[i] = 0;
    phid[i] = (double) (n-1)*2*3.141592;
    }


    cmd_msg.xd = xd[j];
    cmd_msg.yd = yd[j];
    cmd_msg.phid = phid[j];


    cmd_pub.publish(cmd_msg);


    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);
    ROS_INFO("%d",index);
    loop_rate.sleep();


    i++;
    j++;
    if(j==N)
    {
        i=0;
        j=0;
        n++;
        index++;
    }

  }

  return 0;

}

