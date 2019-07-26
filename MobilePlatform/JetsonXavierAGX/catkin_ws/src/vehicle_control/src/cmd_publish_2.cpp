#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd_publisher2");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<vehicle_control::commendMsg>("/ns1/cmd_msg",100); //100 que size//

  int hz1 = 30; // 0.60m/s
//  int hz2 = 20; // 0.30m/s

  ros::Rate loop_rate(hz1); // Setting 50 Hz //


  int N=200; //


  double xd=2;
  double yd=2;
  double phid=0;

  double x1 = 3;
  double y1 = 3;

  int n=1;

  int rotation_index = 1;

  int i=0;
  int j=0;
  int k=0;

  int index=1;


  while(ros::ok())
  {


    vehicle_control::commendMsg cmd_msg;

    // Trajectory : Rectangular

//       y
//       ^
//       |
//       |
//       |
//       4---------------3
//       |               |
//       |               |
//       |               |
//       |               |
//       1---------------2   ---> x
//
//

//     From point 1 to point 2
    if(index ==1){
    xd = (double) x1*i/N;
    yd = 0;

//    Orientation
    phid = (double) 3.141592*i/N/2.0+(double)3.141592*(rotation_index-1);

    i++;
        if(i>N){
            i=0;
            index = 2;
            rotation_index++;
        }

    }

//     From point 2 to point 3
    if(index ==2){
    xd = x1;
    yd = (double) y1*i/N;
    i++;

//    Orientation
    phid = (double) 3.141592*i/N/2.0+(double)3.141592*(rotation_index-1);

        if(i>N){
            i=0;
            index = 3;
            rotation_index++;
            }

    }

//     From point 3 to point 4
    if(index ==3){
    xd = x1*(N-i)/N;
    yd = y1;
    i++;

//    Orientation
    phid = (double) 3.141592*i/N/2.0+(double)3.141592*(rotation_index-1);

        if(i>N){
            i=0;
            index = 4;
            rotation_index++;
        }
    }


//     From point 4 to point 1
    if(index ==4){
    xd = 0;
    yd = y1*(N-i)/N;
    i++;

//    Orientation
    phid = (double) 3.141592*i/N/2.0+(double)3.141592*(rotation_index-1);

        if(i>N){
            i=0;
            index = 1;
            rotation_index++;
            n++;
        }
    }

//      The End
    if(n==6){
    xd=0;
    yd=0;
    index = 5;
    }


    cmd_msg.xd = xd;
    cmd_msg.yd = yd;
    cmd_msg.phid = phid;


    cmd_pub.publish(cmd_msg);


    ROS_INFO("%d" ,n);
    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);

    loop_rate.sleep();

  }

  return 0;

}

