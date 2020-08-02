#include <path_planner/velocity_plan.hpp>

int main(int argc, char** argv){

    ros::init(argc,argv,"vel_generator");

    velocity_plan vel_plan;

    double x_goal;
    double y_goal;

    bool is_init = false;

    vel_plan.publisher_setting();
    vel_plan.subscriber_setting();        

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        //ROS_INFO("goal reached? : %d",vel_plan.goal_reached());

        if(vel_plan.goal_reached() == 0){
            ROS_INFO("init x : %lf, init y : %lf",vel_plan.x_init,vel_plan.y_init);
            vel_plan.time_calculation();
            vel_plan.traj_generation();
         
        }

        if(vel_plan.goal_reached() == 1){
            vel_plan.t = 0;
            vel_plan.t_last = 0;
        }
      
        
        ros::spinOnce();
        loop_rate.sleep();

    }
    

}