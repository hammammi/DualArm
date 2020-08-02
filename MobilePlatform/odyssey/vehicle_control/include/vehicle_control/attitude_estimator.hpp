#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <vehicle_control/as5047Msg.h>
#include <nav_msgs/Odometry.h>


#define _USE_MATH_DEFINES

using vehicle_control::as5047Msg;
using nav_msgs::Odometry;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using Eigen::Vector3d;

using std::cout;
using std::endl;

class attitude_estimator{

    public:
    
    void InitiateVariables();
    void EncoderSubscriberSetting();
    void PublisherSetting();
    void EncoderCallbackFunc(const as5047Msg & mag_enc);
    void WheelPosCalculation();
    void NormalVectorCalculation();
    void RollPitchCalculation();
    void PlatformCenterCalculation();
    void TransformPublisher();

    MatrixXd Rotation_x(double &phi);
    MatrixXd Rotation_y(double &theta);
    void BaseLinkPublish();

    private:

    const int num_joint = 4;

    // Link info
    const double l_1 = 0.010;
    const double l_2 = 0.191;

    // Wheel info
    const double l_sep_x = 0.140;
    const double l_sep_y = 0.310825;
    const double wheel_radious = 0.076;

    // data conversion info
    const double bit_to_deg = (double) 360.0/16363.0;
    const double deg_to_rad = (double) M_PI/180.0;

    // Joints position
    MatrixXd joint_pos = MatrixXd(3,num_joint);

    // Joint angle info
    MatrixXd direction_sign = MatrixXd(num_joint,num_joint);
    VectorXd joint_angle_init = VectorXd(num_joint);
    VectorXd joint_angle_curr = VectorXd(num_joint);
    VectorXd joint_angle_rotation = VectorXd(num_joint);

    // xyz : Link frame, XYZ : reference frame
    MatrixXd link_wheel_center_pos = MatrixXd(3,num_joint);
    MatrixXd p_wheel_center_pos = MatrixXd(3,num_joint);

    // p_z_b : z unit vector of base_footprint w.r.t platform center
    // b_z_p : z unit vector of platform center w.r.t base footprint
    
    Vector3d p_z_b;
    Vector3d b_z_p;
    Vector3d p_z_p;

    VectorXd roll_pitch = VectorXd(2);

    MatrixXd p_R_b = MatrixXd(3,3);
    MatrixXd b_R_p = MatrixXd(3,3);

    // p_p_b : base footprint location w.r.t platform center frame
    // b_p_p : platform center location w.r.t base footprint frame

    Vector3d p_p_b;
    Vector3d b_p_p;

    // p_p_target : target location w.r.t platform center frame
    Vector3d p_p_base_arm;
    Vector3d p_p_camera_sensor;
    Vector3d p_p_lidar_sensor;

    // b_p_target : target location w.r.t base footprint frame
    VectorXd b_p_base_arm;
    VectorXd b_p_camera_sensor;
    VectorXd b_p_lidar_sensor;

    ros::NodeHandle nh;
    ros::Publisher BaseLinkPosePublisher;
    ros::Subscriber Encoder_subscriber;


};

void attitude_estimator::InitiateVariables()
{
    joint_pos << l_sep_x/2, l_sep_x/2, -l_sep_x/2, -l_sep_x/2,
                l_sep_y/2, -l_sep_y/2, -l_sep_y/2, l_sep_y/2,
                0, 0, 0, 0;

    direction_sign << 1,0,0,0,
                    0,-1,0,0,
                    0,0,-1,0,
                    0,0,0,1;

    joint_angle_init <<306.2234, 80.5273,
                    258.8940, 107.3574;
    
    link_wheel_center_pos << -l_1, -l_1, l_1, l_1,
                            0, 0, 0, 0,
                            -l_2, -l_2, -l_2, -l_2;

    p_wheel_center_pos.fill(0.0);

    p_z_p << 0,0,1;

    p_p_base_arm<<0,0,0.520;

}

void attitude_estimator::EncoderSubscriberSetting()
{
    Encoder_subscriber = nh.subscribe("/magEnc",1,&attitude_estimator::EncoderCallbackFunc,this);
}

void attitude_estimator::PublisherSetting()
{
    BaseLinkPosePublisher = nh.advertise<Odometry>("/base_arm_pose",1);
}

void attitude_estimator::EncoderCallbackFunc(const as5047Msg &mag_enc)
{    
    // Encoder Data Acquisition
    for(int i = 0; i < num_joint;++i)
    {
        joint_angle_curr(i) = (double) mag_enc.mag_enc[i] * bit_to_deg;
    }

    joint_angle_rotation = deg_to_rad*direction_sign*(joint_angle_curr - joint_angle_init);
}

void attitude_estimator::WheelPosCalculation()
{
    for(int i = 0; i < num_joint; ++i)
    {
        p_wheel_center_pos.col(i) = Rotation_y(joint_angle_rotation(i))*link_wheel_center_pos.col(i);
    }

    p_wheel_center_pos += joint_pos;
}

void attitude_estimator::NormalVectorCalculation()
{
    MatrixXd w = MatrixXd(3,num_joint);

    Vector3d u_temp;
    Vector3d v_temp;
    Vector3d w_temp;

    u_temp.fill(0.0);
    v_temp.fill(0.0);
    w_temp.fill(0.0);

    p_z_b.fill(0.0);

    for(int i = 0; i < num_joint; ++i)
    {
        int j,k;
        j = (i+2)%num_joint;
        k = (i+1)%num_joint;

        u_temp = p_wheel_center_pos.col(j) - p_wheel_center_pos.col(i);
        v_temp = p_wheel_center_pos.col(k) - p_wheel_center_pos.col(i);

        w_temp = u_temp.cross(v_temp);
        w.col(i) = w_temp/sqrt(w_temp.dot(w_temp));

        p_z_b += w.col(i);

        u_temp.fill(0.0);
        v_temp.fill(0.0);
        w_temp.fill(0.0);
    }

    // Average unit vector
    p_z_b /= num_joint;
    p_z_p.normalize();

}

void attitude_estimator::RollPitchCalculation()
{
    // roll_pitch(0) : roll
    // roll_pitch(1) : pitch
    roll_pitch(1) = asin(p_z_b(0));

    double Y = -p_z_b(1)/cos(roll_pitch(1));
    double Z = p_z_b(2)/cos(roll_pitch(1));

    roll_pitch(0) = atan2(Y,Z);
}

void attitude_estimator::PlatformCenterCalculation()
{
    // Grounded wheel position w.r.t platform center
    double s = 0;
    MatrixXd p_wheel_ground = MatrixXd(3,num_joint);
    Vector3d temp_vec;
    MatrixXd shift = MatrixXd(3,num_joint);

    shift<<0, 0, 0, 0,
            0, 0, 0, 0,
            -wheel_radious, -wheel_radious, -wheel_radious, -wheel_radious;

    p_R_b = Rotation_x(roll_pitch(0))*Rotation_y(roll_pitch(1));
    b_R_p = p_R_b.transpose();

    p_wheel_ground = p_wheel_center_pos + p_R_b * shift;

    // Basefootprint Calculation w.r.t platform center     
    for(int i = 0; i < num_joint; ++i)
    {   
        temp_vec.fill(0.0);
        temp_vec = p_wheel_ground.col(i);
        s += p_z_b.dot(temp_vec)/p_z_b.dot(p_z_p);
    }
    s /= 4;
    p_p_b = s * p_z_p;

    // Platform center w.r.t Basefootprint
    b_p_p = b_R_p * (-p_p_b); 
}

void attitude_estimator::TransformPublisher()
{
    WheelPosCalculation();
    NormalVectorCalculation();
    RollPitchCalculation();
    PlatformCenterCalculation();

    b_p_base_arm = b_p_p + b_R_p * p_p_base_arm;
    cout<<b_p_base_arm<<endl;
    cout<<endl;

}

MatrixXd attitude_estimator::Rotation_x(double &phi)
{
    MatrixXd R_phi = MatrixXd(3,3);

    R_phi << 1, 0, 0,
            0, cos(phi), -sin(phi),
            0, sin(phi), cos(phi);
    return R_phi;
}

MatrixXd attitude_estimator::Rotation_y(double &theta)
{
    MatrixXd R_theta = MatrixXd(3,3);

    R_theta << cos(theta), 0 ,sin(theta),
            0, 1, 0,
            -sin(theta), 0, cos(theta);

    return R_theta;
}