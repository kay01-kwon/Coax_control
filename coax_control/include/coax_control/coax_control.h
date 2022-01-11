#ifndef COAX_CONTROL_H
#define COAX_CONTROL_H

#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <lm4075e_msgs/Int32.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Accel;
using nav_msgs::Odometry;
using std_msgs::Int32;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Eigen::Matrix3d;

using std::cout;
using std::endl;

class CoaxCTRL{
    public:
    // Constructor
    CoaxCTRL();

    // Callback trajectory
    void CallbackDesPos(const Pose & des_p);
    void CallbackDesVel(const Twist & des_v);
    void CallbackDesAcc(const Accel & des_a);

    // Callback Estimated Pose
    void CallbackPose(const Odometry & pose_msg);

    // Position Controller
    void PosControl();

    // Orientation Controller
    void OriControl();

    void throttle_clamping(double &throttle_ptr);
    
    // Destructor
    ~CoaxCTRL();

    private:
    ros::NodeHandle nh;
    ros::Subscriber traj_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Publisher throttle_publisher;
    ros::Publisher tvc_publisher;
    ros::Publisher rudder_publisher;

    Vector3d I_p_CM;
    Vector3d I_v_CM;
    Vector3d I_a_CM;
    Vector3d I_W_CM;
    Vector4d I_q_CM;    //qw, qx, qy, qz

    Vector3d CM_p_CM_T;
    Vector3d CM_u_CM_T;
    Vector2d eq_rp;

    Vector3d I_p_des;
    Vector3d I_v_des;
    Vector3d I_a_des;

    Vector3d u_pos;

    double mass;
    const double g = -9.81;
    double CM_x_T;
    double CM_y_T;
    double CM_z_T;

    double Kp_pos;
    double Kd_pos;

    double Kp_ori;
    double Kd_ori;

    double C_lift;

    double throttle;
    double des_rp;
    double des_yaw;

    const double throttle_max = 10000;
    const double throttle_min = 8000;
};

#endif