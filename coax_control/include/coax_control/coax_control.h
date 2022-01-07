#ifndef COAX_CONTROL_H
#define COAX_CONTROL_H

#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <lm4075e_msgs/Int32.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using nav_msgs::Odometry;

using Eigen::Vector3d;
using Eigen::Matrix3d;

class CoaxCTRL{
    public:
    // Constructor
    CoaxCTRL();

    // Callback trajectory


    // Callback Pose
    void CallbackPose(const Odometry & pose_msg);

    // Position Controller
    void PosControl();

    // Orientation Controller
    void OriControl();

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
    
    Vector3d I_p_des;
    Vector3d I_v_des;
    Vector3d I_a_des;

    Vector3d CM_p_CM2T;

    double Kp_pos;
    double Kd_pos;

    double Kp_ori;
    double Kd_ori;

    double throttle;
    double des_rp;
    double des_yaw;
};

#endif