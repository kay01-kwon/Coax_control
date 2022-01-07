#ifndef COAX_CONTROL_H
#define COAX_CONTROL_H

#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <lm4075e_msgs/Int32.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;

class CoaxCTRL{
    public:

    private:
    Vector3d I_p_CM;
    Vector3d I_v_CM;
    Vector3d I_a_CM;
    
    Vector3d I_p_des;
    Vector3d I_v_des;
    Vector3d I_a_des;

    double Kp_pos;
    double Kd_pos;

    double Kp_ori;
    double Kd_ori;

};

#endif