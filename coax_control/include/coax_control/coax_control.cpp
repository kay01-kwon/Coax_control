#include "coax_control.h"

CoaxCTRL::CoaxCTRL()
{
    // Constructor
    cout<<"*****Mass Parameter Setup*****"<<endl;
    nh.getParam("mass",mass);
    cout<<"Mass (kg): ";
    cout<<mass<<endl;
    I_W_CM << 0, 0, mass*g;

    cout<<"*****COM Offset Parameter Setup*****"<<endl;
    nh.getParam("CM_x_T",CM_x_T);
    nh.getParam("CM_y_T",CM_y_T);
    nh.getParam("CM_z_T",CM_z_T);

    double phi,theta;

    CM_p_CM_T << CM_x_T, CM_y_T, CM_z_T;
    CM_u_CM_T = CM_p_CM_T.normalized();
    cout<<CM_p_CM_T<<endl;

    phi = -asin(CM_u_CM_T(1));
    theta = atan2(CM_u_CM_T(0)/cos(phi),CM_u_CM_T(2)/cos(phi));

    eq_rp << phi, theta; // Roll, pitch (y-x convention)
    hovering_rp = -eq_rp;
    cout<<"*****Equlibrium TVC Info (deg)*****"<<endl;
    cout<<"Roll : ";
    cout<<eq_rp(0)*180.0/M_PI<<endl;
    cout<<"Pitch : ";
    cout<<eq_rp(1)*180.0/M_PI<<endl;

    cout<<"*****Hovering Attitude Info (deg)*****"<<endl;
    cout<<"Roll : ";
    cout<<hovering_rp(0)*180.0/M_PI<<endl;
    cout<<"Pitch : ";
    cout<<hovering_rp(1)*180.0/M_PI<<endl;

    cout<<"*****Get Position Gain Parameter*****"<<endl;
    
    nh.getParam("Kp_pos",Kp_pos);
    nh.getParam("Kd_pos",Kd_pos);

    cout<<"Kp_pos Gain: "<<Kp_pos<<endl;
    cout<<"Kd_pos Gain: "<<Kd_pos<<endl;

    nh.getParam("Kp_ori",Kp_ori);
    nh.getParam("Kd_ori",Kd_ori);

    cout<<"Kp_ori Gain: "<<Kp_ori<<endl;
    cout<<"Kd_ori Gain: "<<Kd_ori<<endl;

    cout<<"*****Get Lift Parameter*****"<<endl;
    nh.getParam("C_lift",C_lift);
    cout<<"C lift : "<<C_lift<<endl;

    cout<<"*****Subscriber Setup*****"<<endl;

    cout<<"Desired Trajectory Subscriber Setup"<<endl;

    cout<<"Estimated Pose Subscriber Setup"<<endl;
    pose_subscriber = nh.subscribe("/mavros/local_position/odom",1,&CoaxCTRL::CallbackPose,this);

    cout<<"*****Publisher Setup*****"<<endl;
    cout<<"Throttle Publisher Setup"<<endl;
    throttle_publisher = nh.advertise<UInt16>("/throttle",1);
    
    cout<<"TVC roll pitch Publisher Setup"<<endl;
    tvc_publisher = nh.advertise<rollpitch>("/des_rp",1);

    cout<<"Rudder Publisher Setup"<<endl;
    rudder_publisher = nh.advertise<Int32>("/des_yaw",1);

    I_q_CM.setZero();
    I_q_CM(0) = 1;
    I_q_des.setZero();
    I_q_des(0) = 1;

}

void CoaxCTRL::CallbackDesPos(const Pose & des_p)
{
    I_p_des << des_p.position.x, 
            des_p.position.y, 
            des_p.position.z;
    
    I_q_des << des_p.orientation.w,
            des_p.orientation.x,
            des_p.orientation.y,
            des_p.orientation.z;

}

void CoaxCTRL::CallbackDesVel(const Twist & des_v)
{
    I_v_des << des_v.linear.x,
            des_v.linear.y,
            des_v.linear.z;
}

void CoaxCTRL::CallbackDesAcc(const Accel & des_a)
{
    I_a_des << des_a.linear.x,
            des_a.linear.y,
            des_a.linear.z;
}

void CoaxCTRL::CallbackPose(const Odometry & pose_msg)
{
    I_p_CM << pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y;
            pose_msg.pose.pose.position.z;
    
    I_q_CM << pose_msg.pose.pose.orientation.w,
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z;
    
    I_v_CM << pose_msg.twist.twist.linear.x,
            pose_msg.twist.twist.linear.y,
            pose_msg.twist.twist.linear.z;

    I_w << pose_msg.twist.twist.angular.x,
            pose_msg.twist.twist.angular.y,
            pose_msg.twist.twist.angular.z;
    
}

void CoaxCTRL::PosControl()
{
    u_pos = mass*(I_a_des
            + Kp_pos*(I_p_des - I_p_CM) 
            + Kd_pos*(I_v_des - I_v_CM)) - I_W_CM;
    
    thrust = sqrt(u_pos.transpose()*u_pos);
    throttle = thrust / gear_ratio;

    throttle_clamping(throttle);

    des_yaw = -2*atan2(qz_des,qw_des);

    roll_pitch_yaw(0) = asin(2*(qy*qz + qw*qx));
    
    roll_pitch_yaw(1) = atan2(2*(qx*qz - qw*qy)/cos(roll_pitch_yaw(0)),
    (1-2*qx*qx))/cos(roll_pitch_yaw(0));
    
    roll_pitch_yaw(2) = atan2(2*(qx*qz - qw*qy)/cos(roll_pitch_yaw(0)),
    (1-2*qx*qx))/cos(roll_pitch_yaw(0));

    if(thrust > 0){
        des_roll_pitch_yaw(0) = asin((u_pos(0)*cos(des_yaw)+u_pos(1)*sin(des_yaw))/thrust) + hovering_rp(0);
        des_roll_pitch_yaw(1) = asin((u_pos(0)*sin(des_yaw)-u_pos(1)*cos(des_yaw))/thrust/cos(des_roll_pitch_yaw(0))) 
        + hovering_rp(1);
        des_roll_pitch_yaw(2) = des_yaw;
    }
    else{
        des_roll_pitch_yaw << 0, 0, des_yaw;
    }

    des_rp_clamping(des_roll_pitch_yaw(0),des_roll_pitch_yaw(1));

}

void CoaxCTRL::OriControl()
{
    u_att = Kp_ori * (des_roll_pitch_yaw - roll_pitch_yaw) - Kd_ori*I_w;
    u_att(0) += eq_rp(0);
    u_att(1) += eq_rp(1);

    throttle_.data = throttle;
    tvc_rp_.roll = u_att(0);
    tvc_rp_.pitch = u_att(1);
    rudder_yaw_.data = u_att(2);

    throttle_publisher.publish(throttle_);
    tvc_publisher.publish(tvc_rp_);
    rudder_publisher.publish(rudder_yaw_);
}

void CoaxCTRL::throttle_clamping(uint8_t &throttle_ptr)
{
    if (throttle_ptr > throttle_max)
        throttle_ptr = throttle_max;
    
    if(throttle_ptr < throttle_min)
        throttle_ptr = throttle_min;

}

void CoaxCTRL::des_rp_clamping(double &des_roll_ptr, double &des_pitch_ptr)
{
    if (abs(des_roll_ptr) > des_roll_max)
        des_roll_ptr = des_roll_max * signum(des_roll_ptr);

    if (abs(des_pitch_ptr) > des_pitch_max)
        des_pitch_ptr = des_pitch_max * signum(des_pitch_ptr);
}

double CoaxCTRL::signum(double &sign_ptr)
{
    if (sign_ptr > 0)
        return 1.0; 
    if (sign_ptr < 0)
        return -1.0;
    return 0.0;
}

CoaxCTRL::~CoaxCTRL()
{

}