#include "coax_control.h"

CoaxCTRL::CoaxCTRL()
{
    // Constructor
    cout<<"Mass Parameter Setup"<<endl;
    nh.getParam("mass",mass);
    I_W_CM << 0, 0, mass*g;

    cout<<"COM Offset Parameter Setup"<<endl;
    nh.getParam("x_com_off",x_com_off);
    nh.getParam("y_com_off",y_com_off);
    nh.getParam("z_com_off",z_com_off);

    cout<<"***Get Position Gain Parameter***"<<endl;
    
    nh.getParam("Kp_pos",Kp_pos);
    nh.getParam("Kd_pos",Kd_pos);

    cout<<"Kp_pos Gain: "<<Kp_pos<<endl;
    cout<<"Kd_pos Gain: "<<Kd_pos<<endl;

    nh.getParam("Kp_ori",Kp_ori);
    nh.getParam("Kd_ori",Kd_ori);

    cout<<"Kp_ori Gain: "<<Kp_ori<<endl;
    cout<<"Kd_ori Gain: "<<Kd_ori<<endl;

    nh.getParam("C_lift",C_lift);
    
    cout<<"***Get Lift Parameter***"<<endl;

    cout<<"C lift : "<<C_lift<<endl;

    cout<<"***Subscriber Setup***"<<endl;

    cout<<"Desired Trajectory Subscriber Setup"<<endl;

    cout<<"Estimated Pose Subscriber Setup"<<endl;
    pose_subscriber = nh.subscribe("/mavros/local_position/odom",1,&CoaxCTRL::CallbackPose,this);

    cout<<"Throttle Publisher Setup"<<endl;
    throttle_publisher = nh.advertise<Int32>("/throttle",1);
    
    cout<<"TVC roll pitch Publisher Setup"<<endl;
    tvc_publisher = nh.advertise<lm4075e_msgs::Int32>("/des_rp",1);

    cout<<"Rudder Publisher Setup"<<endl;
    rudder_publisher = nh.advertise<Int32>("/des_yaw",1);


}

void CoaxCTRL::CallbackDesPos(const Pose & des_p)
{
    I_p_des << des_p.position.x, 
            des_p.position.y, 
            des_p.position.z;
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
}

void CoaxCTRL::PosControl()
{
    u_pos = I_a_des 
            + Kp_pos*(I_p_des - I_p_CM) 
            + Kd_pos*(I_v_des - I_v_CM) - I_W_CM;
}