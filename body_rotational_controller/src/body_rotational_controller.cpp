#include "body_rotational_controller/body_rotational_controller.h"

BodyRotationalController::BodyRotationalController(void) : local_nh_("~")
{
    local_nh_.getParam("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE_);
    local_nh_.getParam("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE_);
    local_nh_.getParam("H", H_);
    local_nh_.getParam("L", L_);
    local_nh_.getParam("HZ", HZ_);
    local_nh_.getParam("DT", DT_);
    local_nh_.getParam("SMOOTHING_RATE", SMOOTHING_RATE_);

    roll_pitch_pub_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("/roll_pitch", 1);
    // odom_sub_ = nh_.subscribe("/odom", 1, &BodyRotationalController::odom_callback, this);
    // model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &BodyRotationalController::model_states_callback, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &BodyRotationalController::cmd_vel_callback, this);

}

// void BodyRotationalController::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     odom_ = *msg;
//     BEFORE_V_ = CURRENT_V_;
//     CURRENT_V_ = odom_.twist.twist.linear.x;
//     W_ = odom_.twist.twist.angular.z;
//     // ROS_ERROR_STREAM("V: " << V_ << ", W: " << W_);
// }

// void BodyRotationalController::model_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
// {
//     model_states_ = *msg;
//     BEFORE_V_ = CURRENT_V_;
//     CURRENT_V_ = model_states_.twist[1].linear.x;
//     W_ = model_states_.twist[1].angular.z;
//     // ROS_ERROR_STREAM("V: " << CURRENT_V_ << ", W: " << W_);
// }

void BodyRotationalController::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_ = *msg;
    BEFORE_V_ = CURRENT_V_;
    CURRENT_V_ = cmd_vel_.linear.x;
    W_ = cmd_vel_.angular.z;
    // ROS_ERROR_STREAM("V: " << CURRENT_V_ << ", W: " << W_);
}

double BodyRotationalController::calc_com_vel(double v, double w, double roll)
{
    return (v + L_*w*cos(roll));
}

double BodyRotationalController::calc_accell(double before_vel, double current_vel)
{

    return (current_vel - before_vel)/DT_;
}

double BodyRotationalController::calc_pitch(double accell)
{
    double theta = -2*(asin((accell*H_/9.81/L_)/sqrt(1 + pow(accell/9.81, 2))) + asin((accell/9.81)/sqrt(1 + pow(accell/9.81, 2))));
    if(theta > MAX_PITCH_ANGLE_)
    {
        theta = MAX_PITCH_ANGLE_;
    }
    else if(theta < -MAX_PITCH_ANGLE_)
    {
        theta = -MAX_PITCH_ANGLE_;
    }
    return theta;
}

double BodyRotationalController::calc_roll(double v, double w)
{
    double theta = -(asin((v*w*H_)/(9.81*L_*sqrt(1 + pow(v*w*L_/9.81, 2)))) + asin((v*w*L_/9.81)/sqrt(1 + pow(v*w*L_/9.81, 2))));
    if(theta > MAX_ROLL_ANGLE_)
    {
        theta = MAX_ROLL_ANGLE_;
    }
    else if(theta < -MAX_ROLL_ANGLE_)
    {
        theta = -MAX_ROLL_ANGLE_;
    }
    return theta;
}

void BodyRotationalController::publish_attitude(double roll, double pitch)
{
    ros::Time now = ros::Time::now();
    roll_pitch_.roll = roll;
    roll_pitch_.pitch = pitch;
    roll_pitch_pub_.publish(roll_pitch_);
}

void BodyRotationalController::process(void)
{
    ros::Rate loop_rate(HZ_);
    while (ros::ok())
    {
        double v = BodyRotationalController::calc_com_vel(CURRENT_V_, W_, ROLL_);
        double accell = BodyRotationalController::calc_accell(BEFORE_V_, CURRENT_V_);
        PITCH_ = BodyRotationalController::calc_pitch(accell);
        ROLL_ = BodyRotationalController::calc_roll(v, W_);
        ROS_INFO_STREAM("PITCH: " << PITCH_ << ", ACCELL: " << accell);
        ROS_WARN_STREAM("ROLL: " << ROLL_ << ", V: " << v << ", W: " << W_);
        BodyRotationalController::publish_attitude(ROLL_, PITCH_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

