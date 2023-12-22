#ifndef BODY_ROTATIONAL_CONTROLLER_H
#define BODY_ROTATIONAL_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <sq2_ccv_roll_pitch_msgs/RollPitch.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include <math.h>
#include <vector>


class BodyRotationalController
{
public:
    BodyRotationalController();

    void process();

private:

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void calc_attitude(double vel, double yaw_rate, double accell);
    void publish_attitude(double roll, double pitch);

    double calc_com_vel(double vel, double yaw_rate, double roll);
    double calc_roll(double vel, double yaw_rate);
    double calc_accell(double before_vel, double current_vel);
    double calc_pitch(double accell);
    double smoothing_vel(std::vector<double> vel_list, double smoothing_rate);

    double MAX_ROLL_ANGLE_;
    double MAX_PITCH_ANGLE_;
    double H_;
    double L_;
    double HZ_;
    double DT_;

    double CURRENT_V_;
    double BEFORE_V_;
    double W_;
    double ROLL_;
    double PITCH_;
    double SMOOTHING_RATE_;
    double bv2_, bv3_, bv4_, bv5_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher roll_pitch_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber model_states_sub_;

    nav_msgs::Odometry odom_;
    gazebo_msgs::ModelStates model_states_;
    geometry_msgs::Twist cmd_vel_;
    sq2_ccv_roll_pitch_msgs::RollPitch roll_pitch_;
};

#endif // BODY_ROTATIONAL_CONTROLLER_H