#ifndef PREDICT_VEL_GOAL_CREATOR_H
#define PREDICT_VEL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class PredictVelPlanner
{
public:
    PredictVelPlanner();
    void process();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishCurGoal();
    void publishPreGoal();
    double getCurDistance();
    double getPreDistance();

    int hz_;
    int index_step_;
    int cur_goal_index_;
    int pre_goal_index_;
    double cur_taeget_distance_;
    double pre_taeget_distance_;
    bool is_path_ = false;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher cur_local_goal_pub_;
    ros::Publisher pre_local_goal_pub_;

    nav_msgs::Path path_;
    geometry_msgs::PoseStamped cur_goal_;
    geometry_msgs::PoseStamped pre_goal_;
    geometry_msgs::PoseStamped pose_;
};

#endif // PREDICT_VEL_GOAL_CREATOR_H