#include "predict_vel_goal_creator/predict_vel_goal_creator.h"

PredictVelPlanner::PredictVelPlanner() : private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("index_step", index_step_);
    private_nh_.getParam("cur_target_dist_to_goal", cur_taeget_distance_);
    private_nh_.getParam("pre_target_dist_to_goal", pre_taeget_distance_);
    private_nh_.getParam("cur_goal_index", cur_goal_index_);
    private_nh_.getParam("pre_goal_index", pre_goal_index_);

    cur_goal_.header.frame_id = "map";
    pre_goal_.header.frame_id = "map";
    cur_goal_.pose.orientation.w = 1.0;
    pre_goal_.pose.orientation.w = 1.0;

    path_sub_ = nh_.subscribe("/global_path", 1, &PredictVelPlanner::pathCallback, this);
    pose_sub_ = nh_.subscribe("/estimated_pose", 1, &PredictVelPlanner::poseCallback, this);
    cur_local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/cur_local_goal", 1);
    pre_local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pre_local_goal", 1);
}

void PredictVelPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_ = *msg;
}

void PredictVelPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_ = *msg;
    is_path_ = true;
}

void PredictVelPlanner::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(is_path_)
        {
            publishCurGoal();
            publishPreGoal();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PredictVelPlanner::publishCurGoal()
{
    double distance = getCurDistance();
    if(distance < cur_taeget_distance_)
    {
        cur_goal_index_ += index_step_;
        if(cur_goal_index_ >= path_.poses.size())
        {
            cur_goal_index_ = path_.poses.size() - 1;
        }
    }
    cur_goal_.pose.position.x = path_.poses[cur_goal_index_].pose.position.x;
    cur_goal_.pose.position.y = path_.poses[cur_goal_index_].pose.position.y;
    cur_goal_.header.stamp = ros::Time::now();
    cur_local_goal_pub_.publish(cur_goal_);
}

double PredictVelPlanner::getCurDistance()
{
    double dx = path_.poses[cur_goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[cur_goal_index_].pose.position.y - pose_.pose.position.y;
    return hypot(dx, dy);
}

void PredictVelPlanner::publishPreGoal()
{
    double distance = getPreDistance();
    if(distance < pre_taeget_distance_)
    {
        pre_goal_index_ += index_step_;
        if(pre_goal_index_ >= path_.poses.size())
        {
            pre_goal_index_ = path_.poses.size() - 1;
        }
    }
    pre_goal_.pose.position.x = path_.poses[pre_goal_index_].pose.position.x;
    pre_goal_.pose.position.y = path_.poses[pre_goal_index_].pose.position.y;
    pre_goal_.header.stamp = ros::Time::now();
    pre_local_goal_pub_.publish(pre_goal_);
}

double PredictVelPlanner::getPreDistance()
{
    double dx = path_.poses[pre_goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[pre_goal_index_].pose.position.y - pose_.pose.position.y;
    return hypot(dx, dy);
}

