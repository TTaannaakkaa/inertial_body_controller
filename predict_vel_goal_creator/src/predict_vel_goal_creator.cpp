#include "predict_vel_goal_creator/predict_vel_goal_creator.h"

LocalGoalCreator::LocalGoalCreator() : private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("index_step", index_step_);
    private_nh_.getParam("target_dist_to_goal", taeget_distance_);
    private_nh_.getParam("goal_index", goal_index_);

    goal_.header.frame_id = "map";

    path_sub_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::pathCallback, this);
    pose_sub_ = nh_.subscribe("/estimated_pose", 1, &LocalGoalCreator::poseCallback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

void LocalGoalCreator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_ = *msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_ = *msg;
    is_path_ = true;
}

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(is_path_)
        {
            publishGoal();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LocalGoalCreator::publishGoal()
{
    double distance = getDistance();
    // std::cout << "distance: " << distance << std::endl;
    if(distance < taeget_distance_)
    {
        goal_index_ += index_step_;
        if(goal_index_ >= path_.poses.size())
        {
            goal_index_ = path_.poses.size() - 1;
        }
    }
    goal_.point.x = path_.poses[goal_index_].pose.position.x;
    goal_.point.y = path_.poses[goal_index_].pose.position.y;
    // std::cout << "goal_index: " << goal_index_ << std::endl;
    // std::cout << "goal_x: " << goal_.point.x << std::endl;
    // std::cout << "goal_y: " << goal_.point.y << std::endl;
    goal_.header.stamp = ros::Time::now();
    local_goal_pub_.publish(goal_);
}

double LocalGoalCreator::getDistance()
{
    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    return hypot(dx, dy);
}

