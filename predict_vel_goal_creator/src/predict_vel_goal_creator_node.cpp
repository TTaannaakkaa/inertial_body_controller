#include "predict_vel_goal_creator/predict_vel_goal_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "predict_vel_goal_creator");
    LocalGoalCreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}