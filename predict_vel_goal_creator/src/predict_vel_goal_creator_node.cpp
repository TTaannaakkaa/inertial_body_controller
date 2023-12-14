#include "predict_vel_goal_creator/predict_vel_goal_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "predict_vel_goal_creator");
    PredictVelPlanner rredictvelplanner;
    rredictvelplanner.process();

    return 0;
}