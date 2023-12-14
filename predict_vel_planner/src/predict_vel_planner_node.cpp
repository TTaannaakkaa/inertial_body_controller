#include "predict_vel_planner/predict_vel_planner.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "predict_vel_planner");
    DWAPlanner dwa_planner;
    dwa_planner.process();
    return 0;
}