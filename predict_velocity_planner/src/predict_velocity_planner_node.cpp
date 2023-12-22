// Copyright 2023 amsl

#include "predict_velocity_planner/predict_velocity_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "predict_velocity_planner");
    DWAPlanner planner;
    planner.process();
    return 0;
}
