#include "body_rotational_controller/body_rotational_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_rotational_controller_node");
    BodyRotationalController body_rotational_controller;
    body_rotational_controller.process();
    return 0;
}