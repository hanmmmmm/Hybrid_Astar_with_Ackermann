

#include "../include/planner/class_path_planner.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_plan_node");

    ros::NodeHandle n;

    ClassPathPlanner planner( n );

    ros::AsyncSpinner s(4);
    s.start();

    ros::waitForShutdown();

    return 0;
}





