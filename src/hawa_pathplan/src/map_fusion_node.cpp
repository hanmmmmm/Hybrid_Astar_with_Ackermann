
#include "../include/map_fusion/class_map_fusion.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_fusion_node");

    ros::NodeHandle n;

    ClassMapFusion map_fusor( n );

    // ros::spin();
    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}





