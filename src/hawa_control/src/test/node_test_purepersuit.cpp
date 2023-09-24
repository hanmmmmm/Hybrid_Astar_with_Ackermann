
#include <iostream>
#include "ros/ros.h"
#include "pure_pursuit/class_node_test_pure_pursuit.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "purepursuit_test_node");

    ros::NodeHandle n;

    ClassNodeTestPurePursuit oo( n );

    // ros::spin();
    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}







