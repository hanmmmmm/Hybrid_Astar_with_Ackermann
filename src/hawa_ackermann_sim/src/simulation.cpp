
#include "../include/hawa_ackermann_sim/class_node_ackermann_simulation.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "akm_sim_node");

    ros::NodeHandle n;

    ClassNodeAckermannSim simer( n );

    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}




