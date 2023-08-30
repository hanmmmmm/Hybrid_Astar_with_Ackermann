
#include "../include/computer_load/class_pc_load.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_load_node");

    ros::NodeHandle n;

    ClassPcLoad pl( n );

    pl.get_ram_free_percent();
    

    return 0;
}


