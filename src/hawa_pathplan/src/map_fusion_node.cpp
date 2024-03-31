
#include "../include/map_fusion/class_map_fusion.h"

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief The main function of the map fusion node.
*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ClassMapFusion>());

    rclcpp::shutdown();

    return 0;

    // ros::init(argc, argv, "map_fusion_node");

    // ros::NodeHandle n;

    // ClassMapFusion map_fusor( n );

    // // ros::spin();
    // ros::AsyncSpinner s(2);
    // s.start();

    // ros::waitForShutdown();

    // return 0;
}





