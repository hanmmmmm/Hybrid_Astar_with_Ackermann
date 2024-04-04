
#include "class_map_fusion.h"

/**
 * @brief The main function of the map fusion node.
*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<hawa::ClassMapFusion>());

    rclcpp::shutdown();

    return 0;

}
