
#include "lqr/class_node_lqr.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ClassNodeLQR>());

    rclcpp::shutdown();

    return 0;
}






