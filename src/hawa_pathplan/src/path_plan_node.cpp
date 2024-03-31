// MIT License

// Copyright (c) 2023 Mingjie

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * This file is the entry point of the path planner module.
*/

#include "rclcpp/rclcpp.hpp"

#include "../include/planner/class_path_planner.h"

/**
 * @brief Entry point.
*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ClassPathPlanner>());

    rclcpp::shutdown();

    return 0;

    // ros::init(argc, argv, "path_plan_node");

    // ros::NodeHandle n;

    // ClassPathPlanner planner( n );

    // ros::AsyncSpinner s(4);
    // s.start();

    // ros::waitForShutdown();

    // return 0;
}





