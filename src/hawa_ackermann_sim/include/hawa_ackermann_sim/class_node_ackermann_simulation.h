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
 * @file class_node_ackermann_simulation.h
 * @author Mingjie
 * @brief This is the ros node of running the simulated robot.
 * @version 0.3
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_CLASS_NODE_ACKERMANN_SIMULATION
#define HAWA_CLASS_NODE_ACKERMANN_SIMULATION

#include <iostream>
#include <vector>
#include <string>
#include <mutex>

// #include <random>
// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/json_parser.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "class_car_body_box_marker.h"
#include "hawa_sim_tools.h"
#include "class_akm_sim_dynamic_states.h"
#include "class_random_sin_noise_generator.h"


namespace hawa
{
    
using namespace std::chrono_literals;
using std::placeholders::_1;

class ClassNodeAckermannSim : public rclcpp::Node
{
private:

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_bcr_;
    
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_sub_ptr_akm_drive_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_ptr_odometry_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_ptr_car_body_vis_;
    rclcpp::TimerBase::SharedPtr m_periodic_timer_;

    std::string m_robot_frame_;
    std::string m_topic_name_akm_drive_; 
    std::string m_topic_name_odometry_;  
    std::string m_topic_name_car_body_vis_; 

    ackermann_msgs::msg::AckermannDriveStamped m_akm_drive_msg_;
    double m_akm_drive_msg_stamp_system_;
    nav_msgs::msg::Odometry m_odometry_msg_;

    ClassCarBox2D m_car_box_;

    std::unique_ptr<ClassRandomSinNoiseGenerator> m_noise_generator_;

    bool FLAG_use_noise_ = true;

private:
    DynamicsStates m_dyna_states_;
    double m_timestamp_last_update_;
    double m_main_loop_timer_interval_second_;
    bool FLAG_pub_tf__map2odom_;
    
private:
    void akmDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg);
    void mainUpdate( );
    void loadParameters();

public:
    ClassNodeAckermannSim();
    ~ClassNodeAckermannSim();
};


}


#endif
